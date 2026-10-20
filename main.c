/*
 * ATmega328P Line follower
 *
 * The MCU has two TCRT5000 LM393 infrared sensors used to detect the black
 * line on the white background. It uses four motors controlled by two L298N
 * drivers, one for the left motors and other for the right ones.
 *
 * The two sensors are plugged into PC0/ADC0/A0 and PC1/ADC1/A1.
 * The motor velocities are plugged into PB3/OC2A/D11 and PD3/OC2B/D3.
 * On the left L298N, In1/In3 are tied to PD2/D2 and In2/In4 to PD4/D4.
 * On the right L298N, In1/In3 are tied to PD5/D5 and In2/In4 to PD6/D6.
 *
 * Notice the careful wiring to bind the pairs of the left direction pins
 * together and to do the same with the right ones. Also notice the code
 * supposes the sensors read lower values for white and higher for black.
 *
 * PID uses Q8.8 numbers for the constants.
*/

#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#define LEFT_MOTORS_SPEED	PD3
#define RIGHT_MOTORS_SPEED	PB3

#define LEFT_DIRECTION_A	PD2
#define LEFT_DIRECTION_B	PD4
#define RIGHT_DIRECTION_A	PD5
#define RIGHT_DIRECTION_B	PD6

#define SENSOR_LEFT		0       /* PC0/ADC0 */
#define SENSOR_RIGHT		1       /* PC1/ADC1 */

#define CALIBRATION_DURATION	1500 /* Measured in ticks of 2ms; 2ms * 1500 = 3s */
#define CALIBRATION_SPEED	200

#define BASE_SPEED      	200
#define SHARP_TURN_SPEED 	225
#define MAX_DUTY        	245
#define MIN_DUTY        	0

#define BLACK_THRESHOLD			800
#define WHITE_THRESHOLD			150
#define SHARP_CURVE_THRESHOLD		20000 /* Currently disabled */
#define CENTER_THRESHOLD		10
#define TURN_DELAY_DURATION		50 /* Measured in ticks of 2ms; 2ms * 50 = 100ms */

#define KP             	750
#define KI              20
#define KD             	50
#define INTEGRAL_CLAMP  5000

#define CLAMP(v, low, high)  ((v) < (low) ? (low) : ((v) > (high) ? (high) : (v)))

enum finite_state_machine {
	STATE_FOLLOWING_LINE,
	STATE_STARTING_SHARP_CURVE,
	STATE_DOING_SHARP_CURVE
};

static uint16_t adc_read(uint8_t channel);
static int16_t pid(int16_t error);

static int32_t pid_integral;
static int16_t pid_previous_error;
static uint16_t calibration_min[2];
static uint16_t calibration_max[2];
static volatile uint8_t tick_flag;

ISR(TIMER0_COMPA_vect)
{
	tick_flag = 1;
}

static uint16_t
adc_read(uint8_t channel)
{
	uint16_t s1, s2, s3, temp;

	ADMUX = (uint8_t)((ADMUX & 0xF0) | (channel & 0x07));

	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	s1 = ADC;

	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	s2 = ADC;

	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	s3 = ADC;

	if (s1 > s2) {
		temp = s1;
		s1 = s2;
		s2 = temp;
	}
	if (s2 > s3) {
		temp = s2;
		s2 = s3;
		s3 = temp;
	}
	if (s1 > s2) {
		temp = s1;
		s1 = s2;
		s2 = temp;
	}

	return s2;
}

static int16_t
pid(int16_t error)
{
	int32_t out;
	int16_t derivative;

	pid_integral += error;
	if (pid_integral >  INTEGRAL_CLAMP)
		pid_integral =  INTEGRAL_CLAMP;
	if (pid_integral < -INTEGRAL_CLAMP)
		pid_integral = -INTEGRAL_CLAMP;

	derivative = (int16_t)(error - pid_previous_error);
	pid_previous_error = error;

	out = ((int32_t)KP * error) + ((int32_t)KI * pid_integral) + ((int32_t)KD * derivative);

	return (int16_t)(out >> 8);
}

static uint16_t
normalize(uint16_t raw, uint8_t sensor)
{
	int32_t v;

	v = (((int32_t)raw - (int32_t)calibration_min[sensor]) * 1000) /
		((int32_t)calibration_max[sensor] - (int32_t)calibration_min[sensor]);

	if (v < 0)
		v = 0;
	if (v > 1000)
		v = 1000;

	return (uint16_t)v;
}

int
main(void)
{
	enum finite_state_machine state;
	int32_t left_speed, right_speed;
	uint16_t left_sensor, right_sensor, turn_delay_counter;
	int16_t correction, calibration_counter, error, last_error;
	uint8_t line_seem;

	/* ADC init */
	ADMUX  = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	/* Dummy ADC first conversion */
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	/* Timer0 init for 2 ms fixed loop tick */
	TCCR0A = (1 << WGM01);                  
	TCCR0B = (1 << CS02);                   
	OCR0A = 124;                            
	TIMSK0 = (1 << OCIE0A);                 
	sei();                                  

	/* Timer2, used for motor speeds */
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (1 << CS22);                             /* /64 -> ~976 Hz */
	OCR2A = 0;
	OCR2B = 0;

	/* Set PWM pins as outputs */
	DDRD |= (1 << LEFT_MOTORS_SPEED);
	DDRB |= (1 << RIGHT_MOTORS_SPEED);

	/* Setting direction pairs as outputs */
	DDRD |= (1 << LEFT_DIRECTION_A) | (1 << LEFT_DIRECTION_B) |
		(1 << RIGHT_DIRECTION_A) | (1 << RIGHT_DIRECTION_B);

	/* Calibrating sensors */

	/* Turn counter-clockwise */
	PORTD |=  (1 << LEFT_DIRECTION_B) | (1 << RIGHT_DIRECTION_A);
	PORTD &= ~((1 << LEFT_DIRECTION_A)  | (1 << RIGHT_DIRECTION_B));

	OCR2B = CALIBRATION_SPEED;
	OCR2A = CALIBRATION_SPEED;

	calibration_min[0] = 1023;
	calibration_min[1] = 1023;
	calibration_max[0] = 0;
	calibration_max[1] = 0;

	for (tick_flag = 0, calibration_counter = CALIBRATION_DURATION; calibration_counter > 0; calibration_counter--) {
		while (!tick_flag);
		tick_flag = 0;

		left_sensor = adc_read(SENSOR_LEFT);
		right_sensor = adc_read(SENSOR_RIGHT);

		if (left_sensor < calibration_min[0])
			calibration_min[0] = left_sensor;
		if (left_sensor > calibration_max[0])
			calibration_max[0] = left_sensor;
		if (right_sensor < calibration_min[1])
			calibration_min[1] = right_sensor;
		if (right_sensor > calibration_max[1])
			calibration_max[1] = right_sensor;
	}

	if (calibration_max[0] - calibration_min[0] < 200)
		calibration_max[0] = calibration_min[0] + 200;
	if (calibration_max[1] - calibration_min[1] < 200)
		calibration_max[1] = calibration_min[1] + 200;

	/* Main loop */
	for (tick_flag = 0, state = STATE_FOLLOWING_LINE;;) {
		while (!tick_flag);
		tick_flag = 0;

		left_sensor = normalize(adc_read(SENSOR_LEFT), 0);
		right_sensor = normalize(adc_read(SENSOR_RIGHT), 1);

		error = (int16_t)right_sensor - left_sensor;

		switch (state) {
		case STATE_FOLLOWING_LINE:
			/* Go forward */
			PORTD |=  (1 << LEFT_DIRECTION_A) | (1 << RIGHT_DIRECTION_A);
			PORTD &= ~((1 << LEFT_DIRECTION_B)  | (1 << RIGHT_DIRECTION_B));

			if (error < -SHARP_CURVE_THRESHOLD || error > SHARP_CURVE_THRESHOLD || (left_sensor > BLACK_THRESHOLD && right_sensor > BLACK_THRESHOLD)) {
				state = STATE_STARTING_SHARP_CURVE;
				turn_delay_counter = TURN_DELAY_DURATION;
				last_error = (error > -SHARP_CURVE_THRESHOLD && error < SHARP_CURVE_THRESHOLD) ?
					     ((left_sensor > right_sensor) ? -SHARP_CURVE_THRESHOLD : SHARP_CURVE_THRESHOLD) : error;
				break;
			}

			correction = pid(error);

			left_speed = (int32_t)BASE_SPEED + correction;
			right_speed = (int32_t)BASE_SPEED - correction;

			OCR2B = CLAMP(left_speed, MIN_DUTY, MAX_DUTY);
			OCR2A = CLAMP(right_speed, MIN_DUTY, MAX_DUTY);
			break;
		case STATE_STARTING_SHARP_CURVE:
			if (turn_delay_counter > 0) {
				turn_delay_counter--;
			} else {
				state = STATE_DOING_SHARP_CURVE;
				pid_integral = 0;
				pid_previous_error = 0;
				line_seem = 0;

				if (last_error < -SHARP_CURVE_THRESHOLD) {
					/* Turn left */
					PORTD |=  (1 << LEFT_DIRECTION_B) | (1 << RIGHT_DIRECTION_A);
					PORTD &= ~((1 << LEFT_DIRECTION_A)  | (1 << RIGHT_DIRECTION_B));
				} else {
					/* Turn right */
					PORTD |=  (1 << LEFT_DIRECTION_A) | (1 << RIGHT_DIRECTION_B);
					PORTD &= ~((1 << LEFT_DIRECTION_B)  | (1 << RIGHT_DIRECTION_A));
				}

				OCR2A = SHARP_TURN_SPEED;
				OCR2B = SHARP_TURN_SPEED;
			}
			break;
		case STATE_DOING_SHARP_CURVE:
			if (left_sensor > BLACK_THRESHOLD || right_sensor > BLACK_THRESHOLD)
				line_seem = 1;
			if (error > -CENTER_THRESHOLD && error < CENTER_THRESHOLD && line_seem) {
				state = STATE_FOLLOWING_LINE;
				pid_integral = 0;
				pid_previous_error = 0;
			}
			break;
		}
	}
}
