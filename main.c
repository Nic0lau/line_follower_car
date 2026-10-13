/*
 * ATmega328P Line follower
 *
 * The MCU has two TCRT5000 LM393 infrared sensors used to detect the black
 * line on the white background. It uses four motors controlled by two L298N
 * drivers, one for the left motors and other for the right ones.
 *
 * The two sensors are plugged into PC0/ADC0/A0 and PC1/ADC1/A1.
 * The motor velocities are plugged into PB3/OC2A/D11, PD3/OC2B/D3,
 * PD5/OC0B/D5 and PD6/OC0A/D6.
 * On the left L298N, In1 is tied to PD4/D4, In2 to PD7/D7, In3 to PB0/D8 and
 * In4 to PB1/D9.
 * On the right L298N, In1 is tied to PB0/D8, In2/In4 PB1/D9, In3 to PD4/D4 and
 * In4 to PD7/D7.
 *
 * Notice the careful wiring to bind the pairs of the front motors direction
 * pins together and to do the same with the rear ones. Also notice the code
 * supposes the sensors read higher values for white and lower values for black.
 *
 * PID uses Q8.8 numbers for the constants.
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>

#define FRONT_LEFT_MOTOR	PD6
#define FRONT_RIGHT_MOTOR	PB3
#define REAR_LEFT_MOTOR		PD5 
#define REAR_RIGHT_MOTOR	PD3

#define FRONT_DIRECTION_A	PD4
#define FRONT_DIRECTION_B	PD7
#define REAR_DIRECTION_A	PB0
#define REAR_DIRECTION_B	PB1

#define SENSOR_LEFT	0       /* PC0/ADC0 */
#define SENSOR_RIGHT	1       /* PC1/ADC1 */

#define BASE_SPEED      180
#define MAX_DUTY        245
#define MIN_DUTY        0

#define KP              40
#define KI              0
#define KD              30
#define INTEGRAL_CLAMP  10000

static uint16_t adc_read_raw(uint8_t channel);
static uint16_t adc_read(uint8_t channel);
static int16_t pid_step(int16_t error);
static uint8_t clamp_u8(int32_t v, int32_t low, int32_t high);

static uint16_t
adc_read_raw(uint8_t channel)
{
	ADMUX = (uint8_t)((ADMUX & 0xF0) | (channel & 0x07));
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

static uint16_t
adc_read(uint8_t channel)
{
	(void) adc_read_raw(channel);
	return adc_read_raw(channel);
}

static int16_t
pid_step(int16_t error)
{
	static int32_t integral;
	static int16_t previous_error;
	int32_t out;
	int16_t derivative;

	integral += error;
	if (integral >  INTEGRAL_CLAMP)
		integral =  INTEGRAL_CLAMP;
	if (integral < -INTEGRAL_CLAMP)
		integral = -INTEGRAL_CLAMP;

	derivative = (int16_t)(error - previous_error);
	previous_error = error;

	out = ((int32_t)KP * error) + ((int32_t)KI * integral) + ((int32_t)KD * derivative);

	return (int16_t)(out >> 8);
}

static uint8_t
clamp_u8(int32_t v, int32_t low, int32_t high)
{
	if (v < low)
		return (uint8_t)low;
	if (v > high)
		return (uint8_t)high;
	return (uint8_t)v;
}

int
main(void)
{
	int32_t left_speed, right_speed;
	uint16_t left_sensor, right_sensor;
	int16_t correction, error;

	/* ADC init */
	ADMUX  = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	/* Dummy ADC first conversion */
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	/* PWM init */

	/* Timer0 */
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01)  | (1 << CS00);              /* /64 -> ~976 Hz */
	OCR0A = 0;
	OCR0B = 0;

	/* Timer2 */
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (1 << CS22);                             /* /64 -> ~976 Hz */
	OCR2A = 0;
	OCR2B = 0;

	/* Set PWM pins as outputs */
	DDRD |= (1 << FRONT_LEFT_MOTOR) | (1 << REAR_LEFT_MOTOR) | (1 << REAR_RIGHT_MOTOR);
	DDRB |= (1 << FRONT_RIGHT_MOTOR);

	/* Setting direction pairs as outputs */
	DDRD |= (1 << FRONT_DIRECTION_A) | (1 << FRONT_DIRECTION_B);
	DDRB |= (1 << REAR_DIRECTION_A)  | (1 << REAR_DIRECTION_B);

	/* Setting A high and B low, making the motors go forward */
	PORTD |=  (1 << FRONT_DIRECTION_A);
	PORTD &= ~(1 << FRONT_DIRECTION_B);
	PORTB |=  (1 << REAR_DIRECTION_A);
	PORTB &= ~(1 << REAR_DIRECTION_B);

	for (;;) {
		left_sensor = adc_read(SENSOR_LEFT);
		right_sensor = adc_read(SENSOR_RIGHT);

		error = (int16_t)right_sensor - (int16_t)left_sensor;

		correction = pid_step(error);

		left_speed = (int32_t)BASE_SPEED - correction;
		right_speed = (int32_t)BASE_SPEED + correction;

		OCR0A = clamp_u8(left_speed, MIN_DUTY, MAX_DUTY);
		OCR0B = clamp_u8(left_speed, MIN_DUTY, MAX_DUTY);

		OCR2A = clamp_u8(right_speed, MIN_DUTY, MAX_DUTY);
		OCR2B = clamp_u8(right_speed, MIN_DUTY, MAX_DUTY);
	}
}
