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

#include <avr/io.h>
#include <stdint.h>

#define LEFT_MOTORS_SPEED	PD3
#define RIGHT_MOTORS_SPEED	PB3

#define LEFT_DIRECTION_A	PD2
#define LEFT_DIRECTION_B	PD4
#define RIGHT_DIRECTION_A	PD5
#define RIGHT_DIRECTION_B	PD6

#define SENSOR_LEFT	0       /* PC0/ADC0 */
#define SENSOR_RIGHT	1       /* PC1/ADC1 */

#define BASE_SPEED      100
#define MAX_DUTY        245
#define MIN_DUTY        0

#define KP              500
#define KI              0
#define KD              100
#define INTEGRAL_CLAMP  10000

static uint16_t adc_read_raw(uint8_t channel);
static uint16_t adc_read(uint8_t channel);
static int16_t pid(int16_t error);
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
pid(int16_t error)
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

	/* Timer2 */
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

	/* Setting A high and B low, making the motors go forward */
	PORTD |=  (1 << LEFT_DIRECTION_A) | (1 << RIGHT_DIRECTION_A);
	PORTD &= ~((1 << LEFT_DIRECTION_B)  | (1 << RIGHT_DIRECTION_B));

	for (;;) {
		left_sensor = adc_read(SENSOR_LEFT);
		right_sensor = adc_read(SENSOR_RIGHT);

		error = (int16_t)right_sensor - (int16_t)left_sensor;

		correction = pid(error);

		left_speed = (int32_t)BASE_SPEED + correction;
		right_speed = (int32_t)BASE_SPEED - correction;

		OCR2B = clamp_u8(left_speed, MIN_DUTY, MAX_DUTY);
		OCR2A = clamp_u8(right_speed, MIN_DUTY, MAX_DUTY);
	}
}
