#include "motor.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "usart.h"
#include "util/delay.h"

bool reverseMotor = false;
bool motorStoped = false;

ISR(TIMER2_COMPB_vect)
{
    if (reverseMotor)
    {
        PORTC &= ~_BV(PC3);
        return;
    }

    PORTC &= ~_BV(PC1);
}

ISR(TIMER2_COMPA_vect)
{
    if (reverseMotor)
    {
        PORTC |= _BV(PC3);
        return;
    }

    PORTC |= _BV(PC1);
}

/**
 * @brief Enable PWM outputs for dc motor control
 * @note PC0 - Forward Latch
 * @note PC1 - Forward PWM
 * @note PC2 - Reverse Latch
 * @note PC3 - Reverse PWM
 */
void configureMotorPWM(void)
{
    // Setup PC0, PC1, PC2, and PC3 as outputs. and turns everything off
    PORTC = 0;
    DDRC = (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3);

    // Enable PWM with a frequency of 5khz. OC2A and OC2B are disabled.
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << WGM22);
    TIMSK2 = _BV(OCIE2B) | _BV(OCIE2A);

    // Set top value to 199 for a frequency of 5khz
    OCR2A = 199;
    OCR2B = 1;
}

void configureSteeringPWM(void)
{
    // Set OC1B pin as output — OC1B is PD4 on ATmega1284
    DDRD |= (1 << PD4);

    // Configure Timer1 for Fast PWM mode 14 (WGM13:0 = 1110)
    // COM1B1 = 1, COM1B0 = 0 for non-inverting PWM on OC1B
    TCCR1A = (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // prescaler 8

    // Set TOP value for 50 Hz
    ICR1 = 20000;

    // Set pulse width for neutral servo position (~1.5 ms)
    OCR1B = 1450;
}

void stopMotor(void)
{
    // Turn off timer 2
    TCCR2B = (1 << WGM22);

    // Turn off all motor outputs
    PORTC = 0;

    motorStoped = true;
    PORTD &= ~_BV(PD5);
}

void setMotorSpeed(int8_t speed)
{
    OCR2B = reverseMotor ? -speed : speed;
}

void setMotorDirection(bool forward)
{
    // Turn off timer 2
    TCCR2B = (1 << WGM22);

    // Turn off all motor outputs
    PORTC = 0;

    // Wait 1 second for the outputs to turn off
    _delay_ms(150);

    // Turn on our reverse latch
    PORTC = forward ? _BV(PC0) : _BV(PC2);

    reverseMotor = !forward;
    TCCR2B = (1 << WGM22) | (1 << CS21); // Prescaler = 8
    motorStoped = false;
    PORTD |= _BV(PD5);
}