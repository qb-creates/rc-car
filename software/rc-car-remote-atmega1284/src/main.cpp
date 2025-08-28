#include "joystick.h"
#include "rf.h"
#include "usart.h"
#include <RF24.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#define BOOST_TIME 2000
#define BOOST_COOLDOWN_TIME 4000

bool boostEnabled = false;
bool boostCooldown = false;
int16_t boostCounter = 0;
int16_t boostCooldownCounter = 0;

void initializeBoostTimer(void)
{
    // CTC mode (WGM32 = 1)
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS31); // CTC, prescaler = 8

    // gives us a 1ms delay
    OCR3A = 999;

    // Disable all interrupts
    TIMSK3 = 0;
}

int main(void)
{
    // Set DDRC to be all inputs
    DDRC = 0x00;

    // Enable pull up resisotr for PC7.
    PORTC = _BV(PC7);

    enableUSART();
    initializeBoostTimer();
    initializeJoysticks();
    rfConfigureRadio();

    MotorControlPayload payload = {1, 1450};

    while (true)
    {
        if (!boostEnabled && !boostCooldown && !(PINC & _BV(PC7)))
        {
            boostEnabled = true;
            boostCounter = 0;
        }

        if (boostEnabled && boostCounter < BOOST_TIME)
        {
            if (TIFR3 & _BV(OCF3A))
            {
                // Clear overflow flag
                TIFR3 |= _BV(OCF3A);
                boostCounter++;
            }

            if (boostCounter >= BOOST_TIME)
            {
                boostEnabled = false;
                boostCooldown = true;
            }
        }

        if (boostCooldown)
        {
            if (TIFR3 & _BV(OCF3A))
            {
                // Clear overflow flag
                TIFR3 |= _BV(OCF3A);
                boostCooldownCounter++;
            }

            if (boostCooldownCounter >= BOOST_COOLDOWN_TIME)
            {
                boostCooldownCounter = 0;
                boostCounter = 0;
                boostCooldown = false;
            }
        }

        int16_t speedValue = readSpeedJoystick();
        uint16_t steeringValue = readSteeringJoystick();
        int16_t speedBoostValue = speedValue + (speedValue * 1.5);

        payload.ocrMotor = boostEnabled ? speedBoostValue : speedValue;
        payload.ocrSteering = steeringValue;
        rfTransmitData(payload);
    }
}