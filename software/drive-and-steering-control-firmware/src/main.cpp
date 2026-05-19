#include "motor.h"
#include "rf.h"
#include "usart.h"
#include <RF24.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

MotorControlPayload payload = {0, 1900};

int main(void)
{
    // Clear watchdog reset flag and disable watchdog timer.
    MCUSR &= ~(1 << WDRF);
    wdt_disable();
    
    DDRD = 0xFF;
    PORTD |= _BV(PD5);

    sei();
    configureMotorPWM();
    configureSteeringPWM();
    enableUSART();
    configureRFRadio();
    
    while (true)
    {
        readAndPrintRFData(&payload);

        OCR1B = payload.ocrSteering;

        if (payload.ocrMotor >= -15 && payload.ocrMotor <= 15)
        {
            stopMotor();
        }
        else if (payload.ocrMotor < -15 && (!reverseMotor || motorStoped))
        {
            setMotorDirection(false);
        }
        else if (payload.ocrMotor > 15 && (reverseMotor || motorStoped))
        {
            setMotorDirection(true);
        }

        setMotorSpeed(payload.ocrMotor);
    }    
}