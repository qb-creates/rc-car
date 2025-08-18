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

MotorControlPayload payload = {0, 1450};

int main(void)
{
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

        if (payload.ocrMotor >= -5 && payload.ocrMotor <= 5)
        {
            stopMotor();
        }
        else if (payload.ocrMotor < -5 && (!reverseMotor || motorStoped))
        {
            setMotorDirection(false);
        }
        else if (payload.ocrMotor > 5 && (reverseMotor || motorStoped))
        {
            setMotorDirection(true);
        }

        setMotorSpeed(payload.ocrMotor);
    }    
}