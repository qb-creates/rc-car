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

extern "C" void __cxa_pure_virtual()
{
}

int main(void)
{
    DDRD = 0xFF;
    PORTD = 0x00;

    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << WGM02) | (1 << CS01); // Prescaler = 8
    OCR0A = 49;

    enableUSART();
    initADC();
    rfConfigureRadio();

    MotorControlPayload payload = {1, 1450};

    while (true)
    {
        // Max 100
        // int16_t speedValue = readADC(1);
        // speedValue = 1023 - speedValue;
        // speedValue = speedValue / 5.115;
        // speedValue = speedValue - 100;

        // Max is 40
        // int16_t speedValue = readADC(1);
        // speedValue = 1023 - speedValue;
        // speedValue = speedValue / 12.7875;
        // speedValue = speedValue - 40;

        // Max is 50
        int16_t speedValue = readADC(1);
        speedValue = 1023 - speedValue;
        speedValue = speedValue / 10.23;
        speedValue = speedValue - 50;

        // Max is 60
        // int16_t speedValue = readADC(1);
        // speedValue = 1023 - speedValue;
        // speedValue = speedValue / 8.525;
        // speedValue = speedValue - 60;

        // Max is set to 1900. Min is 1000. Middle is 1450
        uint16_t steeringValue = readADC(0);
        steeringValue = 1023 - steeringValue;

        // Divide by 1.13666667 to get 900 and add 1000. Makes the min to max 1000 to 1900
        steeringValue = (steeringValue / 1.1366666667) + 1000;

        payload.ocrMotor = speedValue;
        payload.ocrSteering = steeringValue;
        rfTransmitData(payload);
    }
}