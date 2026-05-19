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

int main(void)
{
    // Set DDRC to be all inputs
    DDRC = 0x00;

    // Enable pull up resisotr for PC7.
    PORTC = _BV(PC7);

    enableUSART();
    initializeJoysticks();
    rfConfigureRadio();

    MotorControlPayload payload = {1, 1450};

    while (true)
    {
        int16_t speedValue = readSpeedJoystick();
        uint16_t steeringValue = readSteeringJoystick();

        payload.ocrMotor = speedValue;
        payload.ocrSteering = steeringValue;
        rfTransmitData(payload);
    }
}