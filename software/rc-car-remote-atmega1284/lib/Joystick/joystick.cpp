#include "joystick.h"

void initializeJoysticks(void)
{
    // Set ADC reference to AVCC (5V) and select ADC0 (PA0)
    ADMUX = (1 << REFS0);

    // Enable ADC and set prescaler to 128 (for 16MHz clock: 16MHz/128 = 125kHz)
    // ADC frequency should be between 50kHz-200kHz for maximum resolution
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Perform a dummy conversion to initialize the ADC
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;
}

uint16_t readADC(uint8_t channel)
{
    // Select ADC channel (0-7 for ATmega1284)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC))
        ;

    // Return ADC result (10-bit value: 0-1023)
    return ADC;
}

int16_t readSpeedJoystick(void)
{
    // Max is 60
    // int16_t speedValue = readADC(1);
    // speedValue = 1023 - speedValue;
    // speedValue = speedValue / 8.525;
    // speedValue = speedValue - 60;

    int16_t speedValue = readADC(1);
    speedValue = 1023 - speedValue;
    speedValue = speedValue / 10.23;
    speedValue = speedValue - 50;
    return speedValue;
}

uint16_t readSteeringJoystick(void)
{
    // Max is set to 1900. Min is 1000. Middle is 1450
    uint16_t steeringValue = readADC(0);
    steeringValue = 1023 - steeringValue;

    // Divide by 1.27875 to get 800 and add 1000. Makes the min to max 1000 to 1800
    steeringValue = (steeringValue / 1.27875) + 1000;
    steeringValue = steeringValue <= 1100 ? 1100 : steeringValue;

    return steeringValue;
}
