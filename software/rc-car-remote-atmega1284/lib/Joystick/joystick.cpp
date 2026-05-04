
/**
 * @file joystick.cpp
 * @brief Provides functions to initialize and read analog joystick values for speed and steering.
 *
 * Contains ADC initialization and reading, as well as mapping joystick positions to speed and steering values.
 */

#include "joystick.h"

/**
 * @brief Initializes the ADC hardware for joystick input.
 */
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

/**
 * @brief Reads a value from the specified ADC channel (0-7).
 * @param channel The ADC channel to read.
 * @return 10-bit ADC value (0-1023).
 */
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

/**
 * @brief Reads and maps the speed joystick value to a signed speed output.
 *        Inverts and scales the ADC value to a range centered around zero.
 * @return Signed speed value.
 */
int16_t readSpeedJoystick(void)
{
    int16_t speedValue = readADC(1);
    speedValue = 1023 - speedValue;
    speedValue = speedValue / 2.6921052;
    speedValue = speedValue - 190;
    return speedValue;
}

/**
 * @brief Reads and maps the steering joystick value to a PWM-compatible range.
 *        Inverts and scales the ADC value to fit servo or motor controller input.
 * @return Steering value in the range 1400-2000 (center ~1675).
 */
uint16_t readSteeringJoystick(void)
{
    // Piecewise mapping: ADC 0 -> 1400, ADC ~511/512 -> 1675, ADC 1023 -> 2000
    uint16_t adcValue = readADC(0);
    adcValue = 1023 - adcValue; // Invert if needed for joystick direction

    // Map 0-511 to 1400-1675.
    if (adcValue <= 511) {
        return ((uint32_t)adcValue * 275) / 511 + 1400;
    }

    // Map 512-1023 to 1675-2000.
    return ((uint32_t)(adcValue - 512) * 325) / 511 + 1675;
}
