
/**
 * @file joystick.cpp
 * @brief Provides functions to initialize and read analog joystick values for speed and steering.
 *
 * Contains ADC initialization and reading, as well as mapping joystick positions to speed and steering values.
 */

#include "joystick.h"

static constexpr uint16_t STEERING_CENTER = 1780;
static constexpr int16_t STEERING_RANGE = 380;
static constexpr uint16_t ADC_CENTER = 512;
static constexpr int16_t STEERING_MIN = (int16_t)STEERING_CENTER - STEERING_RANGE;
static constexpr int16_t STEERING_MAX = (int16_t)STEERING_CENTER + STEERING_RANGE;
static constexpr int16_t STEERING_CENTER_OFFSET_MIN = -120;
static constexpr int16_t STEERING_CENTER_OFFSET_MAX = 120;
static constexpr int16_t STEERING_CENTER_OFFSET = 50;

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
 * @return Steering value in the range 1400-2160 (center 1780).
 */
uint16_t readSteeringJoystick(void)
{
    uint16_t adcValue = readADC(0);
    adcValue = 1023 - adcValue;

    int16_t centerOffset = STEERING_CENTER_OFFSET;
    if (centerOffset < STEERING_CENTER_OFFSET_MIN)
    {
        centerOffset = STEERING_CENTER_OFFSET_MIN;
    }
    else if (centerOffset > STEERING_CENTER_OFFSET_MAX)
    {
        centerOffset = STEERING_CENTER_OFFSET_MAX;
    }

    int16_t centeredAdc = (int16_t)adcValue - (int16_t)ADC_CENTER;
    int16_t steeringCenter = (int16_t)STEERING_CENTER + centerOffset;
    if (steeringCenter <= STEERING_MIN)
    {
        steeringCenter = STEERING_MIN + 1;
    }
    else if (steeringCenter >= STEERING_MAX)
    {
        steeringCenter = STEERING_MAX - 1;
    }

    int16_t leftRange = steeringCenter - STEERING_MIN;
    int16_t rightRange = STEERING_MAX - steeringCenter;

    int16_t steeringValue;
    if (centeredAdc < 0)
    {
        steeringValue = steeringCenter + (int16_t)(((int32_t)centeredAdc * leftRange) / (int32_t)ADC_CENTER);
    }
    else
    {
        steeringValue = steeringCenter + (int16_t)(((int32_t)centeredAdc * rightRange) / (int32_t)(ADC_CENTER - 1));
    }

    if (steeringValue < STEERING_MIN)
    {
        steeringValue = STEERING_MIN;
    }
    else if (steeringValue > STEERING_MAX)
    {
        steeringValue = STEERING_MAX;
    }

    return (uint16_t)steeringValue;
}
