
/**
 * @file joystick.cpp
 * @brief Provides functions to initialize and read analog joystick values for speed and steering.
 *
 * Contains ADC initialization and reading, as well as mapping joystick positions to speed and steering values.
 */

#include "joystick.h"

// Steering voltage calibration points in millivolts.
// These are measured joystick voltages for min/center/max steering positions.
static constexpr uint16_t STEERING_VOLTAGE_MIN_MV = 3285;
static constexpr uint16_t STEERING_VOLTAGE_CENTER_MV = 1700;
static constexpr uint16_t STEERING_VOLTAGE_MAX_MV = 0;
static constexpr uint16_t ADC_REFERENCE_MV = 3300;

static constexpr uint16_t STEERING_CENTER = 1900;
static constexpr int16_t STEERING_RANGE = 380;

// Convert measured voltage to the inverted ADC domain used by steering logic
// (adc = 1023 - readADC(1)).
static constexpr uint16_t voltageToInvertedADC(uint16_t voltageMv)
{
    uint32_t raw = ((uint32_t)voltageMv * 1023u + (ADC_REFERENCE_MV / 2u)) / ADC_REFERENCE_MV;
    if (raw > 1023u)
    {
        raw = 1023u;
    }

    return (uint16_t)(1023u - raw);
}

static constexpr uint16_t STEERING_ADC_MIN = voltageToInvertedADC(STEERING_VOLTAGE_MIN_MV);
static constexpr uint16_t STEERING_ADC_CENTER = voltageToInvertedADC(STEERING_VOLTAGE_CENTER_MV);
static constexpr uint16_t STEERING_ADC_MAX = voltageToInvertedADC(STEERING_VOLTAGE_MAX_MV);
static constexpr int16_t STEERING_MIN = (int16_t)STEERING_CENTER - STEERING_RANGE;
static constexpr int16_t STEERING_MAX = (int16_t)STEERING_CENTER + STEERING_RANGE;
static constexpr int16_t STEERING_DEADZONE = 50;

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
    int16_t speedValue = readADC(0);
    speedValue = 1023 - speedValue;
    speedValue = speedValue / 2.6921052;
    speedValue = speedValue - 190;
    return speedValue;
}

/**
 * @brief Reads and maps the steering joystick value to a PWM-compatible range.
 *        Inverts and scales the ADC value to fit servo or motor controller input.
 *        Snaps to center when the mapped value is within deadzone.
 * @return Steering value in the range 1520-2280 (center 1900).
 */
uint16_t readSteeringJoystick(void)
{
    uint16_t adcValue = readADC(1);
    adcValue = 1023 - adcValue;

    int16_t steeringValue;
    if (adcValue < STEERING_ADC_CENTER)
    {
        int32_t spanToMin = (int32_t)STEERING_ADC_CENTER - (int32_t)STEERING_ADC_MIN;
        int32_t deltaFromCenter = (int32_t)adcValue - (int32_t)STEERING_ADC_CENTER;
        steeringValue = (int16_t)STEERING_CENTER + (int16_t)((deltaFromCenter * STEERING_RANGE) / spanToMin);
    }
    else
    {
        int32_t spanToMax = (int32_t)STEERING_ADC_MAX - (int32_t)STEERING_ADC_CENTER;
        int32_t deltaFromCenter = (int32_t)adcValue - (int32_t)STEERING_ADC_CENTER;
        steeringValue = (int16_t)STEERING_CENTER + (int16_t)((deltaFromCenter * STEERING_RANGE) / spanToMax);
    }

    if (steeringValue < STEERING_MIN)
    {
        steeringValue = STEERING_MIN;
    }
    else if (steeringValue > STEERING_MAX)
    {
        steeringValue = STEERING_MAX;
    }

    int16_t deltaFromCenter = steeringValue - (int16_t)STEERING_CENTER;
    if (deltaFromCenter < 0)
    {
        deltaFromCenter = -deltaFromCenter;
    }

    if (deltaFromCenter <= STEERING_DEADZONE)
    {
        steeringValue = (int16_t)STEERING_CENTER;
    }

    return (uint16_t)steeringValue;
}
