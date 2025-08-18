#include "joystick.h"

void initADC(void)
{
    // Set ADC reference to AVCC (5V) and select ADC0 (PA0)
    ADMUX = (1 << REFS0);
    
    // Enable ADC and set prescaler to 128 (for 16MHz clock: 16MHz/128 = 125kHz)
    // ADC frequency should be between 50kHz-200kHz for maximum resolution
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // Perform a dummy conversion to initialize the ADC
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
}

uint16_t readADC(uint8_t channel)
{
    // Select ADC channel (0-7 for ATmega1284)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return ADC result (10-bit value: 0-1023)
    return ADC;
}

int16_t readJoystickAxis(void)
{
    // Read ADC value from PA0 (channel 0)
    uint16_t adcValue = readADC(0);
    
    // ADC gives us 0-1023 for 0V-5V reference
    // Our input range is 0V-3.3V:
    // 0V corresponds to ADC value: 0
    // 1.65V (center) corresponds to ADC value: (1.65/5.0) * 1023 = ~337
    // 3.3V corresponds to ADC value: (3.3/5.0) * 1023 = ~675
    
    // Map ADC value to our desired range:
    // ADC 0 (0V) -> +198
    // ADC 337 (1.65V) -> 0 (center)
    // ADC 675 (3.3V) -> -198
    
    int16_t result;
    
    if (adcValue <= 675)
    {
        // Linear interpolation with correct mapping:
        // ADC 0 (0V) -> +198
        // ADC 337 (1.65V) -> 0 
        // ADC 675 (3.3V) -> -198
        
        // Two-segment linear mapping:
        if (adcValue <= 337)
        {
            // Map 0-337 ADC to +198 to 0
            // result = 198 - (adcValue * 198 / 337)
            result = 198 - ((int32_t)adcValue * 198) / 337;
        }
        else
        {
            // Map 337-675 ADC to 0 to -198
            // result = 0 - ((adcValue - 337) * 198 / (675 - 337))
            result = -((int32_t)(adcValue - 337) * 198) / 338;
        }
        
        // Clamp to valid range
        if (result > 198) result = 198;
        if (result < -198) result = -198;
    }
    else
    {
        // Values above 3.3V (ADC > 675) are clamped to -198
        result = -198;
    }
    
    return result;
}