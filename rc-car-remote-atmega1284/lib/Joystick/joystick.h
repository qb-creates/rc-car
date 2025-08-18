#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <avr/io.h>
#include <stdint.h>

// Function declarations
void initADC(void);
uint16_t readADC(uint8_t channel);
int16_t readJoystickAxis(void);

#endif