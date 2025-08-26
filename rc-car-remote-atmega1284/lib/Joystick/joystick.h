#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <avr/io.h>
#include <stdint.h>

// Function declarations
void initializeJoysticks(void);
int16_t readSpeedJoystick(void);
uint16_t readSteeringJoystick(void);

#endif