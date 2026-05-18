#ifndef RF_H
#define RF_H

#include <RF24.h>

struct MotorControlPayload
{
    int16_t ocrMotor;
    uint16_t ocrSteering;
};

void rfConfigureRadio(void);
void rfTransmitData(MotorControlPayload data);

#endif