#ifndef RF_H
#define RF_H

#include <RF24.h>

struct MotorControlPayload
{
    int16_t ocrMotor; // only using 6 characters for TX & ACK payloads
    uint16_t ocrSteering;
};

void configureRFRadio(void);
void readAndPrintRFData(MotorControlPayload *payload);
void printRF24Status(void);

#endif