
/**
 * @file rf.cpp
 * @brief Handles configuration and data transmission for the nRF24L01+ RF module.
 *
 * Provides functions to initialize the RF module and transmit data payloads.
 */

#include "rf.h"
#include "usart.h"

#define CE_PIN 0
#define CSN_PIN 1

RF24 radio(CE_PIN, CSN_PIN);
// uint8_t address[][6] = {"jag-1", "jag-2"};
uint8_t address[][6] = {"bmw-1", "bmw-2"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

/**
 * @brief Configures the nRF24L01+ radio and sets up communication parameters.
 *        Initializes timer, sets addresses, and prepares the radio for TX/RX.
 */
void rfConfigureRadio(void)
{
    // Initialize 1 millisecond timer on timer0
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << WGM02) | (1 << CS01);
    OCR0A = 49;

    if (!radio.begin())
    {
        usartTransmit("radio hardware is not responding!!\r\n", 38);
        while (1)
        {
        }
    }

    usartTransmit("RF radio ready\r\n", 16);
    usartTransmit("radioNumber = ", 14);
    uart_send_uint8_as_ascii(radioNumber);
    usartTransmit("\r\n", 2);

    radio.setPALevel(RF24_PA_LOW);
    radio.enableDynamicPayloads();
    radio.setAutoAck(false);

    // set the TX address of the RX node for use on the TX pipe (pipe 0)
    radio.stopListening(address[radioNumber]); // put radio in TX mode

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1
}

/**
 * @brief Transmits a MotorControlPayload struct over the RF link.
 * @param payload The data payload to transmit.
 */
void rfTransmitData(MotorControlPayload payload)
{
    bool report = radio.write(&payload, sizeof(payload));

    if (!report)
    {
        usartTransmit("Transmission failed or timed out\r\n", 35);
    }
}