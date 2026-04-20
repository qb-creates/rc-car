
/**
 * @file usart.cpp
 * @brief Provides USART (Universal Synchronous/Asynchronous Receiver/Transmitter) initialization and data transmission functions for ATmega1284.
 *
 * Functions for enabling USART, transmitting bytes and strings, and sending unsigned integers as ASCII.
 */

#include "usart.h"
#include <avr/io.h>


/**
 * @brief Initializes the USART hardware with 8N1 format and 38400 baud rate.
 */
void enableUSART(void)
{
    UBRR0L = 25;
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}


/**
 * @brief Transmits a single byte over USART.
 * @param data Byte to transmit.
 */
void usartTransmit(uint8_t data)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}


/**
 * @brief Transmits a string of bytes over USART.
 * @param data Pointer to the data buffer.
 * @param length Number of bytes to transmit.
 */
void usartTransmit(const char *data, int8_t length)
{
    for (int8_t i = 0; i < length; i++)
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = data[i];
    }
}


/**
 * @brief Sends a 16-bit unsigned integer as ASCII characters over USART.
 * @param value The value to send (0-65535).
 */
void uart_send_uint8_as_ascii(uint16_t value)
{
    bool started = false;

    if (value >= 10000)
    {
        usartTransmit('0' + (value / 10000));
        value %= 10000;
        started = true;
    }
    if (started || value >= 1000)
    {
        usartTransmit('0' + (value / 1000));
        value %= 1000;
        started = true;
    }
    if (started || value >= 100)
    {
        usartTransmit('0' + (value / 100));
        value %= 100;
        started = true;
    }
    if (started || value >= 10)
    {
        usartTransmit('0' + (value / 10));
        value %= 10;
        started = true;
    }

    usartTransmit('0' + value);
}