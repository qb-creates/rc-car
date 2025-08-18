#include "usart.h"
#include <avr/io.h>

void enableUSART(void)
{
    UBRR0L = 25;
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

void usartTransmit(uint8_t data)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}

void usartTransmit(const char *data, int8_t length)
{
    for (int8_t i = 0; i < length; i++)
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = data[i];
    }
}

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