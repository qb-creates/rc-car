#ifndef USART_H
#define USART_H

#include "stdint.h"
void enableUSART(void);
void usartTransmit(uint8_t data);
void usartTransmit(const char *data, int8_t length);
void uart_send_uint8_as_ascii(uint16_t value);

#endif