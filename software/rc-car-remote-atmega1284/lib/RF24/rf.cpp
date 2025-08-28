#include "rf.h"
#include "usart.h"

#define CE_PIN 0
#define CSN_PIN 1

RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

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

void rfTransmitData(MotorControlPayload payload)
{
    bool report = radio.write(&payload, sizeof(payload));

    if (!report)
    {
        // usartTransmit("Transmission failed or timed out\r\n", 35);
    }
}

void printRF24Status(void)
{
    //     usartTransmit("RF24 Status:\r\n", 13);

    //     // Test basic SPI communication first
    //     usartTransmit("Testing basic SPI...\r\n", 22);

    //     // // Simple SPI test - try to read CONFIG register (0x00)
    //     // PORTB &= ~(1 << PB1); // CSN low (PB1)
    //     // SPDR = 0x00;          // Send read command for CONFIG register
    //     // while (!(SPSR & (1 << SPIF)))
    //     //     ;
    //     // SPDR = 0xFF; // Send dummy byte
    //     // while (!(SPSR & (1 << SPIF)))
    //     //     ;
    //     // uint8_t config_reg = SPDR;
    //     // PORTB |= (1 << PB1); // CSN high (PB1)

    //     // usartTransmit("CONFIG register: ", 17);
    //     // uart_send_uint8_as_ascii(config_reg);
    //     usartTransmit("\r\n", 2);

    //     if (radio.isChipConnected())
    //     {
    //         usartTransmit("- Chip: Connected\r\n", 18);
    //     }
    //     else
    //     {
    //         usartTransmit("- Chip: NOT Connected!\r\n", 24);
    //     }

    //     usartTransmit("- Channel: ", 11);
    //     uart_send_uint8_as_ascii(radio.getChannel());
    //     usartTransmit("\r\n", 2);

    //     usartTransmit("- Data Rate: ", 13);
    //     if (radio.getDataRate() == RF24_1MBPS)
    //     {
    //         usartTransmit("1Mbps\r\n", 7);
    //     }
    //     else if (radio.getDataRate() == RF24_2MBPS)
    //     {
    //         usartTransmit("2Mbps\r\n", 7);
    //     }
    //     else
    //     {
    //         usartTransmit("250kbps\r\n", 9);
    //     }

    //     usartTransmit("- Power Level: ", 15);
    //     if (radio.getPALevel() == RF24_PA_LOW)
    //     {
    //         usartTransmit("LOW\r\n", 5);
    //     }
    //     else if (radio.getPALevel() == RF24_PA_HIGH)
    //     {
    //         usartTransmit("HIGH\r\n", 6);
    //     }
    //     else
    //     {
    //         usartTransmit("MAX\r\n", 5);
    //     }
}