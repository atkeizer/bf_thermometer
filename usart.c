
#include <avr/io.h>
#include <stdio.h>

#include "usart.h"

void init_usart_19k2()
{
    // Set baud rate
    UBRRH = 0;
    UBRRL = 51;

    // Enable 2x speed
    UCSRA = (1<<U2X);

    UCSRB = (1<<RXEN)|(1<<TXEN)|(0<<RXCIE)|(0<<UDRIE);

    // Async. mode, 8N1
    UCSRC = (0<<UMSEL)|(0<<UPM0)|(0<<USBS)|(3<<UCSZ0)|(0<<UCPOL);
}



void putc_usart(char c, FILE *stream)
{
    if (c == '\n')
        putc_usart('\r', stream);
    while (!(UCSRA & (1<<UDRE)));
    UDR = c;
}





char getc_usart(void)
{
    while (!(UCSRA & (1<<RXC)));
    return UDR;
}
