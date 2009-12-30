
#include <avr/io.h>
#include "delay.h"
#include "display.h"

// PB1 is RS
#define RS PB1
// PB2 is R/W
#define RW PB2
// PB3 is E
#define EN PB3
// PORTD is D0..7

// Busy flag
#define BF=0x80


void display_init(void){
   delay_ms(15);
   display_write_instr(0x30);
   delay_ms(5);
   display_write_instr(0x30);
   delay_us(100);
   display_write_instr(0x30);
   while ( display_read() & BF );
   display_write_instr(0x38);
   while ( display_read() & BF );
   display_write_instr(0x08);  
   while (  display_read() & BF );
   display_write_instr(0x01); 
   while (  display_read() & BF );
   display_write_instr(0x38);  
}
      

void display_puts_f(const char *flashdata){
}


void display_puts(const char *data){
}


void display_write_instr(char byte ){
   PORTB |= (1 << EN);
   PORTB &= ~(1 << RS);
   PORTB &= ~(1 << RW);
   PORTD = byte;
   PORTB &= ~(1 << EN);
}



void display_write_data(char byte ){
   while ( display_read() & BF );
   PORTB |= (1 << EN);
   PORTB |= (1 << RS);
   PORTB &= ~(1 << RW);
   PORTD = byte;
   PORTB &= ~(1 << EN);
}


char display_read(void){

char byte;
   PORTB |= (1 << EN);
   PORTB |= (1 << RW);
   PORTB &= ~(1 << RS);
   byte = PORTD;
   PORTB &= ~(1 << EN);
}


