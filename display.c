
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
#define BF 0x80


void display_init(void){
   DDRB |= (1<<RS) | (1<<RW) | (1<<EN); //are outputs
   DDRD = 0; // initially inputs
   delay_ms(15);
   display_write_instr(0x30);
   delay_ms(5);
   display_write_instr(0x30);
   delay_us(100);
   display_write_instr(0x30);
   while ( display_read() & BF );
   display_write_instr(0x38);        // 8 bits two lines 5x7 fonts
   while ( display_read() & BF );
   display_write_instr(0x08);        // display off
   while (  display_read() & BF );
   display_write_instr(0x01);        // display clear
   while (  display_read() & BF );
   display_write_instr(0x06);        // entry with icrement and no display shift 
}
      

void display_puts_f(const char *flashdata){
}


void display_puts(const char *data){
}


void display_write_instr(char byte ){
   DDRD = 0xff; // all are outputs
   PORTB |= (1 << EN);
   PORTB &= ~(1 << RS);
   PORTB &= ~(1 << RW);
   PORTD = byte;
   PORTB &= ~(1 << EN);
   DDRD = 0;  // inputs again
}



void display_write_data(char byte ){
   while ( display_read() & BF );
   DDRD = 0xff; // all are outputs
   PORTB |= (1 << EN);
   PORTB |= (1 << RS);
   PORTB &= ~(1 << RW);
   PORTD = byte;
   PORTB &= ~(1 << EN);
   DDRD = 0;  // inputs again
}


char display_read(void){

char byte;
   PORTB |= (1 << EN);
   PORTB |= (1 << RW);
   PORTB &= ~(1 << RS);
   byte = PIND;
   PORTB &= ~(1 << EN);
   return byte;
}


