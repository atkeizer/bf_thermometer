
#include <stdint.h>
#include <stdlib.h> 
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "LCD_driver.h"
#include "usart.h"
#include "onewire.h"
#include "crc8.h"
#include "delay.h"


#define MAX_1W 5

//void main(void) __attribute__((noreturn));
void init(void);
void OSCCAL_calibration(void);
void ADC_init(char);
int ADC_read(char);
void write_temp(char);

unsigned char scan_1w_rom();
void conv_ds18s20( unsigned char* );
char read_ds18s20( unsigned char*, char*, unsigned char* );
char search_ds18s20(void);

unsigned char romcodes[MAX_1W][8];

static FILE mystdout = FDEV_SETUP_STREAM(putc_usart, NULL, _FDEV_SETUP_WRITE);


int main(void)
{    
    unsigned char n, i, j, r, th;
    char tc;

    stdout = &mystdout;

    LCD_Init();                 // initialize the LCD

    sei();
    LCD_puts("INIT");
    init();           
    LCD_puts("INIT COMPLETE");
    printf("\n\nINIT COMPLETE\n\n");

    while (1)
    {
       for ( i=0 ; i < MAX_1W ; i++ ) 
          for ( j=0 ; j < 8 ; j++ ) 
             romcodes[i][j] = 0;
       printf("romcodes initialized\n");
       for ( i=0 ; i < 64 ; i++ )  printf("*");
       printf("\n");
       if ( (n=scan_1w_rom()) ) {
          printf("%i devices found on 1-wire bus\n", n);
       }
       else {
          printf("No devices found on 1-wire bus\n");
       }
       for ( i=0 ; i < n ; i++ ) {
          for ( r=0 ; r < 8 ; r++ ) 
             printf("%02X",romcodes[i][r]);
          read_ds18s20( romcodes[i], &tc, &th );
          printf("\nTemp = %i,%i C\n", tc, th);
       }
        printf("\n");
       _delay_ms(3000);
    }    
}


unsigned char scan_1w_rom() {

   unsigned char byte, byte_bit, bit, branch, i, o, n=0, branches[MAX_1W];

   do {
       ow_reset();
       ow_byte_wr( OW_SEARCH_ROM );
       o=1;  // offset
       bit = 0;
       branch=0;
       do {
          byte =  7 - bit/8;
          byte_bit = (bit & 7);
          if ( ow_bit_io( 1 ) ) {     // if first bit is 1
             if ( ow_bit_io( 1 ) )    // and second bit is one, no devices participating in search
                return 0;
             else {                   // or second bit is 0, only devices with 1 in this bit position
                romcodes[n][byte] |= ( 1 << byte_bit );
                ow_bit_io( 1 );
                printf("1");
             }
          }
          else {                      // first bit is 0
             if ( ow_bit_io( 1 ) ) {  // and second bit is 1, only devices with 0 in this bit position
                romcodes[n][byte] &= ~( 1 << byte_bit );
                ow_bit_io( 0 );
                printf("0");
             }   
             else                   // both 1 and 0 in this bit position 
             if ( branches[n] != bit ) { // branch at this position already explored
                if (  romcodes[n][byte] & ( 1 << byte_bit ) ) { // if bit was already set then continue
                                        // with the 1's branch, 0 branch was done in a previous iteration
                   printf("continueing with 1 branch, decreasing branch counter\n");
                   branches[n]=bit;
                   ow_bit_io( 1 ); 
                   printf("1");
                   branch--;
                }
                else {
                   printf("cloning romcode continuing 0 branch, increasing branch counter\n");
                   branch++;
                   for ( i=byte ; i > 0 ; i-- )   // clone the rom id so far obtained
                      romcodes[n+branch][i] = romcodes[n][i];
                   romcodes[n+branch][byte] |= ( 1 << byte_bit );
                   romcodes[n][byte] &= ~( 1 << byte_bit ); // continue with 0 branch
                   ow_bit_io( 0 );
                   printf("0");
                }
             }
          }      
       } while ( bit++ < 63 ) ;
       printf("\n");
    } while ( ( ++n < MAX_1W ) &&  branch ) ;
    return n;
}




void init(void)
{
    OSCCAL_calibration();

    CLKPR = (1<<CLKPCE);        // set Clock Prescaler Change Enable

    CLKPR = 0;  // 8 MHz

    init_usart_19k2();

    PORTF |= (1<<PF3); // mt sbi(PORTF, PORTF3);     // Enable the VCP (VC-peripheral)
    DDRF |= (1<<DDF3); // sbi(DDRF, PORTF3);        
}





void OSCCAL_calibration(void)
{
    unsigned char calibrate = 0;
    int temp;
    unsigned char tempL;

    CLKPR = (1<<CLKPCE);        // set Clock Prescaler Change Enable
    
    // set prescaler = 8, Inter RC 8Mhz / 8 = 1Mhz
    CLKPR = (1<<CLKPS1) | (1<<CLKPS0);
	
    
    TIMSK2 = 0;             //disable OCIE2A and TOIE2

    ASSR = (1<<AS2);        //select asynchronous operation of timer2 (32,768kHz)
    
    OCR2A = 200;            // set timer2 compare value 

    TIMSK0 = 0;             // delete any interrupt sources
        
    TCCR1B = (1<<CS10);     // start timer1 with no prescaling
    TCCR2A = (1<<CS20);     // start timer2 with no prescaling

    while((ASSR & 0x01) | (ASSR & 0x04));       //wait for TCN2UB and TCR2UB to be cleared

    _delay_ms(1000);    // wait for external crystal to stabilise
    
    while(!calibrate)
    {
        cli(); // mt __disable_interrupt();  // disable global interrupt
        
        TIFR1 = 0xFF;   // delete TIFR1 flags
        TIFR2 = 0xFF;   // delete TIFR2 flags
        
        TCNT1H = 0;     // clear timer1 counter
        TCNT1L = 0;
        TCNT2 = 0;      // clear timer2 counter
           
        // shc/mt while ( !(TIFR2 && (1<<OCF2A)) );   // wait for timer2 compareflag    
        while ( !(TIFR2 & (1<<OCF2A)) );   // wait for timer2 compareflag

        TCCR1B = 0; // stop timer1

        sei(); // __enable_interrupt();  // enable global interrupt
    
        // shc/mt if ( (TIFR1 && (1<<TOV1)) )
        if ( (TIFR1 & (1<<TOV1)) )
        {
            temp = 0xFFFF;      // if timer1 overflows, set the temp to 0xFFFF
        }
        else
        {   // read out the timer1 counter value
            tempL = TCNT1L;
            temp = TCNT1H;
            temp = (temp << 8);
            temp += tempL;
        }
    
        if (temp > 6250)
        {
            OSCCAL--;   // the internRC oscillator runs to fast, decrease the OSCCAL
        }
        else if (temp < 6120)
        {
            OSCCAL++;   // the internRC oscillator runs to slow, increase the OSCCAL
        }
        else
            calibrate = 1;   // the interRC is correct

        TCCR1B = (1<<CS10); // start timer1
    }
}



void ADC_init(char channel)
{
    PRR &= ~(1<<PRADC);
    ADMUX = channel;    // external AREF and ADCx
    ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);    // set ADC prescaler to , 1MHz / 8 = 125kHz    
}



int ADC_read(char channel)
{

    ADC_init(channel);

    ADCSRA |= (1<<ADSC);        // do single conversion
    while(!(ADCSRA & (1<<ADIF)));   // wait for conversion done, ADIF flag active   
    ADCSRA |= (1<<ADIF);            //clear ADIF flag by writing a 1 to it
    return ADCL + (ADCH << 8);
}





//char search_ds18s20(void)
//{
//        unsigned char last, n=0 ;
//
//        last = OW_SEARCH_FIRST;
//printf("Searching for DS18S20\n");
//        
//        do {
//		  
//printf("last before ow_rom_search %X\n", last);
//            last = ow_rom_search( last, romcodes[n] );
//printf("last after ow_rom_search  %X\n", last);
//        //if (romcodes[n][0] == 0x10 )  // ds18s20 family ID only
//        n++;
//        } while ((last != OW_LAST_DEVICE) &&
//                 (n < MAX_DS18S20)  &&
//                 (last != OW_PRESENCE_ERR) &&
//                 (last != OW_DATA_ERR)) ;
//printf("after while loop found %i, last  =  %X\n", n, last);
//        return n;
//}


void conv_ds18s20( unsigned char *id )
{
   ow_command( 0x44, id );
}


char read_ds18s20( unsigned char *id, char *deg, unsigned char *hundreds )
{
   unsigned char i, scratch[9];

   ow_command(0xBE, id);  // Read scratchpad
   for ( i=0 ; i<9 ; i++ ) { 
       scratch[i]=ow_byte_rd();
   }
   if ( crc8( scratch, 9 ) )
      return 1;
   else {
      // byte 0 is MSB byte 1 is LSB
      // byte 6 is COUNT REMAIN and byte 7 is COUNT PER C
      *hundreds = (scratch[1] & 1) * 25 + (scratch[7] - scratch[6])/scratch[7]; 
      *deg = (scratch[1]/2) | (scratch[0] & 128); // 2 complements signed char
      return 0;                                // take sign of MSB
   }
}

