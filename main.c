// Use PB4 to DIR connection
// Use PP1 for EN connection (duty cycle)
// Ground together
// This is for reference only, Peter Han

#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <stdio.h>

// Function Prototypes
void init_ATD(void);
void initLCD(void);
void initPorts(void);
void cmdwrt(unsigned char);
void datawrt(char);
void delay1u(void);
void delay100u(void);
void delay2m(void);
void delays(int);
void LCDputs(char *);
void position(int, int); 
void init_PWM(void);
void init_SW4_SW2(void);                 
void delay10mS(void);

#define EN  0x80
#define DIR 0x20

//globals - 4-bit initialization sequence for LCD  -  data: PC7,6,5,4  E: PC2 , RS: PC0
const unsigned char cInit_commands[20] = {0x30,0x30,0x30,0x20,0x20,0x90,0x10,0x50,0x70,0x80,0x50,0xE0,
                                          0x60,0xA0,0x00,0xC0,0x00,0x10,0x00,0x60};

unsigned int value;                       // ADC data
unsigned char EN_key;                     // EN (SW4) status
unsigned char DIR_key;                    // DIR (SW2) status
unsigned char swdata;                     // swdata (switch input from PAD)
unsigned char swdatadeb;                  // switch debounced after 10 mS
unsigned char swdatapre;                  // switch input previously read
unsigned char swdataLO;                   // falling edge detection
unsigned char DutyCycle;                  // Duty ccyle

const unsigned char cE  = 0x04;         // PC2
const unsigned char cRS = 0x01;         // PC0

void main(void) 
{
  unsigned char cValue;
  char ADCdisplay[5];
  unsigned int old_value;
  
  DDRT = 0xF0;                          // PORT T7-T4 output (LEDs)
  init_ATD();                           // Call ATD initialization function
  initPorts( );                         // port initializations
  initLCD( );                           // LCD inititialization
  init_PWM();
  
  DDRT = 0xF0;                          // Read key input, control LEDs
  DDRB |= 0x10;                         // Let PB4 bacomes OUTPUT
  
  PTT = 0xF0;                       // initialize with LEDs off
  init_SW4_SW2();                   // Configure the sw2 and sw4 as DIR and POWER
  EN_key = 0;
  DIR_key = 0;
  
  TSCR1 = 0x90;                     // Enable Timer and Fast Flag Clear Automatically
  TSCR2 = 0x00;                     // no prescaler
  TIOS |= 0x01;                     // ch0 is used for output compare
  TIE = 0x00;                       // No OC or IC INT 
  TFLG1 |= 0x01;                    // clear the flag so timer will start over

	position(1,0);
	LCDputs("ADC read:");             // Leave constant LCD display data outside the loop

  position(2,0);
	LCDputs("Duty Cycle:");           // Leave constant LCD display data outside the loop
	
  position(2,14);                   // Leave constant LCD display data outside the loop
  datawrt('%');

  position(3,0);
  LCDputs("POWER:   DIR:");         // Leave constant LCD display data outside the loop
		  
	
  for(;;) 
  {
		while(!(ATDSTAT0_SCF))          // wait for conversion to complete
		   ;;
		value = ATDDR0;                 // Read ATD0 data and assign result to variable value.  (16 bits)
		ATDSTAT0_SCF = 1;               // clear conv seq complete flag
		ATDCTL5 = 0x00;                 // start a new conversion
		//PTT = (~value) & 0xF0;          // logic Low turns on LED
	
		if (value != old_value)         // Only update LCD when ADC reading is different
		{
		  PWMPER01 = 800;                  // obtain a period width 1.024mS
      PWMDTY01 = (value * 800L)/1023;   // make it the value from 0 ~ 800
     
  		(void)sprintf(ADCdisplay,"%d",value);
	  	//cmdwrt(0x00); cmdwrt(0x10); // clear LCD
	  	position(1,9);
	  	LCDputs("    ");
	  	position(1,9);
	  	LCDputs(ADCdisplay);
		  
		  DutyCycle = (value * 100L) /1023;
		  (void)sprintf(ADCdisplay,"%d",DutyCycle);
  		position(2,11);
  		LCDputs("   ");
  		position(2,11);
      LCDputs(ADCdisplay);
		}
		
		old_value = value;
				
    //Keypad detection
		swdata = (PT1AD & 0xF0);          // read PAD inputs
		delay10mS();
		
		if (swdata == (PT1AD & 0x0F0))
		  swdatadeb = (PT1AD & 0xF0);     // debounced key read
		  
		swdata = (swdatadeb ^ swdatapre) & 0xF0;    // Find which key changes
		swdataLO = swdata & swdatapre;
		
		swdatapre = swdatadeb;          // update the previous reading		
		
		if (swdataLO & EN) 
		{
		  EN_key ^= 0x01;
		
		  if (EN_key == 0x01) 
      {
        PWME |= 0x02;                 // Turn PWME channel 1 on
        PTT &= ~0x80;                 // Turn SW4 LED on
        position(3,6);
        LCDputs("   ");
        position(3,6);
        LCDputs("ON");
	
      }
      else 
      {
        PWME &= ~0x02;                // Off PWM channe 1
        PTT |= 0x80;                  // Off SW4 LED
        position(3,6);
        LCDputs("OFF");
	        
      }
		
		}
		
		if (swdataLO & DIR) 
		{
		  DIR_key ^= 0x01;
		  
		  if (DIR_key == 0x01)
		  {
		    PORTB |= 0x10;
		    PTT &= ~0x20;                 // Turn SW2 LED on
		    position(3,13);
        LCDputs("CCW");		    
		  }
		  else 
		  {
		    PORTB &= ~0x10;
		    PTT |= 0x20;                   // Off SW2 LED
		    position(3,13);
        LCDputs("   ");
        position(3,13);
        LCDputs("CW");
		  }
		}
		
		swdata = 0x00;
  }                                 // end of infinite for loop
}                                   // end of main

// initialize ATD converter
void init_ATD(void)
{
	ATDCTL1_SRES = 1;              // 10-bit results
	ATDCTL3_DJM = 1;               // right justified data
	ATDCTL3_S8C = 0;               // one conversion per sequence
	ATDCTL3_S4C = 0;               // one conversion per sequence
	ATDCTL3_S2C = 0;               // one conversion per sequence
	ATDCTL3_S1C = 1;               // one conversion per sequence
	ATDCTL4_PRS = 11;              // bus clk div 24, = ATD clock = 6.25MHz/24 = 260.416KHz
	ATDCTL4_SMP = 4;               // 12 ATD clock cycle to sample (at the center)
	ATDSTAT0_SCF = 1;	          // clear flag
	ATDCTL5 = 0;                        // select channel 0-starts first conversion
}


void position(int iRow_value, int iCol_value) 
{
     int iPos_h, iPos_l, iValue;
     
     if(iRow_value == 1) 
        iValue = (0x80+0x00);
     
     if(iRow_value == 2) 
        iValue = (0x80+0x10);
     
     if(iRow_value == 3) 
        iValue = (0x80+0x20);
     
     iPos_h = ((iValue + iCol_value) & 0xF0);
     iPos_l = ((iValue + iCol_value) & 0x0F) << 4;
     
     cmdwrt(iPos_h);
     cmdwrt(iPos_l);
}
        
//Sends a string of characters to the LCD;...  
void LCDputs(char *sptr)
{  
     while(*sptr)
     {                                //...the string must end in a 0x00 (null character)
        datawrt(*sptr);               // sptr is a pointer to the characters in the string
        ++sptr;
     }
}  
                                          
// sends initialization commands one-by-one
void initLCD( )
{
  unsigned char i;
  
  for (i=0;i<20;i++)
  {  
    cmdwrt(cInit_commands[i]);
  }  
}

 
void initPorts( )
{ 
  unsigned char cValue;
  
  DDRB   = 0x80;                        //LCD CSB active low
  DDRC   = 0xFF;                        // PC7-PC4 - 4-bit LCD data bus, PC2 - E, PC1 - R/W~, PC0 - RS: all outputs
  cValue = PORTB;
  PORTB  = (cValue & ~0x80);            // LCD CSB (PORT B7) enabled with a logic low  
}  
  

// sends a control word to LCD bus
void cmdwrt(unsigned char cCtrlword)
{  
  PORTC = cCtrlword;   // output command onto LCD data pins
  PORTC = cCtrlword + cE;   // generate enable pulse to latch it (xxxx x100)
  
  delay1u( );    // hold it for 1us
  
  PORTC = cCtrlword;    // end enable pulse  (xxxx x000)
  
  delay2m();    // allow 2ms to latch command inside LCD
  
  PORTC = 0x00;
  
  delay2m();    // allow 2ms to latch command inside LCD
}

// sends the character passed in by caller to LCD 
void datawrt(char cAscii)
{                   
  char cAscii_high, cAscii_low;
  
  cAscii_high = (cAscii & 0xF0);
  cAscii_low  = (cAscii & 0x0F) << 4; // Shift left by 4 bits 
  PORTC = cAscii_high;                // output ASCII character upper nibble onto LCD data pins
  PORTC = cAscii_high + cRS + cE;     // generate enable pulse to latch it  (0xxx x101)
  
  delay1u( );                         // hold it for 1us
  
  PORTC = cAscii_high + cRS;          // end enable pulse   (0xxx x001)
  
  delay1u( );                         // hold it for 1us
  
  PORTC = cAscii_low;                 // output ASCII character lower nibble onto LCD data pins
  PORTC = cAscii_low + cRS + cE;      // generate enable pulse to latch it  (0xxx x101)
  
  delay1u( );                         // hold it for 1us
  
  PORTC = cAscii_low + cRS;           // end enable pulse   (0xxx x001)
  
  delay100u( );                       // allow 100us to latch data inside LCD
}


void delay1u( )
{
  unsigned int i;
  
  for(i=0;i<=0x0f;i++)
  { /* adjust condition field for delay time */
    asm("nop");
  }
}

void delay100u( )
{
  unsigned int i,j;
  
  for(i=0;i<=0x02;i++)
  {  /* adjust condition field for delay time */
    for(j=0;j<=0xff;j++)
    {
       asm("nop");
    }
  }
}

void delay2m( )
{
   unsigned int i,j; 
   
   for (i=0;i<=0x20;i++)
   { /* adjust condition field for delay time */
     for (j=0;j<=0xff;j++)
     {
      asm("nop");
    }     
   }
}   

 
void delays(int k )
{
   unsigned int i,j; 
   
   for (i=0;i<=k;i++)
   { /* adjust condition field for delay time */
     for (j=0;j<=0xff;j++)
     {
      asm("nop");
    }     
   }
}     



void init_PWM()
{
  PWMCAE = 0x00;                // left aligned
  PWMCLK = 0x00;                // Ch. 0 - Ch. 1 source is clock A, ch.2 & ch 3 use clock B 
  PWMCLKAB = 0x00;              // Use clock A and B respectively
  PWMPOL = 0x0A;                // initial HIGH output on ch. 1 and 3 (Use odd # registers)
  PWMPRCLK = 0x33;              // Clk A pre-scale = 8 (PWM clock = 6.25MHz/8 = 781.25 KHz =>period = 1.28uS), so is clock B
  PWMCTL = 0x30;                // CON01 = '1' and Con23 = '1': 16-bit PWM counter, period and duty regs.
  //PWME |= 0x0A;                 // turn-on PWM ch. 1 and ch. 3 (16 bits)
}

// initialize push button switch (SW4 and SW2)
void init_SW4_SW2(void)
{
    //ATDDIEN = 0x00F0;	      //Make PAD7-PAD4 as digital Input for SW4-Sw1. (The default is analog for ADC) 
    
    ATDDIEN_IEN7 = 1;	    //make port1AD7 a digital pin, SW4
    ATDDIEN_IEN5 = 1;	    //make port1AD5 a digital pin, SW2

    DDR1AD_DDR1AD7 = 0;   //SW4 of MCU board (port1AD7 - input)
    DDR1AD_DDR1AD5 = 0;   //SW2 of MCU board (port1AD5 - input)

    PPS1AD_PPS1AD7 = 0;   //pull-up port1AD7; 
    PPS1AD_PPS1AD5 = 0;   //pull-up port1AD5; 

    PER1AD_PER1AD7 = 1;   //enable pulls
    PER1AD_PER1AD5 = 1;   //enable pulls
}

void delay10mS()
{
  
  TC0 = TCNT + 62500;                     // 62500/6.25MHz = 10mS
  while(!(TFLG1 & 0x01))
     ;;
}


