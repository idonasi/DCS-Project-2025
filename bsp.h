#ifndef _bsp_H_
#define _bsp_H_

#include  <msp430g2553.h>          // MSP430x2xx
//#include  <msp430xG46x.h>  // MSP430x4xx


#define   debounceVal      500


//#define   LEDs_SHOW_RATE   0xFFFF  // 62_5ms

// RGB abstraction
#define RGBArrPortOut      P2OUT
#define RGBArrPortDir      P2DIR
#define RGBArrPortSEL      P2SEL

// LEDs abstraction
//#define LEDsArrPort        P1OUT
//#define LEDsArrPortDir     P1DIR
//#define LEDsArrPortSel     P1SEL

// LCDs abstraction
#define LCD_DATA_WRITE     P1OUT
#define LCD_DATA_DIR       P1DIR
#define LCD_DATA_READ      P1IN
#define LCD_DATA_SEL       P1SEL
#define LCD_CTL_SEL        P2SEL


//   Buzzer abstraction
#define BuzzPortSel        P2SEL
#define BuzzPortDir        P2DIR
#define BuzzPortOut        P2OUT


//// Port2 configure
#define Port2IN         	P2IN
#define Port2IntPend     	P2IFG
#define Port2IntEn        	P2IE
#define Port2IntEdgeSel   	P2IES
#define Port2Sel      		P2SEL
#define Port2Sel2      		P2SEL2
#define Port2Dir      		P2DIR
#define Port2Out      		P2OUT

// Port1 configure
#define Port1IN		       P1IN
#define Port1IntPend	   P1IFG
#define Port1IntEn		   P1IE
#define Port1IntEdgeSel	   P1IES
#define Port1Sel	       P1SEL
#define Port1Dir	       P1DIR
#define Port1Out	       P1OUT
#define PB0                0x01 // P2.0   
#define PB1                0x80 // P2.7 
//#define PB2                0x04 // P2.2 
//#define PB3                0x08 // P2.3

// UART
#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1


extern void GPIOconfig();
extern void ADC_config();
extern void TIMER_A0_config(unsigned int counter);
extern void TIMERB_config();
extern void TIMERB_config_Task3();
extern void Timer0_Setup();
extern void UART_init();
extern void TIMER_A1config();


#endif

