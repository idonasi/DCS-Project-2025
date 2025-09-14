#include  "../header/bsp.h"    // private library - BSP layer

//-----------------------------------------------------------------------------  
//           GPIO configuration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
    WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
	
	// LCD configuration
	LCD_DATA_WRITE 	&= ~0xFF;
	LCD_DATA_DIR 	|= 0xF0;    // P1.4-P1.7 To Output('1')
	LCD_DATA_SEL 	&= ~0xF0;   // Bit clear P1.4-P1.7
	LCD_CTL_SEL  	&= 0xD5;   // Bit clear P2.1,P2.3,P2.5
	
	//PB0 Setup
	Port2Sel 		&= ~0x81;
	Port2Dir 		&= ~0x81;
	//Port2Out 		&= ~0x01;
	Port2IntEdgeSel |=  0x81;
	Port2IntEn 		|=  0x81;
	Port2IntPend 	&= ~0x81;

	////P2.6 for output Trigger 
	Port2Dir 		|= 0x40;
	Port2Sel 		|= 0x40; 
	Port2Sel 		&= ~0x80; 
	Port2Sel2		&= ~(0x40 | 0x80);
	//Port2Out        &= ~0x40;
	
	//P2.2 for input capture
	Port2Dir 		&= ~BIT2;
	Port2Sel 		|= BIT2;
	Port2Sel2 		&= ~BIT2;
	
	//P2.4 for PWM motor
	Port2Dir        |= 0x10;  // output configuration 
	Port2Sel        |= 0x10;  // Timer listening configuration
	
	_BIS_SR(GIE);                     // enable interrupts globally
}

//-------------------------------------------------------------------------------------
//            ADC configuration
//-------------------------------------------------------------------------------------
void ADC_config(void){
	ADC10CTL0 = ADC10SHT_2 + ADC10ON+ SREF_0 + ADC10IE;  // 16*ADCLK+ Turn on, set ref to Vcc and Gnd, and Enable Interrupt
    ADC10CTL1 = ADC10SSEL_3;    				// Input A3 and SMCLK, was |
    ADC10AE0 |= BIT3;                         // P1.3 ADC option select
	ADC10AE0 |= BIT0;							// P1.0 ADC option select
}


//-------------------------------------------------------------------------------------
//                              UART init
//-------------------------------------------------------------------------------------
void UART_init(void){
    if (CALBC1_1MHZ==0xFF){                  // If calibration constant erased
		while(1);                            // do not load, trap CPU!!
	}
    DCOCTL = 0;                              // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                   // Set DCO
    DCOCTL = CALDCO_1MHZ;

  //  P2DIR = 0xFF;                             // All P2.x outputs
  //  P2OUT = 0;                                // All P2.x reset
    P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  //  P1DIR |= RXLED + TXLED;
    P1OUT &= 0xF9; // Only P1.1 and P1.2 equal to zero

    UCA0CTL1 |= UCSSEL_2;                     // CLK = SMCLK
    UCA0BR0 = 104;                            //
    UCA0BR1 = 0x00;                           //
    UCA0MCTL = UCBRS0;             			  //
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}
