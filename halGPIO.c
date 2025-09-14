#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"
// Global Variables
extern int j ;
int flag = 0;
unsigned int rising_capture = 0;
unsigned int falling_capture = 0;
unsigned int num_of_cycles = 0;


// new version
char arr_to_comp[Buffersize];
unsigned int head = 0;
unsigned int tail = 0;

int motor_counter = 0;
int motor_counter_max = 0;

int ADC_result = 0;
int LDR1 = 0;
int LDR2 = 0;

int receiving_angle = 0;
int angle_from_pc = 0;
int receiving_distance = 0;
int max_distance_from_pc = 0;
int low_byte = 0;
int high_byte = 0;
int byte_counter = 0;

int d = 50;
int timer_count_cycles = 0;
// New Variables for file_mode
unsigned int *ptrs; 
unsigned int file_count;
unsigned int start_addr;
unsigned int last_ptr;
unsigned int prev_header;
unsigned int last_length;
unsigned int curr_pointer;
char buf[2];
char big_buf[20];
int  buf_cnt = 0;
int expecting_bytes = 0;
int header;
int word_to_flash;
int last_Data_length;
int last_Name_length;
int chosen;
int Tag_bit;


volatile unsigned int start_time = 0;
volatile unsigned int pulse_width = 10;
volatile unsigned char capture_state = 0; // 0 = מחכה לעלייה, 1 = מחכה לירידה

//--------------------------------------------------------------------
//             System Configuration  
//--------------------------------------------------------------------
void sysConfig(void){ 
	GPIOconfig();
	//Timer0_Setup();
	//StopAllTimers();
	lcd_init();
	lcd_clear();
	UART_init();
}

void StopAllTimers(){
	TA0CTL = TASSEL_2 | MC_0;
	TA1CTL = TASSEL_2 | MC_0;
	TA0CCTL1 &= ~OUTMOD_7;
	TA0CCTL1 &= ~CCIE;
	TA1CCTL2 &= ~OUTMOD_7;
	TA1CCTL2 &= ~CCIE;
}


void TIMER_A1config_for_motor() {
	TA1CTL = MC_1 | TASSEL_2 | TACLR; // up mode | SMCLK
	TA1CCR0 = 30000; //26213
	TA1CCR2 = 618;
	//TA1CCTL2 = OUTMOD_7 + CCIE;
}

void update_angle(int angle) {
	TA1CCR2 = 618 + (int)(9.7*angle);
}

void pause_motor() {
	TA1CCTL2 &= ~OUTMOD_7;
	TA1CCTL2 &= ~CCIE;
}

void continue_motor() {
	TA1CCTL2 |= OUTMOD_7;
	TA1CCTL2 |= CCIE;
}

void first_last_angle() {
	motor_counter_max = 25;
}

void consecutive_angle() {
	motor_counter_max = 5;
}

void measure_light() {
	ADC_config();
	
	//sample LDR2
	ADC10CTL1 &= ~INCH_15; 
	ADC10CTL1 |= INCH_3;
    ADC10CTL0 |= ENC + ADC10SC;             // Start sampling
    __bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
	ADC10CTL0 &= ~ENC;
	LDR2 = ADC_result;
	
	//sample LDR1
	ADC10CTL1 &= ~INCH_15; 
	ADC10CTL1 |= INCH_0;
	ADC10CTL0 |= ENC + ADC10SC;             // Start sampling
	__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
    ADC10CTL0 &= ~ADC10ON; // Don't get into interrupt
	LDR1 = ADC_result;
	
	ADC_result = (LDR1 + LDR2) / 2;
	ADC_result = (ADC_result >> 2) & 0xFF;
	if((state == state6) || ((state == state3) && (ADC_result < 255))) {
		send_angle_and_ldr(j, ADC_result);
	}
}

void measure_light_for_bonus() {
	ADC_config();
	
	//sample LDR2
	ADC10CTL1 &= ~INCH_15; 
	ADC10CTL1 |= INCH_3;
    ADC10CTL0 |= ENC + ADC10SC;             // Start sampling
    __bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
	ADC10CTL0 &= ~ENC;
	LDR2 = ADC_result;
	
	//sample LDR1
	ADC10CTL1 &= ~INCH_15; 
	ADC10CTL1 |= INCH_0;
	ADC10CTL0 |= ENC + ADC10SC;             // Start sampling
	__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
    ADC10CTL0 &= ~ADC10ON; // Don't get into interrupt
	LDR1 = ADC_result;
	
	ADC_result = (LDR1 + LDR2) / 2;
	ADC_result = (ADC_result >> 2) & 0xFF;
	//if((state == state6) || ((state == state3) && (ADC_result < 255))) {
	//	send_angle_and_ldr(j, ADC_result);
	//}
}



void config_trigger_out(){
	TA0CCR0 = 65535;                      // Counts up to 65535 (period for PWM) , one cycle one usec (clock is 1MHz)
    TA0CCR1 = 15;                         // Duty cycle A=15 (usec) for trigger
    TA0CCTL1 = OUTMOD_7;                  //pwm mode
    TA0CTL |=  MC_1 + TASSEL_2 + CCIE;       //Continious up CCR0 + SMCLK
}

void config_echo_input_capture(){
	TA1CCTL1 = CM_1 | CCIS_1 | CAP | SCS | CCIE;
	TA1CTL = TASSEL_2 | MC_2 | TACLR;
}


void send_angle_and_distance(int angle, int distance){
	arr_to_comp[tail] = (unsigned char)(angle & 0xFF);
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)(distance & 0xFF) ;  //  add the lower byte - little endian
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)((distance >> 8) & 0xFF); // add the upper byte
	tail = (tail+1)%Buffersize;
	IE2 &= ~UCA0RXIE; 
	IE2 |= UCA0TXIE;
}

void send_angle_and_ldr(int angle, int distance){
	arr_to_comp[tail] = (unsigned char)(angle & 0xFF);
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)(distance & 0xFF); // add the 8 MSB, discard the right 2 bits
	tail = (tail+1)%Buffersize;
	IE2 &= ~UCA0RXIE; 
	IE2 |= UCA0TXIE;
}

void send_angle_and_data_for_bonus(int angle, int distance, int adc){
	arr_to_comp[tail] = (unsigned char)(angle & 0xFF);
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)(distance & 0xFF) ;  //  add the lower byte - little endian
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)((distance >> 8) & 0xFF); // add the upper byte
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)(adc & 0xFF); // add the 8 MSB, discard the right 2 bits
	tail = (tail+1)%Buffersize;
	IE2 &= ~UCA0RXIE;
	IE2 |= UCA0TXIE;
}

void send_angle_and_ldr_for_bonus(int angle, int adc){
	arr_to_comp[tail] = (unsigned char)(200 & 0xFF); // signal the point is light point
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)(angle & 0xFF);
	tail = (tail+1)%Buffersize;
	arr_to_comp[tail] = (unsigned char)(adc & 0xFF); // add the 8 MSB, discard the right 2 bits
	tail = (tail+1)%Buffersize;
	IE2 &= ~UCA0RXIE; 
	IE2 |= UCA0TXIE;
}



void send_final_char(){
	arr_to_comp[tail] = TERMINATOR;
	tail = (tail + 1) % Buffersize;
	IE2 &= ~UCA0RXIE; 
	IE2 |= UCA0TXIE;
}

void send_ack(int ack){
	head = tail;
	while (!(IFG2 & UCA0TXIFG));  // מחכה שפנוי לשליחה
	UCA0TXBUF = ack;
}


void timer_config_for_script() {
    timer_count_cycles = d;
	
    TA0CCR0 = 10000;  //10,000 cycles --> 10ms
    TA0CTL = TASSEL_2 | MC_1 | TACLR;
	TA0CCTL0 |= CCIE;
}

void timer_stop_for_script() {
	TA0CTL = TASSEL_2 | MC_0;
	TA0CCTL0 &= ~CCIE;
}


//---------------------------------------------------------------------
//       Enable / Disable - interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
	_BIS_SR(GIE);
}

void disable_interrupts(){
	_BIC_SR(GIE);
}

void enable_PB0(){
	Port2IntEn |= 0x01;
	
}

void disable_PB0() {
	Port2IntEn &= ~0x01;
}

//---------------**************************----------------------------
//               Interrupt Services Routines
//---------------**************************----------------------------
//*********************************************************************
//               PORT2 ISR - PB0 P2.0
//*********************************************************************
#pragma vector = PORT2_VECTOR
 __interrupt void PBs_handler(void) {
	delay(debounceVal);
	if(Port2IntPend & PB0) {
		Port2IntPend &= ~PB0;
	}
	else if ( Port2IntPend & PB1) {
		Port2IntPend &= ~PB1;
		if( chosen == 0) {
			state = state0;
		}
		else if (Tag_bit == 0) {
			chosen = 0;
		}
		else if ( Tag_bit == 1) {
			chosen = 1;
		}
	}
	
	switch(lpm_mode){
		case mode0:
			LPM0_EXIT; // must be called from ISR only
			break;
		case mode1:
			LPM1_EXIT; // must be called from ISR only
			break;
		case mode2:
			LPM2_EXIT; // must be called from ISR only
			break;
		case mode3:
			LPM3_EXIT; // must be called from ISR only
			break;
		case mode4:
			LPM4_EXIT; // must be called from ISR only
			break;
	}
}

//*********************************************************************
//                        TIMER A0 ISR
//*********************************************************************
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TimerA0_ISR(void)
{
	if (state == state5 || state == state9){
		timer_count_cycles--;
		if (timer_count_cycles <= 0){
			timer_count_cycles = d;
			__bic_SR_register_on_exit(LPM0_bits);
		}
	}
}

//*********************************************************************
//                        TIMER A1 ISR
//*********************************************************************
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR(void)
{
	switch (__even_in_range(TA1IV, 10))
    {
        case TA1IV_NONE:   
			break;
        case TA1IV_TACCR1: // Interrupt from TA1CCR1
			if (capture_state == 0) {
				// עליה -> שמור זמן התחלה
				start_time = TA1CCR1;
				capture_state = 1;

				// עכשיו נחפש ירידה
				TA1CCTL1 &= ~CM_3;
				TA1CCTL1 |= CM_2; // ירידה בלבד
			}
			else {
				// ירידה -> חישוב רוחב
				unsigned int end_time = TA1CCR1;
				if (end_time >= start_time) {
					pulse_width = end_time - start_time;
				} else {
					// rollover של המונה
					pulse_width = (0xFFFF - start_time) + end_time + 1;
				}

				capture_state = 0;

				// חזרה לחפש עליה
				TA1CCTL1 &= ~CM_3;
				TA1CCTL1 |= CM_1; // עליה בלבד
				
				__bic_SR_register_on_exit(CPUOFF); //exit LPM0
			}
			break;
			
		case TA1IV_TACCR2:  
				// Interrupt from TA1CCR2
			if(motor_counter >= motor_counter_max) {
				motor_counter = 0;
				__bic_SR_register_on_exit(LPM0_bits);  //out from sleep	motor
			}
			else {
				motor_counter++;
			}
			break;
		
		case TA1IV_TAIFG:              // Timer1_A overflow interrupt
            // ...
			break;
	}	
}

//*********************************************************************
//                         ADC10 ISR
//*********************************************************************
#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
	ADC_result = ADC10MEM;
    __bic_SR_register_on_exit(CPUOFF);
}

//*********************************************************************
//                           TX ISR
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
	if(head != tail){
		UCA0TXBUF = arr_to_comp[head];
		head = (head + 1) % Buffersize;	
	}
	else {
		IE2 &= ~UCA0TXIE;
		IE2 |= UCA0RXIE;
	}
	
	/*
	switch(lpm_mode){
    case mode0:
        LPM0_EXIT; // must be called from ISR only
        break;
    case mode1:
        LPM1_EXIT; // must be called from ISR only
        break;
    case mode2:
        LPM2_EXIT; // must be called from ISR only
        break;
    case mode3:
        LPM3_EXIT; // must be called from ISR only
        break;
    case mode4:
        LPM4_EXIT; // must be called from ISR only
        break;
    }
	*/
}


//*********************************************************************
//                         RX ISR
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{ 	
	char received = UCA0RXBUF; 
	if(receiving_angle == 1){
		angle_from_pc = received;
		receiving_angle = 0;
	}
	else if(receiving_distance == 2){
		receiving_distance = 1;
		low_byte = received;
	}
	else if(receiving_distance == 1){
		receiving_distance = 0;
		high_byte = received;
	}
	else if (state == state5 && byte_counter == 0) {
		byte_counter += 1;
		buf[0] = received;	
	}
	else if (state == state5 && byte_counter == 1) {
		byte_counter += 1;
		buf[1] = received;
		header =  (buf[0] << 8) | buf[1] ;	
		buf_cnt = 0;
		int data_length = (header >> 4) & 0x7FF;
		int name_length =  header & 0xF;
		expecting_bytes = data_length +name_length; //Need to add Name_length
		curr_pointer = initial_flash_write(header); // Write header and Pointer into Memory Each for his own location, calculate current pointer to write next word.
	}
	else if (state == state5 && expecting_bytes > 0) {
		expecting_bytes -= 1;
		if( buf_cnt == 0) {
			buf[0] = received;
			buf_cnt += 1;	
		}
		else {
			buf[1] = received;
			word_to_flash =  (buf[0] << 8) | buf[1]; 
			flash_write_word(curr_pointer, word_to_flash);
			curr_pointer -= 2; //
			buf_cnt = 0;
		}
		if (expecting_bytes == 0) {
			if(buf_cnt == 1) { // Meaning that there is odd number of Bytes so need to send only 1 Byte with padding
				word_to_flash = (buf[0] << 8);
				flash_write_word(curr_pointer, word_to_flash);
				buf_cnt = 0;
			}
			byte_counter = 0;
			send_ack(5);
			state = state0; // Just for case that there is more data in RX Buffer 
		}
	}
	else if(received == 0){
        state = state0;
		send_ack(received);
    }
    else if(received == 1){
        state = state1;
		send_ack(received);
    }
    else if(received == 2){
        state = state2;
		send_ack(received);
    }
	else if(received == 3){
		state = state3;
		send_ack(received);
    }
    else if(received == 4){
        state = state4;
		send_ack(received);
    }
    else if(received == 5){
        state = state5;
		send_ack(received);
    }
	else if(received == 6){
		state = state6;
		send_ack(received);
	}
	else if(received == 7){
		
		send_ack(received);
		state = state7;
	}
	else if(received == 8){
		state = state8;
		send_ack(received);
	}
	else if(received == 9){
		state = state9;
		send_ack(received);
	}
	
    switch(lpm_mode){
    case mode0:
        LPM0_EXIT; // must be called from ISR only
        break;
    case mode1:
        LPM1_EXIT; // must be called from ISR only
        break;
    case mode2:
        LPM2_EXIT; // must be called from ISR only
        break;
    case mode3:
        LPM3_EXIT; // must be called from ISR only
        break;
    case mode4:
        LPM4_EXIT; // must be called from ISR only
        break;
    }
}


//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
		_BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
    else if(LPM_level == 0x01) 
		_BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
    else if(LPM_level == 0x02) 
		_BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
		_BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
    else if(LPM_level == 0x04) 
		_BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Flash Functions 
//---------------------------------------------------------------------
unsigned int initial_flash_write( unsigned int curr_header){
	 ptrs = flash_get_pointers(); // point to first pointer in segment C
     file_count = flash_get_file_count(); //Num of files 
	
	 if (file_count == 0) {
        start_addr = SEG1_END; 
    } 
	else {
         last_ptr = ptrs[file_count - 1]; // Get location of last saved file 
         prev_header = *(unsigned int*)last_ptr; //get header of last file
         last_Data_length = (prev_header >> 4) & 0x7FF; // Get length of previous data file entered to flash
		 last_Name_length = prev_header & 0xF;
		 if((last_Data_length+last_Name_length)%2 ==0){
         start_addr = last_ptr - (last_Name_length + last_Data_length + 2); // Calculate where to start to write new data
		 }
		 else {
		 start_addr = last_ptr - (last_Name_length + last_Data_length + 3); // Calculate where to start to write new data 
		 }
    }
	
	flash_write_word((unsigned int)&ptrs[file_count], start_addr); // Save start pointer of new file in Segment C
	
	flash_write_word(start_addr, curr_header);  // Enter Header to memory
	return start_addr - 2 ; //after Write header move the pointer one word forward meaning = decrease one like stack.
}

void flash_write_word(unsigned int addr, unsigned int data) { // function to write first word 
    unsigned int *ptr = (unsigned int*)addr;				  // Header Word = [type,file_length,file_name_length] =[15,14-4,3-0] ( Order in Bits )
    flash_unlock();
    *ptr = data;
	while (FCTL3 & BUSY); 
    flash_lock();
}

void flash_unlock() { // Unlock and set to Write Mode
    FCTL3 = FWKEY;            
    FCTL1 = FWKEY + WRT;       
}

void flash_lock() { // Locking Flash 
    FCTL1 = FWKEY;             
    FCTL3 = FWKEY + LOCK; 
}

unsigned int* flash_get_pointers() { //Get pointers of files from segment C
    return (unsigned int*)Seg_C_addr;
}

unsigned int flash_get_file_count() { //Count number of pointers saved in segment C
    unsigned int *ptrs = (unsigned int*)Seg_C_addr;
    unsigned int count = 0;

    while (count < MAX_FILES && ptrs[count] != 0xFFFF)
        count++;

    return count;
}






//---------------------------------------------------------------------
//            LCD
//---------------------------------------------------------------------
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
    while(*s)
        lcd_data(*s++);
}
//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){
    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
		LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
	LCD_EN(1);
	asm("NOP");
	// asm("NOP");
	LCD_EN(0);
}

//---------------------------------------------------------------------
//                     Polling delays
//---------------------------------------------------------------------
//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec

}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec

}
//******************************************************************
//            Polling based Delay function
//******************************************************************
void delay(unsigned int t){  //
    volatile unsigned int i;

    for(i=t; i>0; i--);
}

void int2str(char *str, unsigned int num){
    int strSize = 0;
    long tmp = num, len = 0;
    int q;
    // Find the size of the intPart by repeatedly dividing by 10
    while(tmp){
        len++;
        tmp /= 10;
    }

    // Print out the numbers in reverse
    for(q = len - 1; q >= 0; q--){
        str[q] = (num % 10) + '0';
        num /= 10;
    }
    strSize += len;
    str[strSize] = '\0';
}
