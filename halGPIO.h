#ifndef _halGPIO_H_
#define _halGPIO_H_

#include  "../header/bsp.h"    		// private library - BSP layer
#include  "../header/app.h"    		// private library - APP layer
#include  <msp430g2553.h> 

extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable
extern char string1[3];
extern unsigned int delay_time;
extern int delay_ifg;
extern int Telemeter_angle;
extern unsigned int num_of_cycles;

extern int flag;

#define clk_tmp 131; // ( (2^20) / 8 )*(10^-3) to convert ms to counter value for TACCR0
#define Buffersize 180

extern void sysConfig(void);
extern void print_Tone(int);
extern void Start_Timer1();
extern void Start_Timer0();
extern void StopAllTimers(void);
extern void SetByteToPort(char);
extern void clrPortByte(char);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();
extern void timer_call_counter();

extern void int2str(char *str, unsigned int num);
extern void measure_light();
extern void send_final_char();
extern void send_trigger();
extern void turn_motor(int angle);
extern void configure_timers_state1();
extern void configure_trigger();
extern void configure_input_capture();


extern __interrupt void PBs_handler(void);
extern __interrupt void PBs_handler_P2(void);
extern __interrupt void Timer_A0(void);
extern __interrupt void Timer_A1(void);
extern __interrupt void USCI0RX_ISR(void);
extern __interrupt void USCI0TX_ISR(void);

#endif


/*----------------------------------------------------------
		L_C_D
------------------------------------------------------------*/

// #define CHECKBUSY    1  // using this define, only if we want to read from LCD

#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P2OUT&=~0X02) : (P2OUT|=0X02)) // P2.1 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X02) : (P2DIR|=0X02)) // P2.1 pin direction

#define LCD_RS(a)   (!a ? (P2OUT&=~0X08) : (P2OUT|=0X08)) // P2.3 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X08) : (P2DIR|=0X08)) // P2.3 pin direction

#define LCD_RW(a)   (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off()        lcd_cmd(0x0C)
#define cursor_on()         lcd_cmd(0x0F)
#define lcd_function_set    lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line()      lcd_cmd(0xC0)

extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);

/*
 *  Delay functions for HI-TECH C on the PIC18
 *
 *  Functions available:
 *      DelayUs(x)  Delay specified number of microseconds
 *      DelayMs(x)  Delay specified number of milliseconds
*/

//new version 
extern void int2str(char *str, unsigned int num);
extern void StopAllTimers();
extern void TIMER_A1config_for_motor();
extern void pause_motor();
extern void continue_motor();
extern void first_last_angle();
extern void consecutive_angle();
extern void update_angle(int angle);
extern void measure_light();
extern void measure_light_for_bonus();
extern void config_trigger_out();
extern void config_echo_input_capture();

extern void send_angle_and_distance(int angle, int distance);
extern void send_angle_and_ldr(int angle, int distance);

extern void send_angle_and_data_for_bonus(int angle, int distance, int adc);


extern void send_final_char();
extern void disable_PB0();
extern void enable_PB0();

extern void timer_config_for_script();
extern void timer_stop_for_script();


extern int ADC_result;
extern char arr_to_comp[];
extern unsigned int head;
extern unsigned int tail;

extern volatile unsigned int start_time;
extern volatile unsigned int pulse_width;
extern volatile unsigned char capture_state;
extern int receiving_angle;
extern int angle_from_pc;
extern int receiving_distance;
extern int max_distance_from_pc;
extern int low_byte;
extern int high_byte;

extern int d;
extern int timer_count_cycles;

#define TERMINATOR 0xFE
#define long_distance 0xEEEE

//New defines for Filemode
#define MAX_FILES  10
#define Seg_C_addr 0x1040
#define SEG1_END 0xFDFE
extern unsigned int initial_flash_write(unsigned int curr_header);
extern void flash_write_word(unsigned int addr, unsigned int data);
extern void flash_unlock();
extern void flash_lock();
extern unsigned int* flash_get_pointers();
extern unsigned int flash_get_file_count();

extern int Tag_bit;
extern int chosen;

