#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"
#include <stdlib.h>
#include <string.h>

int j = 0;
char str_to_lcd[10];
unsigned char calibration_array[10];
unsigned char test[10];

void move_to_angle(int angle){
	first_last_angle();
	TIMER_A1config_for_motor();
	update_angle(angle);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);
	pause_motor();
	StopAllTimers();
}

void scan_distance(int x, int y, int receive_max){
	if (receive_max == 0){
		receive_max_distance();
	}
	else {
		max_distance_from_pc = 24672;
	}
	
	j = x;
	first_last_angle();
	TIMER_A1config_for_motor();
	update_angle(x);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);
	consecutive_angle();
	
	while (j <=  y) {
		TIMER_A1config_for_motor();
		update_angle(j);
		continue_motor();
		__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
		pause_motor();
		
		// sample distance using Ultrasonic
		config_trigger_out();
		config_echo_input_capture();
		
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
		
		if (pulse_width <= max_distance_from_pc) {  //pulse_width < 24672 --> less than 400 cm
			send_angle_and_distance(j, pulse_width);
		}
		
		pulse_width = (int)(pulse_width * 2125.0 / 131072.0);
		int2str(str_to_lcd, pulse_width);
		lcd_clear();
		lcd_puts(str_to_lcd);
		lcd_puts(" cm");
		// end sample
		
		j++;
		update_angle(j);
	}
	
	TIMER_A1config_for_motor();
	first_last_angle();
	update_angle(0);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
	pause_motor();
	send_final_char();
}


void telemeter(){
	j = receive_angle();
	
	first_last_angle();
	TIMER_A1config_for_motor();
	update_angle(j);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);
	pause_motor();
	//StopAllTimers();
	//enable_interrupts();
	
	lcd_clear();
	lcd_home();
    lcd_puts("input: ");
	
	config_trigger_out();
	config_echo_input_capture();
	
	while(state == state2){
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
		
        lcd_home();
		lcd_cursor_right();//1
		lcd_cursor_right();//2
		lcd_cursor_right();//3
		lcd_cursor_right();//4
		lcd_cursor_right();//5
		lcd_cursor_right();//6
		lcd_cursor_right();//7
		
		if (pulse_width < 25000) {  //pulse_width < 24672 --> less than 400 cm
			send_angle_and_distance(angle_from_pc, pulse_width);
		}
		else {
			send_angle_and_distance(angle_from_pc, long_distance);
		}
		
		pulse_width = (int)(pulse_width * 2125.0 / 131072.0);
		int2str(str_to_lcd, pulse_width);
		lcd_puts(str_to_lcd);
		lcd_puts(" cm");
    }
	StopAllTimers();
}

void scan_light(int x, int y) {
	j = x;
	first_last_angle();
	TIMER_A1config_for_motor();
	update_angle(x);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);
	consecutive_angle();
	
	while (j <=  y) {
		continue_motor();
		__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
		pause_motor();
		// sample light using LDR
		measure_light();
		j++;
		update_angle(j);
	}
	
	first_last_angle();
	update_angle(90);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
	pause_motor();
	send_final_char();
}


void Light_initializer() {
	first_last_angle();
	TIMER_A1config_for_motor();
	update_angle(90);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);
	pause_motor();
	
	j = 0;
	enable_PB0();

	while (j < 10){
		int2str(str_to_lcd, ((j+1)*5));
		lcd_clear();
		lcd_puts("Calibrate LDR");
		lcd_new_line();
		lcd_puts(str_to_lcd);
		lcd_puts(" cm");
		
	    __bis_SR_register(LPM0_bits + GIE);
		measure_light();
		calibration_array[j] = ADC_result;
		j++;
	}
	disable_PB0();
	send_final_char();
	write_to_segB(calibration_array);
}

void send_calibration(){
	j = 0;
	delay(50000);
	//write_to_segB(calibration_array);
	flash_read_samples(test);
	while (j < 10){
		//delay(debounceVal);
		send_angle_and_ldr(j, test[j]);
		j++;
	}
	send_final_char();
}


int receive_angle(){
	receiving_angle = 1;
	if (receiving_angle == 1){
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
	}
	return angle_from_pc;
}


void receive_max_distance(){
	receiving_distance = 2;
	if (receiving_distance != 0){
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
	}
	if (receiving_distance != 0){
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
	}
	max_distance_from_pc = ((unsigned int)high_byte << 8) | low_byte;
}


void scan_objects_and_lights_bonus(int x, int y){
	receive_max_distance();
	
	j = x;
	first_last_angle();
	TIMER_A1config_for_motor();
	update_angle(x);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);
	consecutive_angle();
	
	while (j <=  y) {
		TIMER_A1config_for_motor();
		update_angle(j);
		continue_motor();
		__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
		pause_motor();
		measure_light_for_bonus();
		
		
		// sample distance using Ultrasonic
		config_trigger_out();
		config_echo_input_capture();
		
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
		
		if (pulse_width <= max_distance_from_pc || ADC_result < 255) {  //pulse_width < 24672 --> less than 400 cm
			//send_angle_and_distance_for_bonus(j, pulse_width);
			send_angle_and_data_for_bonus(j, pulse_width, ADC_result);
		}
		
		pulse_width = (int)(pulse_width * 2125.0 / 131072.0);
		int2str(str_to_lcd, pulse_width);
		lcd_clear();
		lcd_puts(str_to_lcd);
		lcd_puts(" cm");
		// end sample
		
		j++;
		update_angle(j);
	}
	first_last_angle();
	update_angle(90);
	continue_motor();
	__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
	pause_motor();
	send_final_char();
}


//--------------------------------------------------------------------
//             File mode Functions  
//--------------------------------------------------------------------
// Functions to store LDR samples 
void write_to_segB( unsigned char *array){
	unsigned int q;
	volatile unsigned int *flash_ptr = (volatile unsigned int *)Seg_B_addr;
	unsigned int len = 10;
	
	 // מחיקת סגמנט B לפני כתיבה
    FCTL3 = FWKEY;                 // Unlock Flash
    FCTL1 = FWKEY + ERASE;         // Erase mode
    *flash_ptr = 0;                // Erasing whole segment B (switching calibration values)
    while (FCTL3 & BUSY);          // Waiting
    FCTL1 = FWKEY;                 // EXIT ERASE mode
    FCTL3 = FWKEY + LOCK;          // Locking

    FCTL3 = FWKEY;				   // Unlock Flash
    FCTL1 = FWKEY + WRT;		   // Write mode

    for (q = 0; q < len; q += 2) {
        //unsigned int word = array[q]; // Flash writing Word only, so need to combine 2 samples in order to reduced time  
		
        unsigned int word = ((unsigned int)array[q] << 8); // Flash writing Word only, so need to combine 2 samples in order to reduced time                
        if (q + 1 < len) {
            word |= ((unsigned int)array[q+1]); 
        }
        *flash_ptr++ = word;      
        while (FCTL3 & BUSY);    
    }

    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
}


void flash_read_samples(unsigned char *array) // Samples always stored in segment B
{
	unsigned int len = 10;
    unsigned int q;
    volatile unsigned int *flash_ptr = (volatile unsigned int *)Seg_B_addr;  // Pointer to samples array in flash 

    for (q = 0; q < len; q+= 2)
    {
		unsigned int word = *flash_ptr++;
		array[q] = (word >> 8) & 0xFF;
        array[q+1] = word & 0xFF;
	}
}
//----------------------------------------------------
//			Script functions
//---------------------------------------------------- 
void show_files() {
	chosen = 2;
	unsigned int  *ptr = (unsigned int*)0x1040; //Start of segment C where all my files pointers are located
	unsigned int files_number = flash_get_file_count();
	char *Pointer_Firstline;
	char *Pointer_Secondline;
	char Firstline[15]; // Every line on LCD is 16 Chars 
    char Secondline[15];
    int q;
	int end = 0;
	int count_Firstline_file = -1;
	unsigned int First_header;
	unsigned int Second_header;
    int Firstfile_length;
	int Secondfile_length;
	if (files_number != 0) {
		while (state == state9){
			if ( end == 0) {
				Pointer_Firstline = (char*)*ptr;
				ptr ++;
				First_header = *((unsigned int*)Pointer_Firstline);
				Tag_bit = (First_header >> 15) & 0x1;
				Firstfile_length = First_header & 0xF;
				Pointer_Firstline--;
				memset(Firstline, 0, sizeof(Firstline)); 
				for (q = 0 ; q<Firstfile_length;q++){
					Firstline[q] =(char)*Pointer_Firstline;
					Pointer_Firstline--;
				}
				lcd_clear();
				lcd_home();
				lcd_puts(Firstline);
				count_Firstline_file++;
				lcd_cmd(0xc0);
				memset(Secondline, 0, sizeof(Secondline));
				if(*ptr != 0xFFFF){
					Pointer_Secondline = (char*)*ptr;
					Second_header = *((unsigned int*)Pointer_Secondline);
					Secondfile_length = Second_header & 0xF;
					Pointer_Secondline-- ;
					for (q = 0 ; q<Secondfile_length;q++){
						Secondline[q] = (char)*Pointer_Secondline;
						Pointer_Secondline--;
					}
					lcd_puts(Secondline);
				}
				else {
					lcd_puts(Secondline);
					end = 1;	
				}
				}
				enterLPM(mode0);
				if(chosen == 0) {
					Read_text(count_Firstline_file);
				}
				else if (chosen == 1){
					Activate_Script(count_Firstline_file );
				}
		}
		chosen = 2;
	}
	else {
		lcd_clear();
        lcd_home();
        lcd_puts("No Saved Files");
		lcd_cmd(0xc0);
		lcd_puts("Going to Sleep");
	}
}
void Read_text(int file_number) {
	 unsigned int *Pointers = flash_get_pointers() ;// Pointers saved at Segment C, every pointers is Word,Pointer points to Header word of file
	 unsigned int *Text_addr =  (unsigned int*)Pointers[file_number]; //Save address of chosen file to activate
	 char *Text_pointer = (char*)((unsigned int*)Pointers[file_number]); // Save address of file to check later for functions
	 unsigned int  Text_header = *Text_addr; // Get header word from Flash memory
	 int Text_length = (Text_header >> 4) & 0x7FF; // Get length of data file from header
	 int Text_name_length = Text_header & 0xF; // Get length of name file from header
	 Text_pointer -= Text_name_length + 1; // Now we need to reduce 2 bytes + Name_length byts in order to read only data bytes.
     int q = 0;
	 int count_printed_bytes = 0;
     //char Secondline[15];
		while(state == state9) {
			lcd_clear();
			lcd_home();
			for(q = 0; q < 32 ; q++) {
				if( count_printed_bytes < Text_length) {
					if(*Text_pointer != 0x0D && *Text_pointer != 0x0A){
					lcd_data(*Text_pointer);
						if(q == 15) {
							lcd_cmd(0xc0);
						}
					}
					count_printed_bytes++;
					Text_pointer--;
				}
			}
			enterLPM(mode0);
			
		}
}
void Activate_Script(int file_number){
        //char StartAddressOfScript = FilleManager.Fille[NumFilleToRun].Fileaddress;
        unsigned int *Pointers = flash_get_pointers() ;// Pointers saved at Segment C, every pointers is Word,Pointer points to Header word of file
		unsigned int *Script_addr =  (unsigned int*)Pointers[file_number]; //Save address of chosen file to activate
		char *Script_pointer = (char*)((unsigned int*)Pointers[file_number]); // Save address of file to check later for functions
		unsigned int  Script_header = *Script_addr; // Get header word from Flash memory
		int Script_length = (Script_header >> 4) & 0x7FF; // Get length of data file from header
		int Script_name_length = Script_header & 0xF; // Get length of name file from header
		Script_pointer -= Script_name_length + 1; // Now we need to reduce 2 bytes + Name_length byts in order to read only data bytes.
        int i = 0;
        while (i<Script_length) {
			if(*Script_pointer != 0x0D && *Script_pointer != 0x0A){
            unsigned char First_Byte= *Script_pointer;//read First Byte from Flash memory
			Script_pointer--;
			i++;
            unsigned char Second_Byte = 0;
            unsigned char Third_Byte = 0;
            if (First_Byte==1 || First_Byte==2 || First_Byte==3 || First_Byte==4 || First_Byte==6){ // Analyze bytes in order to execute the right function 
                Second_Byte =  *Script_pointer;//readOneMoreByte
				Script_pointer--;
				i++;
            }
            else if (First_Byte==7){
                Second_Byte =  *Script_pointer;//readOneMoreByte
				Script_pointer--;
                Third_Byte =  *Script_pointer;//readOneMoreByte
				Script_pointer--;
				i += 2;
            }
            Execute_func(First_Byte,Second_Byte,Third_Byte);
			}
			else {
				i++;
			}
        }

    }

void Execute_func(int opc,int secondbyte,int thirdbyte){
    if ((opc > 8) || (opc < 0)){
        return;
    }
    if (opc == 1){
        inc_lcd(secondbyte);
    }
    else if (opc == 2){
        dec_lcd(secondbyte);
    }
    else if (opc == 3){
        rra_lcd(secondbyte);
    }
    else if (opc == 4){
        set_delay(secondbyte);
    }
    else if (opc == 5){
        clear_lcd();
    }
    else if (opc == 6){
        servo_deg(secondbyte);
    }
    else if (opc == 7){
        servo_scan(secondbyte,thirdbyte);
    }
    else if (opc == 8){
        state = state0;
    }

}


void inc_lcd(int x){ // 0x01
	int up_counter = 0;
	timer_config_for_script();
	while(up_counter <= x)
	{
		lcd_clear();
		int2str(str_to_lcd, up_counter);
		lcd_puts(str_to_lcd);
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
		up_counter ++;
	}
	lcd_clear();
	timer_stop_for_script();
}

void dec_lcd(int x){ // 0x02
	int down_counter = x;
	timer_config_for_script();
	while(down_counter >= 0)
	{
		lcd_clear();
		int2str(str_to_lcd, down_counter);
		lcd_puts(str_to_lcd);
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
		down_counter --;
	}
	lcd_clear();
	timer_stop_for_script();
}

void rra_lcd(int x){ //0x03
	timer_config_for_script();
	int current_place = 0;
	lcd_home();
    lcd_clear();
	cursor_off();
	while(current_place <= 31)
	{
		lcd_data(x);
		__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 w/ interrupt
		lcd_cursor_left();
		lcd_puts(" ");
		if(current_place == 15){
            lcd_new_line();
		}
		current_place ++;
	}
	lcd_clear();
	cursor_on();
	timer_stop_for_script();
}

void set_delay(int delay){ //0x04
	d = delay;
}

void clear_lcd(){ // 0x05
	lcd_clear();
}

void servo_deg(int p){ // 0x06
	scan_distance(p, p, 1);
}

void servo_scan(int l, int r){ //0x07
	scan_distance(l, r, 1);
}

