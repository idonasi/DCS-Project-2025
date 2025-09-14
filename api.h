#ifndef _api_H_
#define _api_H_

#include  "../header/halGPIO.h"     // private library - HAL layer


//new version
extern void move_to_angle(int angle);
extern void scan_distance(int x, int y, int receive_max);
extern void scan_light(int x, int y);
extern void Light_initializer();
extern void send_calibration();
extern void telemeter();
extern int receive_angle();
extern void receive_max_distance();
extern void scan_objects_and_lights_bonus(int x, int y);


extern void write_to_segB( unsigned char *array);
extern void flash_read_samples(unsigned char *array);
extern void Activate_Script(int file_number);
extern void Execute_func(int opc,int secondbyte,int thirdbyte);

extern void set_delay(int delay);
extern void inc_lcd(int x);
extern void dec_lcd(int x);
extern void rra_lcd(int x);
extern void clear_lcd();
extern void servo_deg(int p);
extern void servo_scan(int l, int r);
extern void show_files();
extern void Read_text(int file_number);
extern void Activate_Script(int file_number);
extern void Execute_func(int opc,int secondbyte,int thirdbyte);
extern void show_files();

extern int j;
extern unsigned char calibration_array[10];
extern unsigned char test[10];

#define Seg_B_addr 0x1080

#endif


