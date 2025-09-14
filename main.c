#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer
#include  <stdio.h>


enum FSMstate state;
enum SYSmode lpm_mode;


void main(void){
  
  state = state0;  // start in idle state on RESET
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();     // Configure GPIO, Stop Timers, Init LCD


  while(1){
	switch(state){
	case state0: //idle
		
		StopAllTimers();
		lcd_clear();
		lcd_puts("sleeping...");
	    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
	    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
	    break;
		
	case state1:
		lcd_clear();
		lcd_puts("scanning distance");
		lcd_new_line();
		lcd_puts("objects");
	    scan_distance(0, 180, 0);
		
		state = state0;
		IE2 |= UCA0RXIE;
	    break;
		
	case state2:
		lcd_clear();
		lcd_puts("Telemeter");
	    telemeter();
		
		state = state0;
	    break;

	case state3:
		lcd_clear();
		lcd_puts("scanning light");
		lcd_new_line();
		lcd_puts("sources");
		scan_light(0, 180);
		state = state0;
		IE2 |= UCA0RXIE;	
		break;
		
	case state4:
		lcd_clear();
		lcd_puts("BONUS state");
		scan_objects_and_lights_bonus(0, 180);
		state = state0;
        break;

    case state5:
	
		lcd_clear();
		lcd_puts("Writing to memory");
        break;

    case state6:
		lcd_clear();
		lcd_puts("calibrate using PB0");
		IE2 &= ~UCA0RXIE;
        Light_initializer();
		
		flash_read_samples(test);
		IE2 |= UCA0RXIE;
        state = state0;
        break;
	
	case state7:
		lcd_clear();
		lcd_puts("sending calibration");
		send_calibration();
        state = state0;
        break;

    case state8:
		lcd_clear();
		lcd_puts("state 8");
		move_to_angle(0);

        state = state0;
        break;
	
	case state9:
		enable_interrupts();
		lcd_clear();
		show_files();
        state = state0;
        break;
	}
  }
}
