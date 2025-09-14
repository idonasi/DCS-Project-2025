```
// systemReadMe.txt

// --- main.c ---
// Purpose: This is the core of the system. It contains the main state machine that controls the system's behavior based on commands received via UART. It handles the initialization of all modules and transitions between different operational states such as scanning, telemetering, and file management.

// Functions:
// main() - The primary entry point of the program. It performs initial system configuration and runs the main loop that manages the system's states.


// --- api.c ---
// Purpose: This is the Application Programming Interface (API) layer. It contains high-level functions that use the hardware-level (HAL) functions to perform complex tasks. These tasks include controlling the motor, scanning the environment, and managing flash memory for data storage.

// Functions:
// move_to_angle(int angle) - Controls the motor to move to a specific angle.
// scan_distance(int x, int y, int receive_max) - Scans a range of angles and measures the distance at each point using the ultrasonic sensor.
// telemeter() - Continuously measures and displays the distance from a single point received from a PC.
// scan_light(int x, int y) - Scans a range of angles and measures the light intensity at each point using the LDRs.
// Light_initializer() - A calibration function for the LDR sensors, storing calibration data in flash memory.
// send_calibration() - Reads stored calibration data from flash memory and sends it to the PC.
// receive_angle() - Receives a single angle value from the PC via UART.
// receive_max_distance() - Receives a maximum distance value from the PC via UART.
// scan_objects_and_lights_bonus(int x, int y) - Scans for both objects (distance) and light sources simultaneously.
// write_to_segB(unsigned char *array) - Writes a data array to flash memory Segment B.
// flash_read_samples(unsigned char *array) - Reads a data array from flash memory Segment B.
// show_files() - Displays a list of saved files from flash memory on the LCD.
// Read_text(int file_number) - Reads a specified text file from flash memory and displays it on the LCD.
// Activate_Script(int file_number) - Reads and executes a script file from flash memory.
// Execute_func(int opc, int secondbyte, int thirdbyte) - A utility function to execute a specific script command based on an opcode.
// inc_lcd(int x) - Increments a counter on the LCD from 0 to x.
// dec_lcd(int x) - Decrements a counter on the LCD from x to 0.
// rra_lcd(int x) - Performs a right-to-left scrolling animation of a character on the LCD.
// set_delay(int delay) - Sets the delay time for the script functions.
// clear_lcd() - Clears the LCD display.
// servo_deg(int p) - Moves the servo motor to a specific angle p.
// servo_scan(int l, int r) - Scans the servo motor between two angles.


// --- halGPIO.c ---
// Purpose: This is the Hardware Abstraction Layer (HAL). It contains low-level functions for configuring and controlling the microcontroller's peripherals, including GPIO, timers, ADC, and UART. It also includes functions for managing flash memory and handling all system interrupts.

// Functions:
// sysConfig() - Initializes the system by calling configuration functions for GPIO, timers, and the LCD.
// StopAllTimers() - Halts all active timers (TA0 and TA1).
// TIMER_A1config_for_motor() - Configures Timer A1 for Pulse Width Modulation (PWM) to control the servo motor.
// update_angle(int angle) - Adjusts the PWM duty cycle to move the servo motor to a new angle.
// pause_motor() - Disables the motor's PWM output to stop its movement.
// continue_motor() - Re-enables the motor's PWM output to resume movement.
// first_last_angle() - Sets the motor's counter limit for moving to the first or last scan angle.
// consecutive_angle() - Sets the motor's counter limit for moving between consecutive scan angles.
// measure_light() - Configures the ADC and takes a light intensity reading from the LDRs.
// measure_light_for_bonus() - A dedicated function for measuring light intensity during the bonus state.
// config_trigger_out() - Configures a timer to generate a trigger pulse for the ultrasonic sensor.
// config_echo_input_capture() - Configures a timer to capture the incoming echo pulse from the ultrasonic sensor.
// send_angle_and_distance(int angle, int distance) - Adds angle and distance data to the transmit buffer for UART communication.
// send_angle_and_ldr(int angle, int distance) - Adds angle and light data to the transmit buffer.
// send_angle_and_data_for_bonus(int angle, int distance, int adc) - Adds angle, distance, and ADC data to the transmit buffer.
// send_angle_and_ldr_for_bonus(int angle, int adc) - Adds angle and ADC data with a specific header for light points.
// send_final_char() - Sends a termination character to signal the end of a data transmission.
// send_ack(int ack) - Sends an acknowledgment byte via UART.
// timer_config_for_script() - Configures a timer for script-based delays.
// timer_stop_for_script() - Stops the script timer.
// enable_interrupts() - Globally enables all interrupts.
// disable_interrupts() - Globally disables all interrupts.
// enable_PB0() - Enables the interrupt for Push Button 0.
// disable_PB0() - Disables the interrupt for Push Button 0.
// PBs_handler() - The Interrupt Service Routine (ISR) for the push buttons.
// TimerA0_ISR() - The ISR for Timer A0, used to manage script delays.
// Timer1_A1_ISR() - The ISR for Timer A1, managing motor movement and ultrasonic echo capture.
// ADC10_ISR() - The ISR for the ADC, triggered upon a completed conversion.
// USCI0TX_ISR() - The ISR for UART transmission, sending buffered data.
// USCI0RX_ISR() - The ISR for UART reception, handling incoming commands and data.
// enterLPM(unsigned char LPM_level) - Puts the microcontroller into a specified Low-Power Mode.
// initial_flash_write(unsigned int curr_header) - Calculates and prepares a flash memory location for writing a new file.
// flash_write_word(unsigned int addr, unsigned int data) - Writes a 16-bit word to a specified flash memory address.
// flash_unlock() - Unlocks the flash memory for write/erase operations.
// flash_lock() - Locks the flash memory to prevent accidental changes.
// flash_get_pointers() - Returns a pointer to the start of the file pointers list in flash memory.
// flash_get_file_count() - Counts the number of files stored in flash memory.
// lcd_cmd(unsigned char c) - Sends a command byte to the LCD.
// lcd_data(unsigned char c) - Sends a data byte to the LCD.
// lcd_puts(const char * s) - Prints a string to the LCD.
// lcd_init() - Initializes the LCD module.
// lcd_strobe() - Generates a strobe pulse for the LCD's enable pin.
// DelayUs(unsigned int cnt) - A busy-wait delay function in microseconds.
// DelayMs(unsigned int cnt) - A busy-wait delay function in milliseconds.
// delay(unsigned int t) - A generic busy-wait delay function.
// int2str(char *str, unsigned int num) - Converts an integer to a string.


// --- bsp.c ---
// Purpose: This is the Board Support Package (BSP). It contains all the low-level, board-specific configuration functions for the hardware peripherals used in the project, such as GPIO, ADC, and UART. It provides an abstraction layer between the application code and the specific hardware.

// Functions:
// GPIOconfig() - Configures all the GPIO pins for their designated roles (e.g., inputs, outputs, peripheral functions).
// ADC_config() - Initializes the Analog-to-Digital Converter for analog input readings.
// UART_init() - Initializes the UART communication module for serial data transfer.
```