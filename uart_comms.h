#ifndef uart_comms
#define uart_comms

// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <wiringPi.h>// used to control direction in smd uart/rs485 converter
//update the use of that library to the technologies used on astreos

extern char port_name[100];
extern int serial_port;
extern uint8_t pin_RS485_control;
extern uint8_t response_hitec[7];

void init_serial();
bool restart_serial();
bool read_bus();
bool get_read_bus_checksum();

#endif