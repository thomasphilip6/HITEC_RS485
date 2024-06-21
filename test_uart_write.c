// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <wiringPi.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int serial_port;
struct termios tty;

void init_serial(){
    serial_port = open("/dev/ttyAMA3", O_RDWR);
    if(tcgetattr(serial_port, &tty)!=0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    } 
    	cfsetispeed(&tty, B115200);
    	cfsetospeed(&tty, B115200);

    	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    	tty.c_cflag &= ~CSIZE; // Clear all the size bits
    	tty.c_cflag |= CS8; // 8 bits per byte (most common)
    	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    	tty.c_cflag |= CREAD | CLOCAL;
    //CLOCAL must be low and CREAD high as it allows to read data

    	tty.c_lflag &= ~ICANON;//disables canonical mode
    	tty.c_lflag &= ~ECHO; // Disable echo
    	tty.c_lflag &= ~ECHOE; // Disable erasure
    	tty.c_lflag &= ~ECHONL; // Disable new-line echo
    	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP


    	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    	tty.c_cc[VTIME] = 10;    // return as soon as 7 Byte is read or 100ms has passed
    	tty.c_cc[VMIN] = 7;

    	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    	}

}
uint8_t request_hitec[5]={0x0A,0x0B,0x12,0x13,0x14};

int main() {
	wiringPiSetup();
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);

    init_serial();
    usleep(2000000);
    while(1){
        for (int i = 0; i < 5; i++) {
        printf("0x%02X ", request_hitec[i]);
        }
        printf("\n");
        ssize_t bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));
    if (bytes_written==5){
	    printf("5 bytes written\n");
    }     
    usleep(1000000);
    }
    return 1;
}
