#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <stdbool.h>
#include <stdint.h>

#define BUFFER_SIZE 1024
int serial_fd;
uint8_t response_hitec[5];

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}

bool read_bus(){
	bool no_response=false;
	uint8_t num_bytes = read(serial_fd, &response_hitec, sizeof(response_hitec));
	printf("Read %i bytes.", num_bytes);
	for (int i = 0; i < num_bytes; ++i) {
    printf(" 0x%02X", (unsigned char)response_hitec[i]);
    }
    printf("\n");
	if (num_bytes==0){
		no_response=true;
		}
	return no_response;
}
struct termios tty;

int main() {
    char buffer[BUFFER_SIZE];
    ssize_t bytesRead;

    serial_fd = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        error_exit("Error opening serial port");
    }
    if(tcgetattr(serial_fd, &tty)!=0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    } 
/*
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B115200);  // Set baud rate to 9600
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Mask data size bits
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cc[VMIN] = 5;      // Minimum number of characters to read
    options.c_cc[VTIME] = 0;     // Timeout in deciseconds for read (0 for non-blocking)

    tcsetattr(serial_fd, TCSANOW, &options); 
*/
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

    	if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
    	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    	}
    while (1) {
        /*
        bytesRead = read(serial_fd, buffer, sizeof(buffer)-1);
        if (bytesRead == -1) {
            error_exit("Error reading from serial port");
        } else if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; 
            printf("Received: %s", buffer);
        }
    }
    */
        bool answer=read_bus();
        usleep(1000000);
    }
    close(serial_fd);

    return 0;
}
