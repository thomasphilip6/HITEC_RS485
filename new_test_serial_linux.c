// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}
int serial_port;
void init_serial(){
    serial_port = open("/dev/ttyACM0", O_RDWR);
    struct termios tty;
    if(tcgetattr(serial_port, &tty)!=0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    } 
    	cfsetispeed(&tty, B9600);
    	cfsetospeed(&tty, B9600);

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

    	tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    	tty.c_cc[VMIN] = 2;

    	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    	}

}

int main(){
    init_serial();
    uint8_t read_buf[1];
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
    printf("Read %i bytes.", num_bytes);
	for (int i = 0; i < num_bytes; ++i) {
    printf(" 0x%02X", (unsigned char)read_buf[i]);
    }
    printf("\n");
	close(serial_port);
    return 1;
}