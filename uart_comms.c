#include "uart_comms.h"

uint8_t pin_RS485_control=16;//corresponds to actual pin GPIO15
int serial_port;
uint8_t response_hitec[7];
char port_name[100]="/dev/ttyUSB0";

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}

struct termios tty;

void init_serial(){
    serial_port = open(port_name, O_RDWR);
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

    	tty.c_cc[VTIME] = 10;    // return as soon as 100ms has passed
    	tty.c_cc[VMIN] = 0;

    	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    	}
        wiringPiSetup();
        pinMode(pin_RS485_control, OUTPUT);

}

//return as soon 7 bytes is read is convenient when broadcasting or setup but is dangerous in flight 
//as it can block the execution of the program
bool activate_block_com(){
	tty.c_cc[VTIME] = 0;    //return as soon as 7 bytes are read
    tty.c_cc[VMIN] = 7;
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
	return 1;
}

bool restart_serial(){
	tcflush(serial_port, TCIOFLUSH);
	close(serial_port);
    init_serial();
	usleep(2000000);
	return true;	
}

bool read_bus(){
    digitalWrite(pin_RS485_control, LOW);
	bool no_response=false;
	uint8_t num_bytes = read(serial_port, &response_hitec, sizeof(response_hitec));
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

bool get_read_bus_checksum() {
	uint8_t checksum=0;
	bool flag=0;
	checksum = response_hitec[1]+response_hitec[2]+response_hitec[3]+response_hitec[4]+response_hitec[5];
	checksum = checksum & 0xFF;
	printf("checksum computed is : ");
	printf(" 0x%02X", (unsigned char)checksum);
	printf("\n");
	if (checksum==response_hitec[6] && response_hitec[0]!=0x00){
		flag=1;
	}
	else{}
	return flag;
}