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

#define SERVO_COUNT 4

bool send_flag=0;
bool receive_flag=0;
uint8_t response_hitec[7];
uint8_t request_hitec[5];
uint8_t write_hitec[7];
uint8_t servo_id[SERVO_COUNT];
float current_positions[SERVO_COUNT];
float previous_positions[SERVO_COUNT];
float target_positions[SERVO_COUNT];

int serial_port;

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}

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

    	tty.c_cc[VTIME] = 0;    // return as soon as 1 Byte is read
    	tty.c_cc[VMIN] = 7;

    	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    	}

}

bool read_bus(){
	int num_bytes = read(serial_port, &response_hitec, sizeof(response_hitec));
	printf("Read %i bytes.", num_bytes);
	for (int i = 0; i < num_bytes; ++i) {
    printf(" 0x%02X", (unsigned char)response_hitec[i]);
    }
    printf("\n");
	return 1;
}

bool get_rm_checksum() {
	uint8_t checksum=0;
	bool flag=0;
	checksum = response_hitec[1]+response_hitec[2]+response_hitec[3];
	checksum = checksum & 0xFF;
	printf("checksum computed is : ");
	printf(" 0x%02X", (unsigned char)checksum);
	printf("\n");
	if (checksum==response_hitec[6]){
		flag=1;
	}
	else{}
	return flag;
  // read mode checksum = (ID + Address + Length) & 0xFF
}

uint8_t get_wm_checksum() {
  uint8_t checksum=0;
  checksum = write_hitec[1]+write_hitec[2]+write_hitec[3]+write_hitec[4]+write_hitec[5];
  return checksum & 0xFF;
  // write mode checksum = Check Sum = (ID + Address + Length +Data Low + Data High) & 0xFF
}

void call_servos(uint8_t id_servo){
//request response from all servos
  request_hitec[0]=0x96;//write header
  request_hitec[1]=id_servo;//broadcast id
  request_hitec[2]=0x0C;//REG_POSITION address
  request_hitec[3]=0x00;//Length
  request_hitec[4]=request_hitec[1]+request_hitec[2]+request_hitec[3];
  request_hitec[4]=request_hitec[4] & 0xFF;
  for (int i = 0; i < 5; i++) {
      printf("0x%02X ", request_hitec[i]);
  }
  printf("\n");
  ssize_t bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));
  if (bytes_written==5){
	printf("5 bytes written\n");
	send_flag=1;
  }  
}

void get_ids(){
	call_servos(0x00);
	if (send_flag==1){
		send_flag=0;
		for(int i=0; i<SERVO_COUNT;i++){
			read_bus();
			servo_id[i]=response_hitec[1];
			printf("Servo found, id : ");
			printf("0x%02X ", servo_id[i]);
			printf("\n");
		}
	}
	else {
		printf("Problem when requesting\n");
	}
}

void get_position(uint8_t id){
	call_servos(servo_id[id]);
	if (send_flag==1){
		uint16_t raw_position=(response_hitec[5] << 8) | response_hitec[4];//shifts the high byte from 1 byte and bit-wise OR
		current_positions[id]=(raw_position-8192)/74.48;
  		//MD Series (360°): -90°=4096, 0°=8192, 90°=12288.
	}
	else {
		printf("Problem when requesting\n");
	}
}

int main(){
    init_serial();
	usleep(2000000);  // 2 seconds delay to allow Arduino to reset
	call_servos(0x00);
	read_bus();
	bool checksum=get_rm_checksum();
	read_bus();
	checksum=get_rm_checksum();
	close(serial_port);
    return 1;
}