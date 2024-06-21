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

#define SERVO_COUNT 1

//in degrees
int16_t reg_position_mid=0;
int16_t reg_position_min=-65;
int16_t reg_position_max=65;

bool send_flag=0;
bool receive_flag=0;
uint8_t response_hitec[7];
uint8_t request_hitec[5];
uint8_t write_hitec[7];
uint8_t servo_id[SERVO_COUNT];
int16_t current_positions[SERVO_COUNT];
int16_t previous_positions[SERVO_COUNT];
int16_t target_positions[SERVO_COUNT];
unsigned long delay_after_request=100000;

const float max_delay=20000;
const float min_delay=0;
const float a_coeff=(max_delay-min_delay)/(100-0);

void call_servos(uint8_t id_servo);

int serial_port;

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}

struct termios tty;

void init_serial(){
    serial_port = open("/dev/ttyUSB0", O_RDWR);
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

bool read_bus(){
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
	if (checksum==response_hitec[6]){
		flag=1;
	}
	else{}
	return flag;
  // write mode checksum = (ID + Address + Length + Data Low + Data High) & 0xFF
}

uint8_t get_wm_checksum() {
  uint8_t checksum=0;
  checksum = write_hitec[1]+write_hitec[2]+write_hitec[3]+write_hitec[4]+write_hitec[5];
  return checksum & 0xFF;
  // write mode checksum = Check Sum = (ID + Address + Length +Data Low + Data High) & 0xFF
}

bool restart_serial(){
	tcflush(serial_port, TCIOFLUSH);
	close(serial_port);
    init_serial();
	usleep(2000000);
	return true;	
}

bool writing_failure(uint8_t bytes_number){
	bool status =restart_serial();
	bool decision;
	if (status){
		ssize_t bytes_written;
		if (bytes_number==5){
			bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));	
		}
		else if (bytes_number==7){
			bytes_written=write(serial_port, write_hitec, sizeof(write_hitec));	
		}
		else {
			printf("sending message damaged\n");//memory leak
		}
		if (bytes_written==bytes_number){
			printf("serial restarted and bytes writen\n");
			send_flag=1;
			decision=true;
		}
		else {
			printf("serial restarted and still could not write\n");
			decision=false;
		}
	}
	else {
		decision=false;
		printf("Could not restart serial\n");
	}
	return decision;
}

void call_servos(uint8_t id_servo){
//request response from all servos
  request_hitec[0]=0x96;//write header
  request_hitec[1]=id_servo;//broadcast id
  request_hitec[2]=0x0C;//REG_POSITION address
  request_hitec[3]=0x00;//Length
  request_hitec[4]=(request_hitec[1]+request_hitec[2]+request_hitec[3])& 0xFF;
  for (int i = 0; i < 5; i++) {
      printf("0x%02X ", request_hitec[i]);
  }
  printf("\n");
  ssize_t bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));
  if (bytes_written==5){
	printf("5 bytes written\n");
	send_flag=1;
  }  
  else {
	writing_failure(5);
  }
}

bool get_ids(){
	bool checksum_match=true;
	call_servos(0x00);
	if (send_flag==1){
		send_flag=0;
		for(int i=0; i<SERVO_COUNT;i++){
			read_bus();
			bool check=get_read_bus_checksum();
			if (!check){
				checksum_match=false;
			}
			servo_id[i]=response_hitec[1];
			printf("Servo found, id : ");
			printf("0x%02X ", servo_id[i]);
			printf("\n");
		}
	}
	else {
		printf("Problem when requesting\n");//program should stop and notify OBC
	}
	return checksum_match;
}

bool read_data(uint8_t servo_id, uint8_t register_address) {
	usleep(delay_after_request);
	tcflush(serial_port, TCIOFLUSH);
	usleep(delay_after_request);
	request_hitec[0]=0x96; //write header
	request_hitec[1]=servo_id;
	request_hitec[2]=register_address;
	request_hitec[3]=0x00; //always 0 when requesting
	request_hitec[4]=(request_hitec[1]+request_hitec[2]+request_hitec[3])& 0xFF;
	for (int i = 0; i < 5; i++) {
      printf("0x%02X ", request_hitec[i]);
  	}
  	printf("\n");
  	ssize_t bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));
	if (bytes_written==5){
		printf("5 bytes written\n");
	}  
  	else {
		printf("problem when writting\n");
		exit(1);
  	}
	usleep(delay_after_request);
	read_bus();
	bool check=get_read_bus_checksum();
	return 1;
}

bool write_data(uint8_t servo_id, uint8_t register_address, uint8_t data_high, uint8_t data_low){
	usleep(delay_after_request);
	tcflush(serial_port, TCIOFLUSH);
	usleep(delay_after_request);
	write_hitec[0]=0x96;
	write_hitec[1]=servo_id;
	write_hitec[2]=register_address;
	write_hitec[3]=0x02;//length of data always 2
	write_hitec[4]=data_low;
	write_hitec[5]=data_high;
	write_hitec[6]=get_wm_checksum();
	write_hitec[6]=get_wm_checksum();
	for (int i = 0; i < 7; i++) {
    	printf("0x%02X ", write_hitec[i]);
  	}
  	printf("\n");
	ssize_t bytes_written=write(serial_port, write_hitec, sizeof(write_hitec));
  	if (bytes_written==7){
		printf("7 bytes written\n");
 	} 
	else {
		printf("error when writing\n");
	}	
	return true;	
}

bool get_position(uint8_t id){
	bool position_acquired=true;
	tcflush(serial_port, TCIOFLUSH);
	call_servos(servo_id[id]);
	if (send_flag==1){
		usleep(delay_after_request);
		if(!read_bus()){
			bool checksum_match=get_read_bus_checksum();
			uint8_t checksum_fail_count=0;
			while (!checksum_match){
				printf("----------------\n");
				tcflush(serial_port, TCIOFLUSH);
				usleep(100000);
				call_servos(servo_id[id]);
				if (send_flag==1){
					read_bus();
					checksum_match=get_read_bus_checksum();
				}
				checksum_fail_count+=1;
				if (checksum_fail_count>10){
					printf("Checksum won't match\n");
					exit(1);//alert OBC instead of exit
				}
			}
			uint16_t raw_position=(response_hitec[5] << 8) | response_hitec[4];//shifts the high byte from 1 byte and bit-wise OR
			current_positions[id]=(int16_t)(raw_position-7816)/45.51;
  			//MD Series (360°): -90°=4096, 0°=8192, 90°=12288.
			printf("Servo Position is %i", current_positions[id]);
			printf("\n");
		}
		else{
			printf("position not acquired\n");
			position_acquired=false;
		}
	}
	else { 
		printf("Problem when requesting\n");
		//program should stop and notify OBC
	}
	tcflush(serial_port, TCIOFLUSH);
	return position_acquired;
}

bool servo_move(uint8_t servo, int16_t value){
	float slope=(float)(5600-400)/(reg_position_max-reg_position_min);
	float b_coeff=(float)5600-slope*reg_position_max;
	uint16_t raw_value=(uint16_t)value*slope+b_coeff;
	uint8_t data_low=raw_value;
	uint8_t data_high=raw_value >> 8;
	write_hitec[0]=0x96;//write header
	write_hitec[1]=servo_id[servo];//id of servo
	write_hitec[2]=0x1E; //address REG_POSITION_NEW
	write_hitec[3]=0x02;//length
	write_hitec[4]=data_low;
	write_hitec[5]=data_high;
	//write_hitec[4]=0xE0;
	//write_hitec[5]=0x15;
	write_hitec[6]=get_wm_checksum();
	for (int i = 0; i < 7; i++) {
    	printf("0x%02X ", write_hitec[i]);
  	}
  	printf("\n");
	ssize_t bytes_written=write(serial_port, write_hitec, sizeof(write_hitec));
  	if (bytes_written==7){
		printf("7 bytes written\n");
		send_flag=1;
 	} 
	else {
		writing_failure(7);
	}	
	return true;	
}

bool servo_move_speed(uint8_t servo, int16_t target, uint16_t speed){
	uint16_t delay=(uint16_t)a_coeff*speed;
	printf("delay is : %i",delay);
	printf("\n");
	if (target>current_positions[servo]){
		for(int8_t i=current_positions[servo];i<=target;i++){
			servo_move(servo,i);
			usleep(delay);
		}
	}
	else if (target<current_positions[servo]){
		for(int8_t i=current_positions[servo];i>=target;i--){
			servo_move(servo,i);
			usleep(delay);
		}	
	}
	else{}
	return true;
}

bool activate_no_block_com(){
	tty.c_cc[VTIME] = 10;    // return as soon as 100ms has passed
    tty.c_cc[VMIN] = 0;
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
	return 1;
}

bool get_servo_position(uint8_t id){
	bool pos = get_position(id);
	uint8_t no_answer_cnt=0;
	while (!pos){
		tcflush(serial_port, TCIOFLUSH);
		usleep(100000);
		pos = get_position(id);
		no_answer_cnt+=1;
		if (no_answer_cnt>10){
			printf("Servo doesn't answer\n");
			printf("end of experience\n");
			exit(1);//alert OBC instead of exit
		}
		delay_after_request+=50000;
	}
}

int main(){
    init_serial();
	usleep(2000000);  // 2 seconds delay to allow Arduino to reset
	tcflush(serial_port, TCIOFLUSH);
	bool checksum_match = get_ids();
	uint8_t checksum_fail_count=0;
	while (!checksum_match){
		printf("----------------\n");
		tcflush(serial_port, TCIOFLUSH);
		usleep(100000);
		checksum_match=get_ids();
		checksum_fail_count+=1;
		if (checksum_fail_count>10){
			printf("Checksum won't match\n");
			exit(1);//alert OBC instead of exit
		}
	}
	read_data(0x00,0x32);
	activate_no_block_com();
	
	//get_servo_position(0);
	//write_data(0x00,0x70,0xFF,0xFF);
	/*
	get_servo_position(1);
	//usleep(1000000);
	//servo_move_speed(1,78,20);
	*/
	//servo_move(0,-60);
	//servo_move_speed(0,0,50);
	//servo_id[0]=0x01;
	usleep(3000000);
	//read_data(0x00,0x32);
	//get_servo_position(0);
	tcflush(serial_port, TCIOFLUSH);
	close(serial_port);
    return 1;
}