#include "uart_comms.h" //used to link some common data

#define SERVO_COUNT 4

//in degrees
int16_t reg_position_mid=0;
int16_t reg_position_min=-65;
int16_t reg_position_max=65;

bool send_flag=0;
bool receive_flag=0;
uint8_t request_hitec[5];
uint8_t write_hitec[7];
uint8_t servo_id[SERVO_COUNT]={0x04,0x01,0x02,0x03};
uint16_t b_coeff_encoders[SERVO_COUNT]={7987,7816,8192,8192};//all encoders don't return the same when at 0°, see protocol datasheet 
int16_t straight_JV_value[SERVO_COUNT]={20,6,0,0};//values at which the JV are straight
int16_t current_positions[SERVO_COUNT];
int16_t previous_positions[SERVO_COUNT];
int16_t target_positions[SERVO_COUNT];
unsigned long delay_after_request=100000;

const float max_delay=20000;
const float min_delay=0;
const float a_coeff=(max_delay-min_delay)/(100-0);

uint8_t get_wm_checksum() {
  uint8_t checksum=0;
  checksum = write_hitec[1]+write_hitec[2]+write_hitec[3]+write_hitec[4]+write_hitec[5];
  return checksum & 0xFF;
  // write mode checksum = Check Sum = (ID + Address + Length +Data Low + Data High) & 0xFF
}


//in case of writing failure this functions restarts it and tries send the message that failed
bool writing_failure(uint8_t bytes_number){
	bool status =restart_serial();
	bool decision;
	if (status){
		ssize_t bytes_written;
		if (bytes_number==5){
            digitalWrite(pin_RS485_control, HIGH);
			bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));	
            digitalWrite(pin_RS485_control, LOW);
		}
		else if (bytes_number==7){
            digitalWrite(pin_RS485_control, HIGH);
			bytes_written=write(serial_port, write_hitec, sizeof(write_hitec));	
            digitalWrite(pin_RS485_control, LOW);
		}
		else {
			printf("sending message damaged\n");//memory leak
		}
		if (bytes_written==bytes_number){
			printf("serial restarted and bytes writen\n");
			send_flag=1;
			decision=true;
            //all bytes were send
		}
		else {
			printf("serial restarted and still could not write\n");
			decision=false;
            //all bytes could not be sent
		}
	}
	else {
		decision=false;
		printf("Could not restart serial\n");
	}
	return decision;
}

//this functions asks for a given servo to return its position
void call_servos(uint8_t id_servo){
    request_hitec[0]=0x96;//write header
    request_hitec[1]=id_servo;//broadcast id
    request_hitec[2]=0x0C;//REG_POSITION address
    request_hitec[3]=0x00;//Length
    request_hitec[4]=(request_hitec[1]+request_hitec[2]+request_hitec[3])& 0xFF;
    for (int i = 0; i < 5; i++) {
      printf("0x%02X ", request_hitec[i]);
    }
    printf("\n");
    digitalWrite(pin_RS485_control, HIGH);
    ssize_t bytes_written=write(serial_port, request_hitec, sizeof(request_hitec));
    digitalWrite(pin_RS485_control, LOW);
    if (bytes_written==5){
	    printf("5 bytes written\n");
	send_flag=1;
    }  
    else {
	    writing_failure(5);
    }
}

//returns the position of a given servo
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
			current_positions[id]=(int16_t)(raw_position-b_coeff_encoders[id])/45.51;
  			//MD Series (360°): -90°=4096, 0°=depends on servo, 90°=12288.
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
	write_hitec[6]=get_wm_checksum();
	for (int i = 0; i < 7; i++) {
    	printf("0x%02X ", write_hitec[i]);
  	}
  	printf("\n");
    digitalWrite(pin_RS485_control, HIGH);
	ssize_t bytes_written=write(serial_port, write_hitec, sizeof(write_hitec));
    digitalWrite(pin_RS485_control, HIGH);
  	if (bytes_written==7){
		printf("7 bytes written\n");
		send_flag=1;
 	} 
	else {
		writing_failure(7);
	}	
	return true;	
}

int main(){
    init_serial();
    usleep(2000000);
    tcflush(serial_port, TCIOFLUSH);
    usleep(2000000);
    get_position(0);
    usleep(2000000);
    get_position(1);
    usleep(2000000);
    get_position(2);
    usleep(2000000);
    get_position(3);
    usleep(2000000);
   return 1;
}

