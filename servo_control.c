#include "servo_control.h"

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
void call_servos(uint8_t id_servo, uint8_t register_address){
    request_hitec[0]=0x96;//write header
    request_hitec[1]=id_servo;
    request_hitec[2]=register_address;
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
	call_servos(servo_id[id],REG_POSITION);
	if (send_flag==1){
		usleep(delay_after_request);
		if(!read_bus()){
			bool checksum_match=get_read_bus_checksum();
			uint8_t checksum_fail_count=0;
			while (!checksum_match){
				printf("----------------\n");
				tcflush(serial_port, TCIOFLUSH);
				usleep(100000);
				call_servos(servo_id[id],REG_POSITION);
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

bool write_data(uint8_t servo_id, uint8_t register_address, uint8_t data_high, uint8_t data_low){
	write_hitec[0]=0x96;//write header
	write_hitec[1]=servo_id;
	write_hitec[2]=register_address;
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

bool servo_move(uint8_t servo, int16_t value){
	float slope=(float)(5600-400)/(reg_position_max-reg_position_min);
	float b_coeff=(float)5600-slope*reg_position_max;
	uint16_t raw_value=(uint16_t)value*slope+b_coeff;
	uint8_t data_low=raw_value;
	uint8_t data_high=raw_value >> 8;
	write_data(servo_id[servo],REG_POSITION_NEW,data_high,data_low);
	return true;	
}

int main(){
    init_serial();
    usleep(2000000);
    tcflush(serial_port, TCIOFLUSH);
    usleep(2000000);
	servo_move(0,20);
	usleep(200000);
    get_position(0);
    usleep(200000);
    get_position(1);
    usleep(200000);
    get_position(2);
    usleep(200000);
    get_position(3);
    usleep(200000);
   return 1;
}
