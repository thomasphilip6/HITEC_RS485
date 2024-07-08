#include <stdio.h>
#include <stdint.h>
//#include <unistd.h>

void connect_servos();
uint8_t get_rm_checksum(uint8_t id, uint8_t address,uint8_t length);
int main() {
  connect_servos();
  return 0;
} 
uint8_t get_rm_checksum(uint8_t id, uint8_t address,uint8_t length) {
  uint8_t checksum=0;
  checksum = id+address+length;
  return checksum & 0xFF;
  // read mode checksum = (ID + Address + Length) & 0xFF
}
uint8_t get_wm_checksum(uint8_t id, uint8_t address,uint8_t length,uint8_t dataL, uint8_t dataH) {
  uint8_t checksum=0;
  checksum = id+address+length+dataL+dataH;
  return checksum & 0xFF;
  // write mode checksum = Check Sum = (ID + Address + Length +Data Low + Data High) & 0xFF
}
uint8_t read_id(uint8_t response[7]){
  uint8_t servo_id=0;
  if (response[0]==0x69){
    printf("response hearder valid");
    printf("\n");
    uint8_t checksum=0;
    checksum=get_wm_checksum(response[1],response[2],response[3],response[4],response[5]);
    if (checksum==response[6]){
      printf("response checksum valid");
      printf("\n");
      servo_id=response[1];
    }
    else {
      printf("response checksum invalid : ");
      printf("0x%02X ", response[6]);
      printf("0x%02X ", checksum);
      servo_id=42;
    }
  }
  else {
    printf("unknown response header : ");
    printf("0x%02X ", response[0]);
    printf("\n");
    servo_id=42;
  }
  return servo_id;
}

float read_pos(uint8_t response[7]){
  uint16_t raw_position=(response[5] << 8) | response[4];//shifts the high byte from 1 byte and bit-wise OR
  float servo_pos=(raw_position-8192)/74.48;
  //MD Series (360°): -90°=4096, 0°=8192, 90°=12288.
  return servo_pos;
}

void connect_servos(){
//request response from all servos
  uint8_t packet[5];
  packet[0]=0x96;//write header
  packet[1]=0x00;//broadcast id
  packet[2]=0x0C;//REG_POSITION address
  packet[3]=0x00;//Length
  packet[4]=get_rm_checksum(packet[1],packet[2],packet[3]);

  // Print the packet in hexadecimal format
  for (int i = 0; i < 5; i++) {
      printf("0x%02X ", packet[i]);
  }
  printf("\n");
}


void send_data_uart(){

}

void send_data_usb(){

}