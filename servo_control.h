#ifndef servo_control
#define servo_control

#include "uart_comms.h" //used to link some common data

#define SERVO_COUNT 4

//registers addresses
#define REG_POSITION 0x0C
#define REG_POSITION_NEW 0x1E
#define REG_ID 0x32
#define REG_FAILSAFE_SET 0x4C
#define REG_DEADBAND 0x4E
#define REG_VIBRATION_DEADBAND_MIN 0x66
#define REG_VIBRATION_DEADBAND_MAX 0x68
#define REG_VELOCITY_MAX 0x54
#define REG_SOFT_START_SPEED 0x60
#define REG_OVERLOAD_PROTECTION 0x9C
#define REG_POSITION_MAX 0xB0
#define REG_POSITION_MIN 0xB2
#define REG_POSITION_MID 0xC2
#define REG_POWER_CONFIG 0x46
#define REG_FACTORY_DEFAULT 0x6E
#define REG_CONFIG_SAVE 0x70

int16_t reg_position_mid=0;//in degress
int16_t reg_position_min=-65;
int16_t reg_position_max=65;

bool send_flag=0;
bool receive_flag=0;
uint8_t request_hitec[5];
uint8_t write_hitec[7];
uint8_t broadcast_id=0x00;
uint8_t servo_id[SERVO_COUNT]={0x04,0x01,0x02,0x03};
uint16_t b_coeff_encoders[SERVO_COUNT]={7987,7816,8192,8192};//all encoders don't return the same when at 0°, see protocol datasheet 
int16_t straight_JV_value[SERVO_COUNT]={20,6,0,0};//values at which the JV are straight
int16_t current_positions[SERVO_COUNT];
int16_t previous_positions[SERVO_COUNT];
int16_t target_positions[SERVO_COUNT];
unsigned long delay_after_request=100000;

//functions
uint8_t get_wm_checksum();
bool writing_failure(uint8_t bytes_number);
void call_servos(uint8_t id_servo, uint8_t register_address);
bool read_data(uint8_t id, uint8_t register_address);
bool get_position(uint8_t id);
bool write_data(uint8_t servo_id, uint8_t register_address, uint8_t data_high, uint8_t data_low);
bool servo_move(uint8_t servo, int16_t value);
int main();

#endif