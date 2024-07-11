byte header = 0x69;
byte id = 0x00;
byte address = 0x0C;
byte length = 0x02;
byte data_l = 0x05;
byte data_h = 0xFF;
byte checksum = 0x13;

byte header1 = 0x69;
byte id1 = 0x01;
byte address1 = 0x03;
byte length1 = 0x07;
byte data_l1 = 0xB5;
byte data_h1 = 0x1F;
byte checksum1 = 0x5A;

void setup() {
	Serial.begin(9600);	
}

void loop(){
	if (Serial.available()>=5){
		byte buffer[5];
		for (int i = 0; i < 5; i++) {
            buffer[i] = Serial.read();
        }
		Serial.end();
		Serial.begin(9600);
		delay(250);
		Serial.write(header);
		Serial.write(id);
		Serial.write(address);
		Serial.write(length);
		Serial.write(data_l);
		Serial.write(data_h);
		Serial.write(checksum);

		Serial.write(header1);
                Serial.write(id1);
                Serial.write(address1);
                Serial.write(length1);
                Serial.write(data_l1);
                Serial.write(data_h1);
                Serial.write(checksum1);
}
}
