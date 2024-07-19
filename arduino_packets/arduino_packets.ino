byte header = 0x69;
byte id = 0x01;
byte address = 0x0C;
byte length = 0x02;
byte data_l = 0xD2;
byte data_h = 0x20;
byte checksum = 0x01;

byte header1 = 0x69;
byte id1 = 0x02;
byte address1 = 0x03;
byte length1 = 0x07;
byte data_l1 = 0x00;
byte data_h1 = 0x30;
byte checksum1 = 0x3C;

void setup() {
	Serial.begin(9600);	
}

void loop(){
	if (Serial.available()>=5){
		byte buffer[5];
		for (int i = 0; i < 5; i++) {
            buffer[i] = Serial.read();
        }
	if (buffer[1]==0x00){
		//Serial.end();
		//Serial.begin(9600);
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
	else if (buffer[1]==0x01){
		Serial.write(header);
		Serial.write(id);
		Serial.write(address);
		Serial.write(length);
		Serial.write(data_l);
		Serial.write(data_h);
		Serial.write(checksum);
	}
	else if(buffer[1]==0x02){
		Serial.write(header1);
        Serial.write(id1);
        Serial.write(address1);
        Serial.write(length1);
        Serial.write(data_l1);
        Serial.write(data_h1);
        Serial.write(checksum1);
	}
	else {}
	}
}
