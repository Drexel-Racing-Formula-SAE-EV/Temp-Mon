// Temperature Monitor Code for Teensy 3.5
// Written by Jason Laug for Drexel Formula SAE EV 2020

// Can module 1 definitions

#include <ADC.h>
#include <ADC_util.h>
#include "FlexCAN.h"

#define CAN1_RX 30
#define CAN1_TX 29
#define CAN1_STBY 28

// Can module 2 definitions
#define CAN2_TX 33
#define CAN2_RX 34
#define CAN2_STBY 35

// Temperature warning output definitons
#define TEMP_HIGH 38
#define TEMP_WARN 39

// Multiplexer output definitons
#define OUT_1 14
#define OUT_2 15
#define OUT_3 16

// Multiplexer input definitions
#define S0 3
#define S1 2
#define S2 1
#define S3 0

#define CAN_BASE_ID 0x1839F379
#define MODULE_ID = 1

ADC *adc = new ADC(); //adc object

// Arrays that contain data from the multiplexer / temperature monitors
int analogValues[48]; // Arduino analogRead values
float voltValues[48]; // Mapped voltage values
float tempValues[48]; // Mapped temperature values

int num_tmp_sensors;
int lowest_temp_id;
int highest_temp_id;
int averageTemp;

FlexCAN bus(500000, 0, 1, 1);

void setup() { 
	// Pinmode for multiplexer signal pins
	pinMode(S0, OUTPUT); 
	pinMode(S1, OUTPUT); 
	pinMode(S2, OUTPUT); 
	pinMode(S3, OUTPUT); 

	// Pinmode for reading data from multiplexer
	pinMode(OUT_1, INPUT);
	pinMode(OUT_2, INPUT);
	pinMode(OUT_3, INPUT);

	pinMode(CAN1_STBY, OUTPUT);
	digitalWrite(CAN1_STBY, LOW);

	Serial.begin(9600);

	adc->setAveraging(16);
	adc->setResolution(16);
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
}

void loop() {
	for(int i = 0; i < 15; i++) {
		writeValues(i);
		delayMicroseconds(1000);

		Serial.println(voltValues[0], 8);
		Serial.println(analogValues[0], 8);
	}

	analyzeTemps();
	Serial.println(sendData());
}

void writeValues(int current_bit){
	// Read values from the three multiplexers into respective cell indices in analogValues

	bitset<4> bits(current_bit);
	digitalWrite(S0, bits[0]);
	digitalWrite(S1, bits[1]);
	digitalWrite(S2, bits[2]);
	digitalWrite(S3, bits[3]);
	
	analogValues[current_bit] = adc->analogRead(OUT_1);
	analogValues[current_bit+16] = adc->analogRead(OUT_2);
	analogValues[current_bit+32] = adc->analogRead(OUT_3);
	 
	// Map values from analogValues into voltage in voltValues
	float mapValue = 0.00122070312;
	voltValues[current_bit] = analogValues[current_bit]*mapValue;
	voltValues[current_bit+16] = analogValues[current_bit+16]*mapValue;
	voltValues[current_bit+32] = analogValues[current_bit+32]*mapValue;

	// Map values from volt values into temp values non linearly using linear interpolation
	tempValues[current_bit] = mapTemp(voltValues[current_bit]);
	tempValues[current_bit+16] = mapTemp(voltValues[current_bit+16]);
	tempValues[current_bit+32] = mapTemp(voltValues[current_bit+32]);
}

float mapTemp(float volt){
	// Converts voltage to temperature using values from datasheet table and linear interpolation
	float temp;
	float temperatures[] = {-40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120};
	float voltages[] = {2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30};	

	// Loop for finding the voltage that is one below the current read value
	int i = 0;
	while(volt<voltages[i]){
		i += 1;
	}

	//voltages[i] is the value below the read value
	//voltages[i+1] is the voltage above the read value
	//temperatures[i] is the temp below the actual value
	//temperatures[i+1] is the temp above the actual value
	
	float rate = 5.00/(voltages[i-1]-voltages[i]);
	temp = rate * (volt-voltages[i]) + temperatures[i];
	return temp;
}

void analyzeTemps() {
	int sum = tempValues[0];

	num_tmp_sensors = 48;
	lowest_temp_id = 0;
	highest_temp_id = 0;
	
	for(int i = 1; i < num_tmp_sensors; i++) {
		if(tempValues[i] < tempValues[lowest_temp_id])
			lowest_temp_id = i;
		else if(tempValues[i] < tempValues[lowest_temp_id])
			highest_temp_id = i;
		
		sum += tempValues[i];
	}

	averageTemp = sum / num_tmp_sensors;
}

// returns true if successful
bool sendData() {
	CAN_message_t message;
	message.len = 8;
	message.id = CAN_BASE_ID + MODULE_ID;
	
	message.buf[0] = MODULE_ID;
	message.buf[1] = tempValues[lowest_temp_id];
	message.buf[2] = tempValues[highest_temp_id];
	message.buf[3] = averageTemp;
	message.buf[4] = num_tmp_sensors;
	message.buf[5] = highest_temp_id;
	message.buf[6] = lowest_temp_id;
	message.buf[7] = 0; // check sum
	
	for(int i = 0; i < 7; i++)
		message.buf[7] += message.buf[i];
	
	message.buf[7] += message.len + message.id;

	return bus.write(message);
}