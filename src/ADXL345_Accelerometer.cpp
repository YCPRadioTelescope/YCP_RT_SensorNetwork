
#include <ADXL345_Accelerometer.hpp>         
#include <Wire.h>
#include <SPI.h>

extern "C"{		// this is a fix for undefined references in the linker due to queue initialization
	int __exidx_start(){ return -1;}
  	int __exidx_end(){ return -1; }
}

#define ADXL345_DEVICE (0x53)    // Device Address for ADXL345
#define ADXL345_TO_READ (6)      // Number of Bytes Read - Two Bytes Per Axis
#define ADXL345_Sample_Num (32)  // Max number of samples ADXL holds

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
  //ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
  //ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION


ADXL345::ADXL345(TwoWire& wire) : accelwire(wire){
	status = ADXL345_OK;
	error_code = ADXL345_NO_ERROR;
	
	gains[0] = 0.00376390;		// Original gain 0.00376390
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265
	I2C = true;
	
}

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void ADXL345::init(){

  powerOn();                     // Power on the ADXL345
  
  setInterruptMapping(ADXL345_INT_WATERMARK_BIT, ADXL345_INT1_PIN);    // Map Watermark interrupt to int pin 1
  setInterrupt(ADXL345_INT_WATERMARK_BIT, 1);                          // Enable Watermark interrupt 
  
  writeToI2C(ADXL345_FIFO_CTL,0b01111111);								// (10|FIFO mode) (1|triger to INT1) (11111|trigger at 32 samples)
  setRate(100);     // set sampe rate to 800 Hz
  
}


/****************** MAIN CODE ******************/
/* Accelerometer Readings */
void ADXL345::emptyFifo(){
  //unsigned long start = 0; // the time the delay started
  //start = millis();
  int x,y,z;   
  //Serial.println("starting");
  // loop through fifo buffer and empty it
  for(int i =0; i < 32; i++){  
	   
    accelwire.beginTransmission(ADXL345_DEVICE);
    accelwire.write(ADXL345_DATAX0);
    accelwire.endTransmission(false);
    accelwire.requestFrom(ADXL345_DEVICE, ADXL345_TO_READ, true);  // Request 6 Bytes TODO:192

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	x = (short)((accelwire.read() | accelwire.read() << 8));
	y = (short)((accelwire.read() | accelwire.read() << 8));
	z = (short)((accelwire.read() | accelwire.read() << 8));

	buffer.push({x,y,z});
    //delayMicroseconds(5);                       // minimum time between last read and start of the next read is 5 us
     //Serial.print(x);
     //Serial.print(", ");
     //Serial.print(y);
     //Serial.print(", ");
     //Serial.println(z); 
  }
  //Serial.print("Finished in ");
  //Serial.println(millis() - start);
  //Serial.println(accelwire.);
  //Serial.println("Fifo buffer emptied");

}

void ADXL345::powerCycle(uint8_t pinNumber) {
	digitalWrite(pinNumber, LOW);
    delay(10);
	digitalWrite(pinNumber, HIGH);
	delay(10);
}

void ADXL345::powerOn() {
	if(I2C) {
		accelwire.begin();				// If in I2C Mode Only
		
	}
	//ADXL345 TURN ON
	writeToI2C(ADXL345_POWER_CTL, 0);	// Wakeup
	writeToI2C(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	writeToI2C(ADXL345_POWER_CTL, 8);	// Measure
}

void ADXL345::readAccel(int *x, int *y, int *z) {
	readFromI2C(ADXL345_DATAX0, ADXL345_TO_READ * ADXL345_Sample_Num, _buff);	// Read Accel Data from ADXL345

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (short)((_buff[1] << 8) | _buff[0]);
	*y = (short)((_buff[3] << 8) | _buff[2]);
	*z = (short)((_buff[5] << 8) | _buff[4]);
}

//Set sample rate in Hz
void ADXL345::setRate(double rate){
	byte _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) {
		readFromI2C(ADXL345_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & B11110000);
		writeToI2C(ADXL345_BW_RATE, _s);
	}
}

/*********************** INTERRUPT MAPPING **************************/
/*         Set the Mapping of an Interrupt to pin1 or pin2          */
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345::setInterruptMapping(byte interruptBit, bool interruptPin) {
	setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

void ADXL345::setInterrupt(byte interruptBit, bool state) {
	setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

/*************************** WRITE TO I2C ***************************/
/*      Start; Send Register Address; Send Value To Write; End      */
void ADXL345::writeToI2C(byte _address, byte _val) {

	accelwire.beginTransmission(ADXL345_DEVICE);
	accelwire.write(_address);
	accelwire.write(_val);
	accelwire.endTransmission();
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
void ADXL345::readFromI2C(byte address, int num, byte _buff[]) {

	accelwire.beginTransmission(ADXL345_DEVICE);
	accelwire.write(address);
	accelwire.endTransmission();
	accelwire.requestFrom(ADXL345_DEVICE, num);  // Request 6 Bytes TODO:192
	int i = 0;
	while(accelwire.available())
	{
		_buff[i] = accelwire.read();				// Receive Byte
		i++;
	}
	if(i != num){
		status = ADXL345_ERROR;
		error_code = ADXL345_READ_ERROR;
	}
	
}

void ADXL345::setRegisterBit(byte regAdress, int bitPos, bool state) {
	byte _b;
	readFromI2C(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
	}
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeToI2C(regAdress, _b);
}