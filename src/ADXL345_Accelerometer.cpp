
#include <ADXL345_Accelerometer.hpp>         
#include <Wire.h>
#include <SPI.h>

#define ADXL345_DEVICE (0x53)    // Device Address for ADXL345
#define ADXL345_TO_READ (6)      // Number of Bytes Read - Two Bytes Per Axis

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
  //ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
  //ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION


ADXL345::ADXL345() {
	status = ADXL345_OK;
	error_code = ADXL345_NO_ERROR;

	gains[0] = 0.00376390;		// Original gain 0.00376390
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265
	I2C = true;
}

ADXL345::ADXL345(int CS) {
	status = ADXL345_OK;
	error_code = ADXL345_NO_ERROR;

	gains[0] = 0.00376390;
	gains[1] = 0.00376009;
	gains[2] = 0.00349265;
	_CS = CS;
	I2C = false;
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	pinMode(_CS, OUTPUT);
	digitalWrite(_CS, HIGH);
}

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void ADXL345::init(){

  powerOn();                     // Power on the ADXL345

  setInterruptMapping(ADXL345_INT_OVERRUNY_BIT, ADXL345_INT1_PIN);     // Map Overrun interruppt to int pin 1
  setInterrupt(ADXL345_INT_OVERRUNY_BIT, 1);                           // Enable Overrun interrupt
  
  setInterruptMapping(ADXL345_INT_WATERMARK_BIT, ADXL345_INT1_PIN);    // Map Watermark interrupt to int pin 1
  setInterrupt(ADXL345_INT_WATERMARK_BIT, 0);                          // Disable Watermark interrupt (causes int pin to be constant)
  
  setRate(100);     // set sampe rate to 100 Hz
}


/****************** MAIN CODE ******************/
/* Accelerometer Readings */
void ADXL345::emptyFifo(){
  
  int x,y,z;   
  //Serial.println("Starting Fifo buffer read");
  for(int i =0; i<32; i++){                    // loop through fifo buffer and empty it
    readAccel(&x, &y, &z);                 // reads acceleration
    delayMicroseconds(5);                       // minimum time between last read and start of the next read is 5 us
    // Serial.print(x);
    // Serial.print(", ");
    // Serial.print(y);
    // Serial.print(", ");
    // Serial.println(z); 
  }
  Serial.println("Fifo buffer emptied");

}



void ADXL345::powerOn() {
	if(I2C) {
		Wire.begin();				// If in I2C Mode Only
	}
	//ADXL345 TURN ON
	writeTo(ADXL345_POWER_CTL, 0);	// Wakeup
	writeTo(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	writeTo(ADXL345_POWER_CTL, 8);	// Measure
}

void ADXL345::readAccel(int *x, int *y, int *z) {
	readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);	// Read Accel Data from ADXL345

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (int16_t)((((int)_buff[1]) << 8) | _buff[0]);
	*y = (int16_t)((((int)_buff[3]) << 8) | _buff[2]);
	*z = (int16_t)((((int)_buff[5]) << 8) | _buff[4]);
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
		readFrom(ADXL345_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & B11110000);
		writeTo(ADXL345_BW_RATE, _s);
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

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
void ADXL345::writeTo(byte address, byte val) {
	if(I2C) {
		writeToI2C(address, val);
	}
	else {
		writeToSPI(address, val);
	}
}

/************************ READING NUM BYTES *************************/
/*    Reads Num Bytes. Starts from Address Reg to _buff Array        */
void ADXL345::readFrom(byte address, int num, byte _buff[]) {
	if(I2C) {
		readFromI2C(address, num, _buff);	// If I2C Communication
	}
	else {
		readFromSPI(address, num, _buff);	// If SPI Communication
	}
}

/*************************** WRITE TO I2C ***************************/
/*      Start; Send Register Address; Send Value To Write; End      */
void ADXL345::writeToI2C(byte _address, byte _val) {
	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.write(_address);
	Wire.write(_val);
	Wire.endTransmission();
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
void ADXL345::readFromI2C(byte address, int num, byte _buff[]) {
	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.write(address);
	Wire.endTransmission();

//	Wire.beginTransmission(ADXL345_DEVICE);
// Wire.reqeustFrom contains the beginTransmission and endTransmission in it. 
	Wire.requestFrom(ADXL345_DEVICE, num);  // Request 6 Bytes

	int i = 0;
	while(Wire.available())
	{
		_buff[i] = Wire.read();				// Receive Byte
		i++;
	}
	if(i != num){
		status = ADXL345_ERROR;
		error_code = ADXL345_READ_ERROR;
	}
//	Wire.endTransmission();
}

/************************** WRITE FROM SPI **************************/
/*         Point to Destination; Write Value; Turn Off              */
void ADXL345::writeToSPI(byte __reg_address, byte __val) {
  digitalWrite(_CS, LOW);
  SPI.transfer(__reg_address);
  SPI.transfer(__val);
  digitalWrite(_CS, HIGH);
}

/*************************** READ FROM SPI **************************/
/*                                                                  */
void ADXL345::readFromSPI(byte __reg_address, int num, byte _buff[]) {
  // Read: Most Sig Bit of Reg Address Set
  char _address = 0x80 | __reg_address;
  // If Multi-Byte Read: Bit 6 Set
  if(num > 1) {
  	_address = _address | 0x40;
  }

  digitalWrite(_CS, LOW);
  SPI.transfer(_address);		// Transfer Starting Reg Address To Be Read
  for(int i=0; i<num; i++){
    _buff[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS, HIGH);
}

void ADXL345::setRegisterBit(byte regAdress, int bitPos, bool state) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
	}
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);
}