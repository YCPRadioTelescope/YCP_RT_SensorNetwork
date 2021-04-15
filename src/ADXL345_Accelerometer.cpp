
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

bool ADXL345::selfTest(){

	uint8_t fifolength = 0;
	int x,y,z;

	int Xst_off = 0, Yst_off = 0, Zst_off = 0;
 	writeToI2C(ADXL345_BW_RATE, ADXL345_NORMAL_POWER_MODE << 4 | ADXL345_BW_400); // (000| filler bits), (0| normal power mode), (1101| 800Hz sampling)
	writeToI2C(ADXL345_DATA_FORMAT, (uint8_t)ADXL345_DISABLE_SELF_TEST <<  7 | ADXL345_4_WIRE_SPI_MODE << 6 | ADXL345_INTERUPT_HIGH << 5 | ADXL345_FULL_RESOLUTION_MODE << 3 | ADXL345_RIGHT_JUSTIFY << 2 | ADXL345_RANGE_16_G); // (0| disable self test), (0| 4-wire SPI mode), (0| interrupts active high), (0| fill bit), (1| full-resolution), (0| right-justified), (11| 16 g mode)
	clearAccel();	// clear adxl buffer so no old data persists
	delay(50); //collect data
	readFromI2C(ADXL345_FIFO_STATUS,1, &fifolength);
	fifolength = fifolength & (uint8_t)0b00111111;		//get the number of data values stored in FIFO
	for(int i =0; i < fifolength; i++){  
		
		readAccel(&x,&y,&z);
		Xst_off += x;
		Yst_off += y;
		Zst_off += z;

	}
	Xst_off = Xst_off / fifolength;
	Yst_off = Yst_off / fifolength;
	Zst_off = Zst_off / fifolength;
	//Serial.print("Xst_off - ");
	//Serial.println(Xst_off);
	//Serial.print("Yst_off - ");
	//Serial.println(Yst_off);
	//Serial.print("Zst_off - ");
	//Serial.println(Zst_off);
	
	int Xst_on = 0, Yst_on = 0, Zst_on = 0;
	// Turn on self test
	writeToI2C(ADXL345_DATA_FORMAT, ADXL345_ENABLE_SELF_TEST <<  7 | ADXL345_4_WIRE_SPI_MODE << 6 | ADXL345_INTERUPT_HIGH << 5 | ADXL345_FULL_RESOLUTION_MODE << 3 | ADXL345_RIGHT_JUSTIFY << 2 | ADXL345_RANGE_16_G); // (1| enable self test), (0| 4-wire SPI mode), (0| interrupts active high), (0| fill bit), (1| full-resolution), (0| right-justified), (11| 16 g mode)
	delay(10); // output needs some time to settle 
	clearAccel();

	delay(50); //collect data
	readFromI2C(ADXL345_FIFO_STATUS,1, &fifolength);
	fifolength = fifolength & (uint8_t)0b00111111;		//get the number of data values stored in FIFO
	for(int i =0; i < fifolength; i++){  
		
		readAccel(&x,&y,&z);
		Xst_on += x;
		Yst_on += y;
		Zst_on += z;

	}
	Xst_on = Xst_on / fifolength;
	Yst_on = Yst_on / fifolength;
	Zst_on = Zst_on / fifolength;
	//Serial.print("Xst_on - ");
	//Serial.println(Xst_on);
	//Serial.print("Yst_on - ");
	//Serial.println(Yst_on);
	//Serial.print("Zst_on - ");
	//Serial.println(Zst_on);

	// Turn off self test
	writeToI2C(ADXL345_DATA_FORMAT, (uint8_t)ADXL345_DISABLE_SELF_TEST <<  7 | ADXL345_4_WIRE_SPI_MODE << 6 | ADXL345_INTERUPT_HIGH << 5 | ADXL345_FULL_RESOLUTION_MODE << 3 | ADXL345_RIGHT_JUSTIFY << 2 | ADXL345_RANGE_16_G); // (0| enable self test), (0| 4-wire SPI mode), (0| interrupts active high), (0| fill bit), (1| full-resolution), (0| right-justified), (11| 16 g mode)
	
	delay(10); // output needs some time to settle 
	clearAccel();	// clear adxl buffer so no old data persists

	// 3.9mg/LSB scale factor
	double scalefactor = 3.9;

	int Xst = (Xst_on - Xst_off)/scalefactor;
	int Yst = (Yst_on - Yst_off)/scalefactor;
	int Zst = (Zst_on - Zst_off)/scalefactor;

	//Serial.print("Xst - ");
	//Serial.println(Xst);
	//Serial.print("Yst - ");
	//Serial.println(Yst);
	//Serial.print("Zst - ");
	//Serial.println(Zst);
	// Scale Factors for 3.3 volts
	double XYscalefactor = 1.77;
	double Zscalefactor = 1.47;
	// Mins and maxs for 16g, 10-Bit Resolution Self-Test Output
	double Xmin = 6 * XYscalefactor;
	double Xmax = 67 * XYscalefactor;
	double Ymin = -67 * XYscalefactor;
	double Ymax = -6 * XYscalefactor;
	double Zmin = 10 * Zscalefactor;
	double Zmax = 110 * Zscalefactor;

	bool Xgood = (Xst > Xmin && Xst < Xmax);
	bool Ygood = (Yst > Ymin && Yst < Ymax);
	bool Zgood = (Zst > Zmin && Zst < Zmax);
	if(!Xgood){
		Serial.println("Self-Test for X axis failed");
	}

	if(!Ygood){		
		Serial.println("Self-Test for Y axis failed");
	}
	if(!Zgood){
		Serial.println("Self-Test for Z axis failed");
	}
	if (Xgood && Ygood && Zgood){
		self_test = ADXL345_SELF_TEST_PASS;
	}
	else{
		self_test = ADXL345_SELF_TEST_FAIL;
	}
	return Xgood && Ygood && Zgood;
}

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void ADXL345::init(){

  writeToI2C(ADXL345_DATA_FORMAT, ADXL345_DISABLE_SELF_TEST <<  7 | ADXL345_4_WIRE_SPI_MODE << 6 | ADXL345_INTERUPT_HIGH << 5 | ADXL345_FULL_RESOLUTION_MODE << 3 | ADXL345_RIGHT_JUSTIFY << 2 | ADXL345_RANGE_16_G); // (0| disable self test), (0| 4-wire SPI mode), (0| interrupts active high), (0| fill bit), (1| full-resolution), (0| right-justified), (11| 16 g mode)
  powerOn();                     // Power on the ADXL345
  
  setInterruptMapping(ADXL345_INT_WATERMARK_BIT, ADXL345_INT1_PIN);    // Map Watermark interrupt to int pin 1
  setInterrupt(ADXL345_INT_WATERMARK_BIT, 1);                          // Enable Watermark interrupt 
  writeToI2C(ADXL345_FIFO_CTL,ADXL345_STREAM_MODE <<  6 | ADXL345_INT1_PIN << 5 | ADXL345_THIRTY_TWO_SAMPLES);			// (10|FIFO mode) (0|trigger to INT1) (11111|trigger at 32 samples)
  writeToI2C(ADXL345_BW_RATE, ADXL345_NORMAL_POWER_MODE << 4 | ADXL345_BW_400); // (000| filler bits), (0| normal power mode), (1101| 800Hz sampling)
  
}

void ADXL345::printDataFormat(){

	uint8_t dataFormat;
	readFromI2C(ADXL345_DATA_FORMAT,1,&dataFormat);
	Serial.print  ("Self Test: "); 
	switch((dataFormat & 0x80) >> 7)
	{
		case ADXL345_DISABLE_SELF_TEST:
		Serial.println  ("Disabled "); 
		break;
		case ADXL345_ENABLE_SELF_TEST:
		Serial.println  ("Enabled "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  
	Serial.print  ("SPI Mode: "); 
	switch((dataFormat & 0x40) >> 6)
	{
		case ADXL345_4_WIRE_SPI_MODE:
		Serial.println  ("4 Wire SPI Mode "); 
		break;
		case ADXL345_3_WIRE_SPI_MODE:
		Serial.println  ("3 Wire SPI Mode "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  
	Serial.print  ("Interupt Trigger: "); 
	switch((dataFormat & 0x20) >> 5)
	{
		case ADXL345_INTERUPT_HIGH:
		Serial.println  ("High "); 
		break;
		case ADXL345_INTERUPT_LOW:
		Serial.println  ("Low "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  
	
	Serial.print  ("Resolution Mode: "); 
	switch((dataFormat & 0x08) >> 4)
	{
		case ADXL345_FULL_RESOLUTION_MODE:
		Serial.println  ("Full Resolution Mode "); 
		break;
		case ADXL345_10_BIT_MODE:
		Serial.println  ("10 Bit Mode "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  

	Serial.print  ("Justify Mode: "); 
	switch((dataFormat & 0x04) >> 3)
	{
		case ADXL345_RIGHT_JUSTIFY:
		Serial.println  ("Right "); 
		break;
		case ADXL345_LEFT_JUSTIFY:
		Serial.println  ("Left "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  

	Serial.print  ("Range: +/- "); 
	switch(dataFormat & 0x03)
	{
		case ADXL345_RANGE_16_G:
		Serial.print  ("16 "); 
		break;
		case ADXL345_RANGE_8_G:
		Serial.print  ("8 "); 
		break;
		case ADXL345_RANGE_4_G:
		Serial.print  ("4 "); 
		break;
		case ADXL345_RANGE_2_G:
		Serial.print  ("2 "); 
		break;
		default:
		Serial.print  ("?? "); 
		break;
	}  
	Serial.println(" g"); 
}

void ADXL345::printBWRate(){
	uint8_t BWRate;
	readFromI2C(ADXL345_BW_RATE,1,&BWRate);
	Serial.print  ("Power Mode: "); 
	switch((BWRate & 0x10) >> 4)
	{
		case ADXL345_NORMAL_POWER_MODE:
		Serial.println  ("Normal Power Mode "); 
		break;
		case ADXL345_LOW_POWER_MODE:
		Serial.println  ("Low Power Mode "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  

	Serial.print  ("Bandwidth Rate: "); 
	switch(BWRate & 0x0F)
	{
		case ADXL345_BW_1600:
		Serial.print  ("1600"); 
		break;
		case ADXL345_BW_800:
		Serial.print  ("800"); 
		break;
		case ADXL345_BW_400:
		Serial.print  ("400"); 
		break;
		case ADXL345_BW_200:
		Serial.print  ("200"); 
		break;
		case ADXL345_BW_100:
		Serial.print  ("100"); 
		break;
		case ADXL345_BW_50:
		Serial.print  ("50"); 
		break;
		case ADXL345_BW_25:
		Serial.print  ("25"); 
		break;
		case ADXL345_BW_12_5:
		Serial.print  ("12.5"); 
		break;
		default:
		Serial.print  ("<= 6.25"); 
		break;
	}  
	Serial.println(" Hz"); 
}

void ADXL345::printFIFO_CTL(){
	uint8_t FIFOReg;
	readFromI2C(ADXL345_FIFO_CTL,1,&FIFOReg);
	Serial.print  ("FIFO Mode: "); 
	switch((FIFOReg & 0xC0) >> 6)
	{
		case ADXL345_TRIGGER_MODE:
		Serial.println  ("Trigger Mode "); 
		break;
		case ADXL345_STREAM_MODE:
		Serial.println  ("Stream Mode "); 
		break;
		case ADXL345_FIFO_MODE:
		Serial.println  ("FIFO Mode "); 
		break;
		case ADXL345_BYPASS_MODE:
		Serial.println  ("BYPASS Mode "); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}  
	Serial.print  ("Trigger: "); 
	switch((FIFOReg & 0x20) >> 5)
	{
		case ADXL345_INT1_PIN:
		Serial.println  ("INT 1 Pin"); 
		break;
		case ADXL345_INT2_PIN:
		Serial.println  ("INT 2 Pin"); 
		break;
		default:
		Serial.println  ("?? "); 
		break;
	}
	Serial.print  ("Samples: "); 
	Serial.println  ((FIFOReg & 0x1F) + 1); 
}
/****************** MAIN CODE ******************/
/* Accelerometer Readings */
void ADXL345::emptyFifo(){
	//unsigned long start = 0; // the time the delay started
	//start = millis();
	int x,y,z;   
	//Serial.println("starting fifo empty");
	// loop through fifo buffer and empty it

	uint8_t fifolength = 0;
	readFromI2C(ADXL345_FIFO_STATUS,1, &fifolength);
	fifolength = fifolength & (uint8_t)0b00111111;		//get the number of data values stored in FIFO
	//Serial.print("Buffer Size = ");
	//Serial.println(fifolength);
	for(int i =0; i < fifolength; i++){  

		readAccel(&x,&y,&z);

		buffer.push({x,y,z});
		delayMicroseconds(5);                       // minimum time between last read and start of the next read is 5 us
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
void ADXL345::clearAccel(){
	uint8_t fifolength = 0;
	int x,y,z;
	readFromI2C(ADXL345_FIFO_STATUS,1, &fifolength);
	fifolength = fifolength & (uint8_t)0b00111111;		//get the number of data values stored in FIFO
	for(int i =0; i < fifolength; i++){  //empty buffer
		
		readAccel(&x,&y,&z);

	}
}
uint8_t ADXL345::getSampleBufSize(){
	uint8_t fifolength = 0;
	readFromI2C(ADXL345_FIFO_STATUS,1, &fifolength);
	fifolength = fifolength & (uint8_t)0b00111111;		//get the number of data values stored in FIFO
	return fifolength;
}
void ADXL345::readAccel(int *x, int *y, int *z){

	accelwire.beginTransmission(ADXL345_DEVICE);
	accelwire.write(ADXL345_DATAX0);
	accelwire.endTransmission(false);
	accelwire.requestFrom(ADXL345_DEVICE, ADXL345_TO_READ, true);  // Request 6 Bytes

	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (short)((accelwire.read() | accelwire.read() << 8));
	*y = (short)((accelwire.read() | accelwire.read() << 8));
	*z = (short)((accelwire.read() | accelwire.read() << 8));
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
	accelwire.requestFrom(ADXL345_DEVICE, num);  
	int i = 0;
	while(accelwire.available())
	{
		_buff[i] = accelwire.read();				// Receive Byte
		i++;
	}
	if(i != num){
		status = ADXL345_ERROR;
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