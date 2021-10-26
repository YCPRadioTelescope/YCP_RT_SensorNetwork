#include "Arduino.h"
#include <Wire.h> 
#include <queue>
#ifndef ADXL345_h
#define ADXL345_h

/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID			0x00		// Device ID
#define ADXL345_RESERVED1		0x01		// Reserved. Do Not Access. 
#define ADXL345_THRESH_TAP		0x1D		// Tap Threshold. 
#define ADXL345_OFSX			0x1E		// X-Axis Offset. 
#define ADXL345_OFSY			0x1F		// Y-Axis Offset.
#define ADXL345_OFSZ			0x20		// Z- Axis Offset.
#define ADXL345_DUR				0x21		// Tap Duration.
#define ADXL345_LATENT			0x22		// Tap Latency.
#define ADXL345_WINDOW			0x23		// Tap Window.
#define ADXL345_THRESH_ACT		0x24		// Activity Threshold
#define ADXL345_THRESH_INACT	0x25		// Inactivity Threshold
#define ADXL345_TIME_INACT		0x26		// Inactivity Time
#define ADXL345_ACT_INACT_CTL	0x27		// Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF		0x28		// Free-Fall Threshold.
#define ADXL345_TIME_FF			0x29		// Free-Fall Time.
#define ADXL345_TAP_AXES		0x2A		// Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS	0x2B		// Source of Tap/Double Tap
#define ADXL345_BW_RATE			0x2C		// Data Rate and Power mode Control
#define ADXL345_POWER_CTL		0x2D		// Power-Saving Features Control
#define ADXL345_INT_ENABLE		0x2E		// Interrupt Enable Control
#define ADXL345_INT_MAP			0x2F		// Interrupt Mapping Control
#define ADXL345_INT_SOURCE		0x30		// Source of Interrupts
#define ADXL345_DATA_FORMAT		0x31		// Data Format Control
#define ADXL345_DATAX0			0x32		// X-Axis Data 0
#define ADXL345_DATAX1			0x33		// X-Axis Data 1
#define ADXL345_DATAY0			0x34		// Y-Axis Data 0
#define ADXL345_DATAY1			0x35		// Y-Axis Data 1
#define ADXL345_DATAZ0			0x36		// Z-Axis Data 0
#define ADXL345_DATAZ1			0x37		// Z-Axis Data 1
#define ADXL345_FIFO_CTL		0x38		// FIFO Control
#define ADXL345_FIFO_STATUS		0x39		// FIFO Status

#define ADXL345_BW_1600			0xF			// 1111		IDD = 40uA
#define ADXL345_BW_800			0xE			// 1110		IDD = 90uA
#define ADXL345_BW_400			0xD			// 1101		IDD = 140uA
#define ADXL345_BW_200			0xC			// 1100		IDD = 140uA
#define ADXL345_BW_100			0xB			// 1011		IDD = 140uA 
#define ADXL345_BW_50			0xA			// 1010		IDD = 140uA
#define ADXL345_BW_25			0x9			// 1001		IDD = 90uA
#define ADXL345_BW_12_5		    0x8			// 1000		IDD = 60uA 
#define ADXL345_BW_6_25			0x7			// 0111		IDD = 50uA
#define ADXL345_BW_3_13			0x6			// 0110		IDD = 45uA
#define ADXL345_BW_1_56			0x5			// 0101		IDD = 40uA
#define ADXL345_BW_0_78			0x4			// 0100		IDD = 34uA
#define ADXL345_BW_0_39			0x3			// 0011		IDD = 23uA
#define ADXL345_BW_0_20			0x2			// 0010		IDD = 23uA
#define ADXL345_BW_0_10			0x1			// 0001		IDD = 23uA
#define ADXL345_BW_0_05			0x0			// 0000		IDD = 23uA




 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN		0x00		//INT1: 0
#define ADXL345_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT		0x07
#define ADXL345_INT_SINGLE_TAP_BIT		0x06
#define ADXL345_INT_DOUBLE_TAP_BIT		0x05
#define ADXL345_INT_ACTIVITY_BIT		0x04
#define ADXL345_INT_INACTIVITY_BIT		0x03
#define ADXL345_INT_FREE_FALL_BIT		0x02
#define ADXL345_INT_WATERMARK_BIT		0x01
#define ADXL345_INT_OVERRUNY_BIT		0x00

#define ADXL345_DATA_READY				0x07
#define ADXL345_SINGLE_TAP				0x06
#define ADXL345_DOUBLE_TAP				0x05
#define ADXL345_ACTIVITY				0x04
#define ADXL345_INACTIVITY				0x03
#define ADXL345_FREE_FALL				0x02
#define ADXL345_WATERMARK				0x01
#define ADXL345_OVERRUNY				0x00

/********************** POWER MODES **********************/
#define ADXL345_LOW_POWER_MODE		0x01
#define ADXL345_NORMAL_POWER_MODE	0x00

/********************** FIFO MODES **********************/
#define ADXL345_TRIGGER_MODE		0x03
#define ADXL345_STREAM_MODE			0x02
#define ADXL345_FIFO_MODE			0x01
#define ADXL345_BYPASS_MODE			0x00

/********************** SAMPLE NUMBERS **********************/
#define ADXL345_ONE_SAMPLES				0x00
#define ADXL345_TWO_SAMPLES				0x01
#define ADXL345_THREE_SAMPLES			0x02
#define ADXL345_FOUR_SAMPLES			0x03
#define ADXL345_FIVE_SAMPLES			0x04
#define ADXL345_SIX_SAMPLES				0x05
#define ADXL345_SEVEN_SAMPLES			0x06
#define ADXL345_EIGHT_SAMPLES			0x07
#define ADXL345_NINE_SAMPLES			0x08
#define ADXL345_TEN_SAMPLES				0x09
#define ADXL345_ELEVEN_SAMPLES			0x0A
#define ADXL345_TWELVE_SAMPLES			0x0B
#define ADXL345_THIRTEEN_SAMPLES		0x0C
#define ADXL345_FOURTEEN_SAMPLES		0x0D
#define ADXL345_FITHTEEN_SAMPLES		0x0E
#define ADXL345_SIXTEEN_SAMPLES			0x0F
#define ADXL345_SEVENTEEN_SAMPLES		0x10
#define ADXL345_EIGHTEEN_SAMPLES		0x11
#define ADXL345_NINETEEN_SAMPLES		0x12
#define ADXL345_TWENTY_SAMPLES			0x13
#define ADXL345_TWENTY_ONE_SAMPLES		0x14
#define ADXL345_TWENTY_TWO_SAMPLES		0x15
#define ADXL345_TWENTY_THREE_SAMPLES	0x16
#define ADXL345_TWENTY_FOUR_SAMPLES		0x17
#define ADXL345_TWENTY_FIVE_SAMPLES		0x18
#define ADXL345_TWENTY_SIX_SAMPLES		0x19
#define ADXL345_TWENTY_SEVEN_SAMPLES	0x1A
#define ADXL345_TWENTY_EIGHT_SAMPLES	0x1B
#define ADXL345_TWENTY_NINE_SAMPLES		0x1C
#define ADXL345_THIRTY_SAMPLES			0x1D
#define ADXL345_THIRTY_ONE_SAMPLES		0x1E
#define ADXL345_THIRTY_TWO_SAMPLES		0x1F

/************************** SELF-TEST **************************/
#define ADXL345_DISABLE_SELF_TEST		0x00		
#define ADXL345_ENABLE_SELF_TEST		0x01		

/************************** SPI MODE **************************/
#define ADXL345_4_WIRE_SPI_MODE		0x00		
#define ADXL345_3_WIRE_SPI_MODE		0x01	

/************************** INTERRUPT TRIGGER **************************/
#define ADXL345_INTERUPT_HIGH		0x00		
#define ADXL345_INTERUPT_LOW		0x01	

/************************** RESOLUTION **************************/
#define ADXL345_10_BIT_MODE				0x00		
#define ADXL345_FULL_RESOLUTION_MODE	0x01	

/************************** JUSTIFY BIT **************************/
#define ADXL345_10_BIT_MODE				0x00		
#define ADXL345_FULL_RESOLUTION_MODE	0x01	

 /****************************** JUSTIFY BIT ******************************/
#define ADXL345_RIGHT_JUSTIFY		0x00		
#define ADXL345_LEFT_JUSTIFY		0x01

/****************************** RANGE BITS ******************************/
#define ADXL345_RANGE_2_G		0x00		
#define ADXL345_RANGE_4_G		0x01
#define ADXL345_RANGE_8_G		0x02		
#define ADXL345_RANGE_16_G		0x03

 /****************************** ERRORS ******************************/
#define ADXL345_OK			1		// No Error
#define ADXL345_ERROR		0		// Error Exists

#define ADXL345_SELF_TEST_PASS		1		// Self-test passed
#define ADXL345_SELF_TEST_FAIL		0		// Self-test failed

#define ADXL345_NO_ERROR			0	// Initial State
#define ADXL345_NO_SAMPLES			1	// Sensor stopped sampling
#define ADXL345_WATERMARK_MISSED	2	// Sensor missed the watermark and will not trigger the interupt

struct acc
{
    int x;
    int y;
    int z;
};


class ADXL345
{
public:
	bool status;					// Set When Error Exists 
	byte error_code;
	bool self_test;					// True if self-test passed
	std::queue <acc> buffer;		// queue of acc buffers. This caused some references to be undefined in linker
	double gains[3];				// Counts to Gs
	int wirenumber;
	
	ADXL345(TwoWire& wire);
	
	bool selfTest();
    void init();
    void init(byte samplingRate, byte xOff, byte yOff, byte zOff);
    void emptyFifo();
	void powerCycle(uint8_t pinNumber);

	void powerOn();
	void readAccel(int* x, int* y, int* z);
	void clearAccel();
	uint8_t getSampleBufSize();

	void setInterruptMapping(byte interruptBit, bool interruptPin);
	void setInterrupt(byte interruptBit, bool state);
    
	void printDataFormat();
	void printBWRate();
	void printFIFO_CTL();
	
private:
	void writeToI2C(byte address, byte val);
	void readFromI2C(byte address, int num, byte buff[]);
	void setRegisterBit(byte regAdress, int bitPos, bool state);
	bool getRegisterBit(byte regAdress, int bitPos);  
	byte _buff[6] ;		//	6 Bytes Buffer
	int _CS = 10;
	TwoWire& accelwire;
	bool I2C = true;
	unsigned long SPIfreq = 5000000;
	
};
void print_byte(byte val);
#endif