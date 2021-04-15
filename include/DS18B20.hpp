#include <OneWire.h>
#include <queue>

 /****************************** ERRORS ******************************/
#define TEMPERATURE_OK			1		// No Error
#define TEMPERATURE_ERROR		0		// Error Exists

#define TEMPERATURE_NO_ERROR	  0		// Initial State
#define TEMPERATURE_NO_DATA       1     // No sensor data was found
#define TEMPERATURE_CRC_INVALID   2     // Cyclic redundancy check was invalid


class TemperatureSensor{
    public:
    	bool status;					// Set When Error Exists 
        byte error_code;
        OneWire sensor;
        uint8_t line = 0;
        byte addr[8]={0};
        byte type_s;
        TemperatureSensor(uint8_t);
        int16_t getTemp();
};