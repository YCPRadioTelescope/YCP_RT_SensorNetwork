#include <SPI.h>
#include <queue>

 /****************************** ERRORS ******************************/
#define AZ_ENCODER_OK			1		// No Error
#define AZ_ENCODER_ERROR		0		// Error Exists

#define AZ_ENCODER_NO_ERROR	        0	    // Initial State
#define AZ_ENCODER_BAD_DATA		    1       // Valid flag return false
#define AZ_ENCODER_STALE_DATA		2       // Sync flag return false

class AzimuthEncoder
{
public:
    bool status;					// Set When Error Exists 
    byte error_code;
    std::queue <int16_t> buffer;
    AzimuthEncoder();
    void init(void);
    void procAzEnEvent(void);
};