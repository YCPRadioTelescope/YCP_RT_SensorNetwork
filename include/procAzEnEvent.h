#include <SPI.h>

class AzimuthEncoder
{
public:
    AzimuthEncoder();
    void init(void);
    void procAzEnEvent(void);
};