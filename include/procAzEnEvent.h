#include <SPI.h>
#include <queue>

class AzimuthEncoder
{
public:
    std::queue <int16_t> buffer;
    AzimuthEncoder();
    void init(void);
    void procAzEnEvent(void);
};