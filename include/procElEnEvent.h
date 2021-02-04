#include <Arduino.h>
#include <queue>

class ElevationEncoder
{
public:
    ElevationEncoder();
    void procElEnEvent(void);
    std::queue <int16_t> buffer;
};