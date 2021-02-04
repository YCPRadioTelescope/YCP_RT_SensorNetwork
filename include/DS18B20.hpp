#include <OneWire.h>
#include <queue>

class TemperatureSensor{
    public:
        OneWire sensor;
        uint8_t line = 0;
        byte addr[8]={0};
        byte type_s;
        std::queue <int16_t> buffer;
        TemperatureSensor(uint8_t);
        int getTemp();
};