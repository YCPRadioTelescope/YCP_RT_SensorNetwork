#include <OneWire.h>

class TemperatureSensor{
    public:
        OneWire sensor;
        uint8_t line = 0;
        byte addr[8]={0};
        byte type_s;
        TemperatureSensor(uint8_t);
        int getTemp();
};