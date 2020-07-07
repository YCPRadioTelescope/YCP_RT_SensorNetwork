#include <Wire.h>

class Accelerometer{
    public:
    byte values[6];
    char output [512];
    Accelerometer();
    int init();
    int getCords();
};