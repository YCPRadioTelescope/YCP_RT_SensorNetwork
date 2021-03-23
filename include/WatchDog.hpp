#include <Arduino.h>

class WatchDog
{
    public:
        void init();
        void reset();
        void feed();
};