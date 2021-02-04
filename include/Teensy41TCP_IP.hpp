#include <Arduino.h>
#include <queue>
#include "ADXL345_Accelerometer.hpp"


void initEthernet();
void SendDataToControlRoom(uint8_t *, size_t);
void ethernetloop();
uint32_t calcTransitSize(int16_t, int16_t, int16_t, int16_t, int16_t,int16_t,int32_t);
void prepairTransit(uint8_t *, uint32_t, std::queue <acc> *, std::queue <acc> *, std::queue <acc> *, std::queue <int16_t> *, std::queue <int16_t> *,std::queue <int16_t> *,std::queue <int32_t> *);