#include <Arduino.h>
#include <queue>
#include "ADXL345_Accelerometer.hpp"
#include <NativeEthernet.h>

void SendDataToControlRoom(uint8_t *, size_t, IPAddress, uint16_t, EthernetClient);
uint32_t calcTransitSize(int16_t, int16_t, int16_t, int16_t, int16_t,int16_t,int16_t);
void prepairTransit(uint8_t *, uint32_t, std::queue <accDump> *, std::queue <accDump> *, std::queue <accDump> *, std::queue <int16_t> *, std::queue <int16_t> *,std::queue <int16_t> *,std::queue <int16_t> *, uint8_t, uint32_t);