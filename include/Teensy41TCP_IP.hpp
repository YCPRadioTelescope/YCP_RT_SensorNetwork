#include <Arduino.h>
#include <queue>
#include "ADXL345_Accelerometer.hpp"
#include <NativeEthernet.h>

void SendDataToControlRoom(uint8_t *, size_t, IPAddress, uint16_t, EthernetClient);
uint32_t calcTransitSize(int16_t Acc0Size, int16_t Acc1Size, int16_t Acc2Size, int16_t Temp1Size, int16_t Temp2Size,int16_t ElEnSize,int16_t AzEnSize,int16_t AmbTemp,int16_t AmbHumidity);
void prepairTransit(uint8_t *reply, uint32_t dataSize, std::queue <accDump> *AccElBuffer, std::queue <accDump> *AccAzBuffer,
                    std::queue <accDump> *AccCbBuffer, std::queue <int16_t> *TempElBuffer, std::queue <int16_t> *TempAzBuffer,
                    std::queue <int16_t> *ElEnBuffer, std::queue <int16_t> *AzEnBuffer, std::queue <float> *ambientTempBuffer, std::queue <float> *ambientHumidityBuffer,
                    uint16_t sensorStatuses, uint32_t sensorErrors);