// lwip perf
// to use IDE hack -I into boards.txt
#include <Arduino.h>
#include <NativeEthernet.h>
#include <queue>

#define TCPPORT 1600
#define DATA_TRANSMIT_ID 129;


void SendDataToControlRoom(uint8_t *buff, size_t buffSize, IPAddress controlRoomAddress, uint16_t controlRoomPort, EthernetClient sendClient) {
    //Check if the teensy client is available
    sendClient.write(buff, buffSize);
    
}

//retune number of 8 bit chars required to transmit the presented data
uint32_t calcTransitSize(int16_t Acc0Size, int16_t Acc1Size, int16_t Acc2Size, int16_t Temp1Size, int16_t Temp2Size,int16_t ElEnSize,int16_t AzEnSize)
{
    uint32_t length = 1 + 4 + 14 + 1 + 3; //identifier + total data length + [16|ACCcount0,16|ACCcount1 ,16|ACCcount2,16|tmp1Count,16|Etmp2Count],16|Elencount|,16|Azencount|] + Status + Error Codes
    length += (Acc0Size * 6);
    length += (Acc1Size * 6);
    length += (Acc2Size * 6);
    length += (Temp1Size * 2);
    length += (Temp2Size * 2);
    length += (ElEnSize * 2);
    length += (AzEnSize * 2);
    
    return length;
}

struct acc
{
    int x;
    int y;
    int z;
};
void prepairTransit(uint8_t *reply, uint32_t dataSize, std::queue <acc> *AccElBuffer, std::queue <acc> *AccAzBuffer,
                     std::queue <acc> *AccCbBuffer, std::queue <int16_t> *TempElBuffer, std::queue <int16_t> *TempAzBuffer,
                      std::queue <int16_t> *ElEnBuffer, std::queue <int16_t> *AzEnBuffer, uint8_t sensorStatuses, uint32_t sensorErrors)
{
  
    //[16|ACCcount,16|AZetmpCount,16|ELetmpCount]
    //acc data length = accDat.buffer.size() * 6
    //Serial.println("Data size = ");
    //Serial.println(dataSize);
    
    // Transmit ID
    uint32_t i = 0;
    reply[0] = DATA_TRANSMIT_ID;
    reply[1] = (dataSize & 0xff000000) >> 24;
    reply[2] = (dataSize & 0x00ff0000) >> 16;
    reply[3] = (dataSize & 0x0000ff00) >> 8;
    reply[4] = dataSize & 0x000000ff;
    // Sensor Statuses
    reply[5] = sensorStatuses;
    // Sensor Error Codes
    reply[6] = (sensorErrors & 0x00ff0000) >> 16;   
    reply[7] = (sensorErrors & 0x0000ff00) >> 8;
    reply[8] = sensorErrors & 0x000000ff;
    // Elvation ADXL data size
    uint32_t accElBufSize = AccElBuffer->size();
    reply[9] = ((accElBufSize) & 0xff00) >> 8;
    reply[10] = ((accElBufSize) & 0x00ff);
    //Serial.println("AdxlEl size = ");
    //Serial.println(accElBufSize);

    // Azimuth ADXL data size
    uint32_t accAzBufSize = AccAzBuffer->size();
    reply[11] = ((accAzBufSize) & 0xff00) >> 8;
    reply[12] = ((accAzBufSize) & 0x00ff);
    //Serial.println("AdxlAz size = ");
    //Serial.println(accAzBufSize);

    // Counter Balance ADXL data size
    uint32_t accCbBufSize = AccCbBuffer->size();
    reply[13] = ((accCbBufSize) & 0xff00) >> 8;
    reply[14] = ((accCbBufSize) & 0x00ff);
    //Serial.println("AdxlCb size = ");
    //Serial.println(accCbBufSize);

    // Elvation Temperature data size
    uint32_t tempElBufSize = TempElBuffer->size();
    reply[15] = ((tempElBufSize) & 0xff00) >> 8;
    reply[16] = ((tempElBufSize ) & 0x00ff);
    //Serial.println("TempElBuffer size = ");
    //Serial.println(tempElBufSize);

    // Azimuth Temperature data size
    uint32_t tempAzBufSize = TempAzBuffer->size();
    reply[17] = ((tempAzBufSize) & 0xff00) >> 8;
    reply[18] = ((tempAzBufSize) & 0x00ff);
    //Serial.println("TempAzBuffer size = ");
    //Serial.println(tempAzBufSize);

    // Elvation Encoder data size
    uint32_t elEnBufSize = ElEnBuffer->size();
    reply[19] = ((elEnBufSize) & 0xff00) >> 8;
    reply[20] = ((elEnBufSize) & 0x00ff);
    //Serial.println("elEnBuf size = ");
    //Serial.println(elEnBufSize);

    // Azimuth Encoder data size
    uint32_t azEnBufferSize = AzEnBuffer->size();
    reply[21] = ((azEnBufferSize) & 0xff00) >> 8;
    reply[22] = ((azEnBufferSize) & 0x00ff);
    //Serial.println("azEnBuf size = ");
    //Serial.println(azEnBufferSize);

    i = 23;
    // Elvation ADXL data
    for (uint32_t j = 0; j < accElBufSize; j++)
    {
        acc current = AccElBuffer->front();
        reply[i++] = (current.x & 0xff00) >> 8;
        reply[i++] = (current.x & 0x00ff);
        reply[i++] = (current.y & 0xff00) >> 8;
        reply[i++] = (current.y & 0x00ff);
        reply[i++] = (current.z & 0xff00) >> 8;
        reply[i++] = (current.z & 0x00ff);
        AccElBuffer->pop();
    }
    // Azimuth ADXL data
    for (uint32_t j = 0; j < accAzBufSize; j++)
    {
        acc current = AccAzBuffer->front();
        reply[i++] = (current.x & 0xff00) >> 8;
        reply[i++] = (current.x & 0x00ff);
        reply[i++] = (current.y & 0xff00) >> 8;
        reply[i++] = (current.y & 0x00ff);
        reply[i++] = (current.z & 0xff00) >> 8;
        reply[i++] = (current.z & 0x00ff);
        AccAzBuffer->pop();
    }
    // Counter Balance ADXL data
    for (uint32_t j = 0; j < accCbBufSize; j++)
    {
        acc current = AccCbBuffer->front();
        reply[i++] = (current.x & 0xff00) >> 8;
        reply[i++] = (current.x & 0x00ff);
        reply[i++] = (current.y & 0xff00) >> 8;
        reply[i++] = (current.y & 0x00ff);
        reply[i++] = (current.z & 0xff00) >> 8;
        reply[i++] = (current.z & 0x00ff);
        AccCbBuffer->pop();
    }
    // Elvation Temperature data
    for (uint32_t j = 0; j < tempElBufSize; j++)
    {
        int16_t current = TempElBuffer->front();
        reply[i++] = (current & 0xff00) >> 8;
        reply[i++] = (current & 0x00ff);
        TempElBuffer->pop();
    }
     // Azimuth Temperature data
    for (uint32_t j = 0; j < tempAzBufSize; j++)
    {
        int16_t current = TempAzBuffer->front();
        reply[i++] = (current & 0xff00) >> 8;
        reply[i++] = (current & 0x00ff);
        TempAzBuffer->pop();
    }
    // Elvation Encoder data
    for (uint32_t j = 0; j < elEnBufSize; j++)
    {
        int16_t current = ElEnBuffer->front();
        reply[i++] = (current & 0xff00) >> 8;
        reply[i++] = (current & 0x00ff);
        ElEnBuffer->pop();
    }
    // Azimuth Encoder data
    for (uint32_t j = 0; j < azEnBufferSize; j++)
    {
        int16_t current = AzEnBuffer->front();
        reply[i++] = (current & 0xff00) >> 8;
        reply[i++] = (current & 0x00ff);
        AzEnBuffer->pop();
    }
    
}