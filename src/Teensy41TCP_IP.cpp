// lwip perf
// to use IDE hack -I into boards.txt
#include <Arduino.h>
#include <NativeEthernet.h>
#include <queue>

#define TCPPORT 1602
#define DATA_TRANSMIT_ID 129;

void SendDataToControlRoom(uint8_t *buff, size_t buffSize, IPAddress controlRoomAddress, uint16_t controlRoomPort, EthernetClient sendClient) {
    //sendClient.write(buff, buffSize);
    sendClient.write("testing");
    delay(100);
}

//retune number of 8 bit chars required to transmit the presented data
uint32_t calcTransitSize(int16_t Acc0Size, int16_t Acc1Size, int16_t Acc2Size, int16_t Temp1Size, int16_t Temp2Size,int16_t ElEnSize,int32_t AzEnSize)
{
    uint32_t length = 1 + 16 + 4; //identifier + [16|ACCcount0,16|ACCcount1 ,16|ACCcount2,16|tmp1Count,16|Etmp2Count],16|Elencount|,32|Azencount|] + total data length
    length += (Acc0Size * 6);
    length += (Acc1Size * 6);
    length += (Acc2Size * 6);
    length += (Temp1Size * 2);
    length += (Temp2Size * 2);
    length += (ElEnSize * 2);
    length += (AzEnSize * 4);
    
    return length;
}

struct acc
{
    int x;
    int y;
    int z;
};
void prepairTransit(uint8_t *reply, uint32_t dataSize, std::queue <acc> *Acc0Buffer, std::queue <acc> *Acc1Buffer, std::queue <acc> *Acc2Buffer, std::queue <int16_t> *Temp1Buffer, std::queue <int16_t> *Temp2Buffer, std::queue <int16_t> *ElEnBuffer, std::queue <int32_t> *AzEnBuffer)
{
  
    //[16|ACCcount,16|AZetmpCount,16|ELetmpCount]
    //acc data length = accDat.buffer.size() * 6
    uint32_t i = 0;
    reply[0] = DATA_TRANSMIT_ID;
    reply[1] = (dataSize & 0xff000000) >> 24;
    reply[2] = (dataSize & 0x00ff0000) >> 16;
    reply[3] = (dataSize & 0x0000ff00) >> 8;
    reply[4] = dataSize & 0x000000ff;

    uint32_t acc0BufSize = Acc0Buffer->size();
    reply[5] = ((acc0BufSize * 6) & 0xff00) >> 8;
    reply[6] = ((acc0BufSize * 6) & 0x00ff);
    uint32_t acc1BufSize = Acc1Buffer->size();
    reply[7] = ((acc1BufSize * 6) & 0xff00) >> 8;
    reply[8] = ((acc1BufSize * 6) & 0x00ff);
    uint32_t acc2BufSize = Acc2Buffer->size();
    reply[9] = ((acc2BufSize * 6) & 0xff00) >> 8;
    reply[10] = ((acc2BufSize * 6) & 0x00ff);

    uint32_t temp1BufSize = Temp1Buffer->size();
    reply[11] = (temp1BufSize & 0xff00) >> 8;
    reply[12] = (temp1BufSize & 0x00ff);

    uint32_t temp2BufSize = Temp1Buffer->size();
    reply[13] = (temp2BufSize & 0xff00) >> 8;
    reply[14] = (temp2BufSize & 0x00ff);

    uint32_t elEnBufSize = ElEnBuffer->size();
    reply[15] = (elEnBufSize & 0xff00) >> 8;
    reply[16] = (elEnBufSize & 0x00ff);

    uint32_t azEnBufferSize = AzEnBuffer->size();
    reply[17] = (azEnBufferSize & 0xff000000) >> 24;
    reply[18] = (azEnBufferSize & 0x00ff0000) >> 16;
    reply[19] = (azEnBufferSize & 0x0000ff00) >> 8;
    reply[20] = azEnBufferSize & 0x000000ff;

    i = 21;
    for (uint32_t j = 0; j < acc0BufSize; j++)
    {
        acc current = Acc0Buffer->front();
        reply[i++] = (current.x & 0xff00) >> 8;
        reply[i++] = (current.x & 0x00ff);
        reply[i++] = (current.y & 0xff00) >> 8;
        reply[i++] = (current.y & 0x00ff);
        reply[i++] = (current.z & 0xff00) >> 8;
        reply[i++] = (current.z & 0x00ff);
        Acc0Buffer->pop();
    }
    for (uint32_t j = 0; j < acc1BufSize; j++)
    {
        acc current = Acc1Buffer->front();
        reply[i++] = (current.x & 0xff00) >> 8;
        reply[i++] = (current.x & 0x00ff);
        reply[i++] = (current.y & 0xff00) >> 8;
        reply[i++] = (current.y & 0x00ff);
        reply[i++] = (current.z & 0xff00) >> 8;
        reply[i++] = (current.z & 0x00ff);
        Acc1Buffer->pop();
    }
    for (uint32_t j = 0; j < acc2BufSize; j++)
    {
        acc current = Acc2Buffer->front();
        reply[i++] = (current.x & 0xff00) >> 8;
        reply[i++] = (current.x & 0x00ff);
        reply[i++] = (current.y & 0xff00) >> 8;
        reply[i++] = (current.y & 0x00ff);
        reply[i++] = (current.z & 0xff00) >> 8;
        reply[i++] = (current.z & 0x00ff);
        Acc2Buffer->pop();
    }
    for (uint32_t j = 0; j < temp1BufSize; j++)
    {
        int16_t current = Temp1Buffer->front();
        reply[i++] = (current & 0x00ff);
        reply[i++] = (current & 0xff00) >> 8;
        Temp1Buffer->pop();
    }
    for (uint32_t j = 0; j < temp2BufSize; j++)
    {
        int16_t current = Temp2Buffer->front();
        reply[i++] = (current & 0x00ff);
        reply[i++] = (current & 0xff00) >> 8;
        Temp2Buffer->pop();
    }
    for (uint32_t j = 0; j < elEnBufSize; j++)
    {
        int16_t current = ElEnBuffer->front();
        reply[i++] = (current & 0x00ff);
        reply[i++] = (current & 0xff00) >> 8;
        ElEnBuffer->pop();
    }
    for (uint32_t j = 0; j < azEnBufferSize; j++)
    {
        int32_t current = AzEnBuffer->front();
        reply[i++] = (current & 0xff000000) >> 24;
        reply[i++] = (current & 0x00ff0000) >> 16;
        reply[i++] = (current & 0x0000ff00) >> 8;
        reply[i++] = current & 0x000000ff;
        AzEnBuffer->pop();
    }
    
}