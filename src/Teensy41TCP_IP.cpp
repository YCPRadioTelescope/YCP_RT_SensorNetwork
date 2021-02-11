// lwip perf
// to use IDE hack -I into boards.txt
#include <Arduino.h>
#include "Teensy41EthernetHelp/lwip_t41.h"
#include "Teensy41EthernetHelp/lwip/inet.h"
#include "Teensy41EthernetHelp/lwip/dhcp.h"
#include "Teensy41EthernetHelp/lwip/tcp.h"
#include "Teensy41EthernetHelp/lwip/stats.h"
#include <queue>

#define swap2 __builtin_bswap16
#define swap4 __builtin_bswap32

uint32_t rtt;

#define PHY_ADDR 0 /*for read/write PHY registers (check link status,...)*/
#define DHCP 0
#define IP "192.168.0.123"
#define MASK "255.255.255.0"
#define GW "192.168.1.1"

#define DATA_TRANSMIT_ID 132

// debug stats stuff
extern "C" {
#if LWIP_STATS
  struct stats_ lwip_stats;
#endif
}

void ethernetloop()
{
  static uint32_t last_ms;
  uint32_t ms;

  enet_proc_input();

  ms = millis();
  if (ms - last_ms > 100)
  {
    last_ms = ms;
    enet_poll();
  }
}

void print_stats() {
  // lwip stats_display() needed printf
#if LWIP_STATS
  char str[128];

  // my  LINK stats
  sprintf(str, "LINK in %d out %d drop %d memerr %d",
          lwip_stats.link.recv, lwip_stats.link.xmit, lwip_stats.link.drop, lwip_stats.link.memerr);
  Serial.println(str);
  sprintf(str, "TCP in %d out %d drop %d memerr %d",
          lwip_stats.tcp.recv, lwip_stats.tcp.xmit, lwip_stats.tcp.drop, lwip_stats.tcp.memerr);
  Serial.println(str);
  sprintf(str, "UDP in %d out %d drop %d memerr %d",
          lwip_stats.udp.recv, lwip_stats.udp.xmit, lwip_stats.udp.drop, lwip_stats.udp.memerr);
  Serial.println(str);
  sprintf(str, "ICMP in %d out %d",
          lwip_stats.icmp.recv, lwip_stats.icmp.xmit);
  Serial.println(str);
  sprintf(str, "ARP in %d out %d",
          lwip_stats.etharp.recv, lwip_stats.etharp.xmit);
  Serial.println(str);
#if MEM_STATS
  sprintf(str, "HEAP avail %d used %d max %d err %d",
          lwip_stats.mem.avail, lwip_stats.mem.used, lwip_stats.mem.max, lwip_stats.mem.err);
  Serial.println(str);
#endif
#endif
}

#define PRREG(x) Serial.printf(#x" 0x%x\n",x)

void prregs() {

  PRREG(ENET_PALR);
  PRREG(ENET_PAUR);
  PRREG(ENET_EIR);
  PRREG(ENET_EIMR);
  PRREG(ENET_ECR);
  PRREG(ENET_MSCR);
  PRREG(ENET_MRBR);
  PRREG(ENET_RCR);
  PRREG(ENET_TCR);
  PRREG(ENET_TACC);
  PRREG(ENET_RACC);
  PRREG(ENET_MMFR);
}



static void teensyMAC(uint8_t *mac)
{
  uint32_t m1 = HW_OCOTP_MAC1;
  uint32_t m2 = HW_OCOTP_MAC0;
  mac[0] = m1 >> 8;
  mac[1] = m1 >> 0;
  mac[2] = m2 >> 24;
  mac[3] = m2 >> 16;
  mac[4] = m2 >> 8;
  mac[5] = m2 >> 0;
}


static void netif_status_callback(struct netif *netif)
{
  static char str1[IP4ADDR_STRLEN_MAX], str2[IP4ADDR_STRLEN_MAX], str3[IP4ADDR_STRLEN_MAX];
  Serial.printf("netif status changed: ip %s, mask %s, gw %s\n", ip4addr_ntoa_r(netif_ip_addr4(netif), str1, IP4ADDR_STRLEN_MAX), ip4addr_ntoa_r(netif_ip_netmask4(netif), str2, IP4ADDR_STRLEN_MAX), ip4addr_ntoa_r(netif_ip_gw4(netif), str3, IP4ADDR_STRLEN_MAX));
}

static void link_status_callback(struct netif *netif)
{
  Serial.printf("enet link status: %s\n", netif_is_link_up(netif) ? "up" : "down");
}
//   ----- TCP ------

void tcperr_callback(void * arg, err_t err)
{
  // set with tcp_err()
  Serial.print("TCP err "); Serial.println(err);
  *(int *)arg = err;
}

err_t connect_callback(void *arg, struct tcp_pcb *tpcb, err_t err) {
  Serial.print("connected "); Serial.println(tcp_sndbuf(tpcb));
  *(int *)arg = 1;
  return 0;
}
void InitalConnectToControlRoom(){
  ip_addr_t server;
  struct tcp_pcb * pcb;
  int connected = 0;
  err_t err;
  uint32_t sendqlth;

  Serial.println("tcptx");
  inet_aton(IP, &server);
  pcb = tcp_new();
  tcp_err(pcb, tcperr_callback);
  tcp_arg(pcb, &connected);
  tcp_bind(pcb, IP_ADDR_ANY, 3333);   // local port
  sendqlth = tcp_sndbuf(pcb);
  Serial.println(sendqlth);
  do {
    err = tcp_connect(pcb, &server, 139, connect_callback);
    //Serial.print("err ");Serial.println(err);
    ethernetloop();
  } while (err < 0);
  while (!connected) ethernetloop();
  if (connected < 0) {
    Serial.println("connect error");
    return;  // err
  }
}
void SendDataToControlRoom(uint8_t *buff, size_t buffSize) {
  
  // send to ttcp -r -s
  ip_addr_t server;
  struct tcp_pcb * pcb;
  int connected = 0;
  err_t err;
  uint32_t t, sendqlth;

  Serial.println("tcptx");
  inet_aton(IP, &server);
  pcb = tcp_new();
  tcp_err(pcb, tcperr_callback);
  tcp_arg(pcb, &connected);
  tcp_bind(pcb, IP_ADDR_ANY, 3333);   // local port
  sendqlth = tcp_sndbuf(pcb);
  Serial.println(sendqlth);
  do {
    err = tcp_connect(pcb, &server, 139, connect_callback);
    //Serial.print("err ");Serial.println(err);
    ethernetloop();
  } while (err < 0);
  while (!connected) ethernetloop();
  if (connected < 0) {
    Serial.println("connect error");
    return;  // err
  }
  
  t = micros();
  do {
    err = tcp_write(pcb, buff, buffSize, TCP_WRITE_FLAG_COPY);
    ethernetloop();   // keep checkin while we blast
  } while ( err < 0);  // -1 is ERR_MEM
  tcp_output(pcb);
  
  while (tcp_sndbuf(pcb) != sendqlth) ethernetloop(); // wait til sent
  tcp_close(pcb);
  t = micros() - t;
  Serial.print(t); Serial.print(" us  "); Serial.println(buffSize);
  
}

void initEthernet()
{

  Serial.println(); Serial.print(F_CPU); Serial.print(" ");

  Serial.print(__TIME__); Serial.print(" "); Serial.println(__DATE__);
  Serial.printf("PHY_ADDR %d\n", PHY_ADDR);
  uint8_t mac[6];
  teensyMAC(mac);
  Serial.printf("MAC_ADDR %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("DHCP is %s\n", DHCP == 1 ? "on" : "off");

  ip_addr_t ip, mask, gateway;
  if (DHCP == 1)
  {
    ip = IPADDR4_INIT(IPADDR_ANY);
    mask = IPADDR4_INIT(IPADDR_ANY);
    gateway = IPADDR4_INIT(IPADDR_ANY);
  }
  else
  {
    inet_aton(IP, &ip);
    inet_aton(MASK, &mask);
    inet_aton(GW, &gateway);
  }
  enet_init(&ip, &mask, &gateway);
  netif_set_status_callback(netif_default, netif_status_callback);
  netif_set_link_callback(netif_default, link_status_callback);
  netif_set_up(netif_default);

  if (DHCP == 1)
    dhcp_start(netif_default);

  while (!netif_is_link_up(netif_default)) ethernetloop(); // await on link up
  prregs();

  InitalConnectToControlRoom();
  //SendDataToControlRoom();
  //tcptx(10000);
  //tcprx();

#if 0
  // optional stats every 5 s, need LWIP_STATS 1 lwipopts.h
  while (1) {
    static uint32_t ms = millis();
    if (millis() - ms > 5000) {
      ms = millis();
      print_stats();
    }
    loop();  // poll
  }
#endif
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