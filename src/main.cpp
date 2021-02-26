
#include <OneWire.h>
#include <Wire.h>
#include "Teensy41TCP_IP.hpp"
#include "DS18B20.hpp"
#include "ADXL345_Accelerometer.hpp"
#include "procElEnEvent.h"
#include "procAzEnEvent.h"
#include <NativeEthernet.h>
#include <queue>
#include <NativeEthernet.h>

#define TempEl1Pin 2
#define TempEl2Pin 3
#define TempAz1Pin 4
#define TempAz2Pin 5
#define AdxlElIntPin 9
#define AdxlAzIntPin 10
#define AdxlCbIntPin 11

#define TCPPORT 1600
#define CRPORT 1680   //Control Room Port
// Create an IntervalTimer object 
IntervalTimer myTimer;

TemperatureSensor tempSensorEl1(TempEl1Pin);
TemperatureSensor tempSensorEl2(TempEl2Pin);
TemperatureSensor tempSensorAz1(TempAz1Pin);
TemperatureSensor tempSensorAz2(TempAz2Pin);
ADXL345 adxlEl = ADXL345(Wire);
ADXL345 adxlAz = ADXL345(Wire1);
ADXL345 adxlCb = ADXL345(Wire2);
ElevationEncoder elencoder = ElevationEncoder();
AzimuthEncoder azencoder = AzimuthEncoder();

//ethernet data
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = { 192, 168, 0, 197 }; // The IP will need reset for every different PC the Teensy is connected to
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);



IPAddress ControlRoomIP = IPAddress(192, 168, 0, 10);


EthernetClient client;
//ethernet server
EthernetServer server(CRPORT);

int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

// Event flags that are set by the Control Room used to Configure and Initialize
bool InitEl1TempFlag;
bool InitEl2TempFlag;
bool InitAz1TempFlag;
bool InitAz2TempFlag;
bool InitElEncoderFlag;
bool InitAzEncoderFlag;
bool InitElAccelFlag;
bool InitAzAccelFlag;
bool InitCbAccelFlag;

// Event flags that are check in the main loop to see what processes should be run
bool EthernetEventFlag = false;
bool TimerEventFlag = false;
bool TempEventFlag = false;
bool ElEncoderEventFlag = false;
bool AZEncoderEventFlag = false;
bool ElAccelEventFlag = true;         // init as true to empty 
bool AzAccelEventFlag = true;         // init as true to empty 
bool CbAccelEventFlag = true;         // init as true to empty 

// counters for each clock driven interrupt
int ethernetcounter = 0;
int tempcounter = 0;
int elencodercounter =0;
int azencoercounter =0;

// Timer interrupt
void TimerEvent_ISR(){
  TimerEventFlag = true;
  
}

/********************* ISR *********************/
/* Look for ADXL Interrupts     */
void ADXLEL_ISR() {
  ElAccelEventFlag = true;
  
}
void ADXLAZ_ISR() {
  AzAccelEventFlag = true;

}
void ADXLCB_ISR() {
  CbAccelEventFlag = true;

}

void setup() {

  
  Serial.begin(9600);

  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  }
  else{
    Serial.println("Hardware found");
  }

  // Connect to the control room TCP Client
  if(client.connect(ControlRoomIP, TCPPORT)){
      Serial.println("Connected to the control room's TCP server.");
      client.write("Send Sensor Configuration");
  }
  else{
      Serial.println("Could not connect to the control room.");
  }

  // start listening for clients
  server.begin();
  Serial.print("ethernet server address:");
  Serial.println(Ethernet.localIP());
  Serial.print("ethernet server port:");
  Serial.println(CRPORT);
  
  // wait for a Control Room client:
  Serial.println("Waiting for client");
  EthernetClient controlRoomClient = server.available();
  
  //Wait till we recieve data from the Control Room
  while(!controlRoomClient){
    delay(1);
    controlRoomClient = server.available();
  }
  Serial.println("Client found");

  size_t bytes = controlRoomClient.available();
  uint8_t *ptr;
  uint8_t data[bytes] = {0};
  ptr = data;
  controlRoomClient.read(ptr, bytes);

  InitEl1TempFlag = data[0] == 0 ? false : true;
  InitEl2TempFlag = data[1] == 0 ? false : true;
  InitAz1TempFlag = data[2] == 0 ? false : true;
  InitAz2TempFlag = data[3] == 0 ? false : true;
  InitElEncoderFlag = data[4] == 0 ? false : true;
  InitAzEncoderFlag = data[5] == 0 ? false : true;
  InitAzAccelFlag = data[6] == 0 ? false : true;
  InitElAccelFlag = data[7] == 0 ? false : true;
  InitCbAccelFlag = data[8] == 0 ? false : true;

  for (int j = 0; j < bytes; j++)
  {
    Serial.print(data[j]);
    Serial.print("  ");
  }
  Serial.println();

  controlRoomClient.write("acknoledge");
  // give time to receive the data
  delay(2);
  // close the connection:
  controlRoomClient.stop();
  
  if(InitElAccelFlag){
    adxlEl.init();                               // initialize an ADXL345 to communicate using I2C
    attachInterrupt(digitalPinToInterrupt(AdxlElIntPin), ADXLEL_ISR, RISING);   // Attach ADXL345 Interrupt
    Serial.println("El Adxl Initialized");
  }

  if(InitAzAccelFlag){
    adxlAz.init();                               // initialize an ADXL345 to communicate using I2C
    attachInterrupt(digitalPinToInterrupt(AdxlAzIntPin), ADXLAZ_ISR, RISING);   // Attach ADXL345 Interrupt
    Serial.println("Az Adxl Initialized");
  }

  if(InitCbAccelFlag){
    adxlCb.init();                               // initialize an ADXL345 to communicate using I2C
    attachInterrupt(digitalPinToInterrupt(AdxlCbIntPin), ADXLCB_ISR, RISING);   // Attach ADXL345 Interrupt
    Serial.println("Cb Adxl Initialized");
  }

  if(InitAzEncoderFlag){
    azencoder.init();                          // initialize azimuth encoder to communicate using SPI
    Serial.println("Az Encoder Initialized");
  }

  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond
  

}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {

  if(TimerEventFlag){

    TimerEventFlag = false; 

    //increment each clock event counter by 1
    tempcounter++;

    if(InitElEncoderFlag){

      elencodercounter++;
    }
    if(InitAzEncoderFlag){

      azencoercounter++;
    }

    ethernetcounter++;

    //check if temp sensors are ready to be read. Read every 1s
    if(tempcounter >= 1000){
      tempcounter = 0;
      TempEventFlag = true;
    }

    //check if elevation encoder is ready to be read. Read every 20ms
    if(elencodercounter >= 20){//TODO: switch to constant
      elencodercounter = 0;
      ElEncoderEventFlag = true;
    }

    //check if azimuth encoder is ready to be read. Read every 20ms
    if(azencoercounter >= 20){
      
      azencoercounter = 0;
      AZEncoderEventFlag = true;
    }

    if(ethernetcounter >= 250){
      //Serial.println("Setting ethernet flag");
      ethernetcounter = 0;
      EthernetEventFlag = true;
      
    }

  }

  if(ElAccelEventFlag){
    
    ElAccelEventFlag = false;

    adxlEl.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port.
  }
  
  if(AzAccelEventFlag){
    
    AzAccelEventFlag = false;
    adxlAz.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port.
  }

  if(CbAccelEventFlag){
    
    CbAccelEventFlag = false;
    adxlCb.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port.
  }

  if(TempEventFlag){
    TempEventFlag = false;

    if(InitEl1TempFlag){

    tempSensorEl1.getTemp();         // gets the temperature and prints it to the serial port. 
    }

    if(InitEl2TempFlag){

    tempSensorEl2.getTemp();         // gets the temperature and prints it to the serial port. 
    }

    if(InitAz1TempFlag){

    tempSensorAz1.getTemp();         // gets the temperature and prints it to the serial port. 
    }

    if(InitAz2TempFlag){

    tempSensorAz2.getTemp();         // gets the temperature and prints it to the serial port. 
    }

  }
  
  if(ElEncoderEventFlag){
    
    ElEncoderEventFlag = false;
    elencoder.procElEnEvent();         
    
  }

  if(AZEncoderEventFlag){
    
    AZEncoderEventFlag = false;
    azencoder.procAzEnEvent();         

  }

  if(EthernetEventFlag){

    EthernetEventFlag = false;
    uint32_t dataSize = calcTransitSize(adxlEl.buffer.size(), adxlAz.buffer.size(), adxlCb.buffer.size(), tempSensorEl1.buffer.size(), tempSensorAz1.buffer.size(),elencoder.buffer.size(),azencoder.buffer.size()); // determine the size of the array that needs to be alocated
    uint8_t *dataToSend;
    dataToSend = (uint8_t *)malloc(dataSize * sizeof(uint8_t)); //malloc needs to be used becaus stack size on the loop task is about 4k so this needs to go on the heap
    
    prepairTransit(dataToSend, dataSize, &adxlEl.buffer, &adxlAz.buffer, &adxlCb.buffer, &tempSensorEl1.buffer, &tempSensorAz1.buffer, &elencoder.buffer, &azencoder.buffer);

    SendDataToControlRoom(dataToSend, dataSize, ControlRoomIP, TCPPORT, client);
    
    free(dataToSend);

  }

}

