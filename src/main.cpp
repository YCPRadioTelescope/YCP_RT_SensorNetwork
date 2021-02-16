
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
byte ip[] = { 169, 254, 17, 197 }; // The IP will need reset for every different PC the Teensy is connected to
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);



IPAddress ControlRoomIP = IPAddress(169,254,17,226);


EthernetClient client;
//ethernet server
EthernetServer server(TCPPORT);

int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

// Event flags that are check in the main loop to see what processes should be run
bool EthernetEventFlag = false;
bool TimerEventFlag = false;
bool TempEventFlag = false;
bool ElEncoderEventFlag = false;
bool AZEncoderEventFlag = false;
bool AccelElEventFlag = true;         // init as true to empty 
bool AccelAzEventFlag = true;         // init as true to empty 
bool AccelCbEventFlag = true;         // init as true to empty 

// counters for each clock driven interrupt
int ethernetcounter = 0;
int tempcounter = 0;
int elcodercounter =0;
int azencoercounter =0;

// Timer interrupt
void TimerEvent_ISR(){
  TimerEventFlag = true;
  
}

/********************* ISR *********************/
/* Look for ADXL Interrupts     */
void ADXLEL_ISR() {
  AccelElEventFlag = true;
  
}
void ADXLAZ_ISR() {
  AccelAzEventFlag = true;

}
void ADXLCB_ISR() {
  AccelCbEventFlag = true;

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
  
  adxlEl.init();                               // initialize an ADXL345 to communicate using I2C
  adxlAz.init();                               // initialize an ADXL345 to communicate using I2C
  adxlCb.init();                               // initialize an ADXL345 to communicate using I2C
  azencoder.init();                          // initialize azimuth encoder to communicate using SPI
  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond
  attachInterrupt(digitalPinToInterrupt(AdxlElIntPin), ADXLEL_ISR, RISING);   // Attach ADXL345 Interrupt
  attachInterrupt(digitalPinToInterrupt(AdxlAzIntPin), ADXLAZ_ISR, RISING);   // Attach ADXL345 Interrupt
  attachInterrupt(digitalPinToInterrupt(AdxlCbIntPin), ADXLCB_ISR, RISING);   // Attach ADXL345 Interrupt
  


  // start listening for clients
    server.begin();
    Serial.print("ethernet server address:");
    Serial.println(Ethernet.localIP());
    Serial.print("ethernet server port:");
    Serial.println(TCPPORT);

  // Connect to the control room TCP Client
    if(client.connect(ControlRoomIP, TCPPORT)){
        Serial.println("Connected to the control room's TCP server.");
    }
    else{
        Serial.println("Could not connect to the control room.");
    }
}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {

  if(TimerEventFlag){

    TimerEventFlag = false; 

    //increment each clock event counter by 1
    tempcounter++;
    elcodercounter++;
    azencoercounter++;
    ethernetcounter++;

    //check if temp sensors are ready to be read. Read every 1s
    if(tempcounter >= 1000){
      tempcounter = 0;
      TempEventFlag = true;
    }

    //check if elevation encoder is ready to be read. Read every 20ms
    if(elcodercounter >= 20){//TODO: switch to constant
      elcodercounter = 0;
      ElEncoderEventFlag = true;
    }

    //check if azimuth encoder is ready to be read. Read every 20ms
    if(azencoercounter >= 20){
      
      azencoercounter = 0;
      AZEncoderEventFlag = true;
    }

    if(ethernetcounter >= 1000){
      //Serial.println("Setting ethernet flag");
      ethernetcounter = 0;
      EthernetEventFlag = true;
      
    }

  }

  if(AccelElEventFlag){
    
    AccelElEventFlag = false;

    adxlEl.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }
  
  if(AccelAzEventFlag){
    
    AccelAzEventFlag = false;
    adxlAz.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }

  if(AccelCbEventFlag){
    
    AccelCbEventFlag = false;
    adxlCb.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }

  if(TempEventFlag){
    TempEventFlag = false;
    tempSensorEl1.getTemp();         // gets the temperature and prints it to the serial port. TODO: add return so data can be sent to the control room
    //tempSensorEl2.getTemp();
    tempSensorAz1.getTemp();
    //tempSensorAz2.getTemp();
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
    uint32_t dataSize = calcTransitSize(adxlEl.buffer.size(), adxlAz.buffer.size(), adxlCb.buffer.size(), tempSensorEl1.buffer.size(), tempSensorAz2.buffer.size(),elencoder.buffer.size(),azencoder.buffer.size()); // determine the size of the array that needs to be alocated
    uint8_t *dataToSend;
    dataToSend = (uint8_t *)malloc(dataSize * sizeof(uint8_t)); //malloc needs to be used becaus stack size on the loop task is about 4k so this needs to go on the heap
    
    prepairTransit(dataToSend, dataSize, &adxlEl.buffer, &adxlAz.buffer, &adxlCb.buffer, &tempSensorEl1.buffer, &tempSensorAz1.buffer, &elencoder.buffer, &azencoder.buffer);

    SendDataToControlRoom(dataToSend, dataSize, ControlRoomIP, TCPPORT, client);
    
    free(dataToSend);
  }

}

