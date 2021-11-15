
#include <OneWire.h>
#include <Wire.h>
#include "Teensy41TCP_IP.hpp"
#include "DS18B20.hpp"
#include "ADXL345_Accelerometer.hpp"
#include "procElEnEvent.h"
#include "procAzEnEvent.h"
#include "DHT.h"
#include "WatchDog.hpp"
#include <NativeEthernet.h>
#include <queue>
#include <NativeEthernet.h>
#include <TimeLib.h>

#include <iostream>
#include <fstream>

#define TempEl1Pin 0
#define TempEl2Pin 1
#define TempAz1Pin 2
#define TempAz2Pin 3
#define TempElMount 14
#define AdxlElIntPin 7
#define AdxlAzIntPin 8
#define AdxlCbIntPin 9
#define AdxlElPowerPin 32
#define AdxlAzPowerPin 4
#define AdxlCbPowerPin 5
#define LED1 38
#define LED2 37
#define LED3 36
#define LED4 35

#define TCPPORT 1600  
#define CRPORT 1680   //Control Room Port

using namespace std; //temp

// Create an IntervalTimer object 
IntervalTimer myTimer;

WatchDog wdog1; //Watchdog timmer used for resets

// Sensor constructors 
TemperatureSensor tempSensorEl1(TempEl1Pin);
TemperatureSensor tempSensorEl2(TempEl2Pin);
TemperatureSensor tempSensorAz1(TempAz1Pin);
TemperatureSensor tempSensorAz2(TempAz2Pin);
DHT elmounttemp(TempElMount, DHT22);
ADXL345 adxlEl = ADXL345(Wire);
ADXL345 adxlAz = ADXL345(Wire1);
ADXL345 adxlCb = ADXL345(Wire2);
ElevationEncoder elEncoder = ElevationEncoder();
AzimuthEncoder azEncoder = AzimuthEncoder();

//ethernet data
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = { 192, 168, 0, 197 }; // The IP will need reset for every different PC the Teensy is connected to
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);



IPAddress ControlRoomIP = IPAddress(192, 168, 0, 10);


EthernetClient client;
//ethernet server
EthernetServer server(CRPORT);

std::queue <int16_t> tempSensorElBuffer;
std::queue <int16_t> tempSensorAzBuffer;

std::queue <int16_t> emptyBuff;
std::queue <accDump> emptyAccBuff;

// Time for timmer interrupt
int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

bool HeartBeatLED = false;      // Used to see if super loop is running as expected

// Event flags that are set by the Control Room used to configure and initialize sensors
bool InitElTempFlag;
bool InitAzTempFlag;
bool InitElEncoderFlag;
bool InitAzEncoderFlag;
bool InitElAccelFlag;
bool InitAzAccelFlag;
bool InitCbAccelFlag;

// Event flags that are check in the main loop to see what processes should be run
bool ElAccelEventFlag = false;
bool AzAccelEventFlag = false;
bool CbAccelEventFlag = false;

// counters for each clock driven interrupt
int ethernetcounter = 0;
int eltempcounter = 0;
int elmounttempcounter = 0;
int aztempcounter = 0;    
int encodercounter = 0;
static int dhtDataCollectionTimer = 1;

// Keeps track of the UTC ms
uint64_t UTCms = 0;

uint64_t elAccelTimeStamp = 0;
uint64_t azAccelTimeStamp = 0;
uint64_t cbAccelTimeStamp = 0;

// Time threshold for each clock driven interrupt
int aztempoffset = 250;      // Offset so temp sensors don't sample at the same time
int elmounttempoffset = 500; // Offset for elevation DHT22 ambient temperature/humidity sensor
int ethernetthreshold = 250;
int eltempthreshold = 1000;
int aztempthreshold = 1000 + aztempoffset;
int elmounttempthreshold = 1000 + elmounttempoffset;
int encoderthreshold = 20;

// XYZ offsets for accelerometer calibration
byte cbAccelXOffset = -1;
byte cbAccelYOffset = -4;
byte cbAccelZOffset = 4;

// Timer interrupt
void TimerEvent_ISR(){
  //increment each clock event counter by 1
  if(InitElTempFlag){
    eltempcounter++;
  }
  if(InitAzTempFlag){
    aztempcounter++;
  }
  if(InitElEncoderFlag || InitAzEncoderFlag){

    encodercounter++;
  }

  ethernetcounter++;

  elmounttempcounter++;     //TEMPORARY IMPLEMENTATIONS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  // Count the UTC time
  UTCms++;
}

/********************* ISR *********************/
/* Look for ADXL Interrupts     */
void ADXLEL_ISR() {
  ElAccelEventFlag = true;
  elAccelTimeStamp = UTCms;
}
void ADXLAZ_ISR() {
  AzAccelEventFlag = true;
  azAccelTimeStamp = UTCms;
}
void ADXLCB_ISR() {
  CbAccelEventFlag = true;
  cbAccelTimeStamp = UTCms;
}

void setup() {
  
  // Setup LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, LOW);  // Turn LED on to indicate power
  digitalWrite(LED4, HIGH);

  // Setup and start Teensy Ethernet
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  }
  else{
    Serial.println("Hardware found");
  }

  // Keep trying to connect to the control room until a connection is successfully made
  while(!client.connected()) {
    // Connect to the control room TCP Client
    if(client.connect(ControlRoomIP, TCPPORT)){
        Serial.println("Connected to the control room's TCP server.");
        // Request sensor configuration
        client.write("Send Sensor Configuration");
    }
    else{
        Serial.println("Could not connect to the control room.");
    }
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
  digitalWrite(LED4, LOW);  // Turn on LED4 to show connection to Control Room

  //wdog1.init();   // ~1.5s watchdog timeout

  // Read the configuration packet
  size_t bytes = controlRoomClient.available();
  uint8_t *ptr;
  uint8_t data[bytes] = {0};
  ptr = data;
  controlRoomClient.read(ptr, bytes);

  InitElTempFlag = data[0] == 0 ? false : true;
  InitAzTempFlag = data[1] == 0 ? false : true;
  InitElEncoderFlag = data[2] == 0 ? false : true;
  InitAzEncoderFlag = data[3] == 0 ? false : true;
  InitAzAccelFlag = data[4] == 0 ? false : true;
  InitElAccelFlag = data[5] == 0 ? false : true;
  InitCbAccelFlag = data[6] == 0 ? false : true;

  // Send acknoledgement to Control Room
  controlRoomClient.write("acknoledge");
  // give time to receive the data
  delay(2);
  // close the connection:
  controlRoomClient.stop();


  elmounttemp.begin();


  // Setup ADXLs
  pinMode(AdxlElPowerPin, OUTPUT);
  if(InitElAccelFlag){
    digitalWrite(AdxlElPowerPin, HIGH);
    delay(10);
    adxlEl.init();                               // initialize an ADXL345 to communicate using I2C
    Serial.println();
    Serial.println("El Adxl Initialized");
    Serial.println("Starting El Adxl Self-Test");
    if(adxlEl.selfTest()){
      Serial.println("El ADXL Self-Test Passed"); 
    }
    else{
      Serial.println("El ADXL Self-Test Failed"); 
    }
    Serial.println();
    Serial.println("El Configuration-"); 
    adxlEl.printDataFormat();
    adxlEl.printBWRate();
    adxlEl.printFIFO_CTL();
    attachInterrupt(digitalPinToInterrupt(AdxlElIntPin), ADXLEL_ISR, RISING);   // Attach ADXL345 Interrupt
    ElAccelEventFlag = true;         // init as true to empty 
  }
  else{
    digitalWrite(AdxlElPowerPin, LOW);
  }

  pinMode(AdxlAzPowerPin, OUTPUT);
  if(InitAzAccelFlag){
    digitalWrite(AdxlAzPowerPin, HIGH);
    delay(10);
    adxlAz.init();                               // initialize an ADXL345 to communicate using I2C
    Serial.println();
    Serial.println("Az Adxl Initialized");
    Serial.println("Starting Az ADXL Self-Test");
    if(adxlAz.selfTest()){
        Serial.println("Az ADXL Self-Test Passed");
    }
    else{
        Serial.println("Az ADXL Self-Test Failed");
    }
    Serial.println();
    Serial.println("Az Configuration-"); 
    adxlAz.printDataFormat();
    adxlAz.printBWRate();
    adxlAz.printFIFO_CTL();
    attachInterrupt(digitalPinToInterrupt(AdxlAzIntPin), ADXLAZ_ISR, RISING);   // Attach ADXL345 Interrupt
    AzAccelEventFlag = true;         // init as true to empty 
  }
  else{
    digitalWrite(AdxlAzPowerPin, LOW);
  }

  pinMode(AdxlCbPowerPin, OUTPUT);
  if(InitCbAccelFlag){
    digitalWrite(AdxlCbPowerPin, HIGH);
    delay(10);
    adxlCb.init(ADXL345_BW_50, cbAccelXOffset, cbAccelYOffset, cbAccelZOffset); // initialize an ADXL345 to communicate using I2C
    Serial.println();
    Serial.println("Cb Adxl Initialized");
    Serial.println("Starting Cb ADXL Self-Test");
    if(adxlCb.selfTest()){
        Serial.println("Cb ADXL Self-Test Passed");
    }
    else{
        Serial.println("Cb ADXL Self-Test Failed");
    }
    Serial.println();
    Serial.println("Cb Configuration-"); 
    adxlCb.printDataFormat();
    adxlCb.printBWRate();
    adxlCb.printFIFO_CTL();
    attachInterrupt(digitalPinToInterrupt(AdxlCbIntPin), ADXLCB_ISR, RISING);   // Attach ADXL345 Interrupt
    CbAccelEventFlag = true;         // init as true to empty 
  }
  else{
    digitalWrite(AdxlCbPowerPin, LOW);
  }

  if(InitAzEncoderFlag){
    azEncoder.init();                          // initialize azimuth encoder to communicate using SPI
    Serial.println("Az Encoder Initialized");
  }

  // Get the current time in seconds and convert to ms for starting UTC time
  UTCms = now() * 1000;

  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond
  
 // wdog1.feed(); //reset watchdog


}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {
  
  //check if temp sensors are ready to be read. Read every 1s
  if(eltempcounter >= eltempthreshold){

    eltempcounter = 0;
    int16_t elTempData;

    if(tempSensorEl1.status == TEMPERATURE_OK){
      
      // gets the Elvation motor temperature and checks if sensor is good
      elTempData = tempSensorEl1.getTemp();
      if(tempSensorEl1.status == TEMPERATURE_ERROR){
        Serial.println("El Temp Senor 1 Stoped Working");
      }else{
        tempSensorElBuffer.push(elTempData);
      }    

    }

    // if first temp sensor fails then use the second temp sesnor
    if(tempSensorEl1.status == TEMPERATURE_ERROR && tempSensorEl2.status == TEMPERATURE_OK){
      
      // gets the Elvation motor temperature and checks if sensor is good
      elTempData = tempSensorEl2.getTemp();
      if(tempSensorEl2.status == TEMPERATURE_ERROR){ 
        Serial.println("El Temp Senor 2 Stoped Working");
      }else{
        tempSensorElBuffer.push(elTempData);
      }        
    }
  }

  if(aztempcounter >= aztempthreshold){

    aztempcounter = aztempoffset;    //add offset so temps are not samples at the same time
    int16_t azTempData;

    if(tempSensorAz1.status == TEMPERATURE_OK){
      // gets the Azimuth motor temperature and checks if sensor is good
      azTempData = tempSensorAz1.getTemp();
      
      if(tempSensorAz1.status == TEMPERATURE_ERROR){
        Serial.println("Az Temp Senor 1 Stoped Working");
      }else{
        tempSensorAzBuffer.push(azTempData);
      }       
    }
    if(tempSensorAz1.status == TEMPERATURE_ERROR && tempSensorAz2.status == TEMPERATURE_OK){
      // gets the Azimuth motor temperature and checks if sensor is good
      azTempData = tempSensorAz2.getTemp();

      if(tempSensorAz2.status == TEMPERATURE_ERROR){
        Serial.println("Az Temp Senor 2 Stoped Working");
      }else{
        tempSensorAzBuffer.push(azTempData);
      }           
    }

  }

  if(elmounttempcounter >= elmounttempthreshold){
    elmounttempcounter = 0;
    float elMountTempData;
    float elMountHumData;

    elMountTempData = elmounttemp.readTemperature(true);
    elMountHumData = elmounttemp.readHumidity();

    //ofstream dhtTempFile ("azMountTemp.txt");
    //ofstream dhtHumFile ("azMountHum.txt");
    Serial.print("DHT22 Sample Number: ");
    Serial.println(dhtDataCollectionTimer);
    dhtDataCollectionTimer++;
    
    if(isnan(elMountTempData)){
      Serial.println("DHT22 Sensor Error: Temperature cannot be found");
    }
    else if(elMountTempData < -40 | elMountTempData > 257){
      Serial.println("DHT22 Sensor Error: Temperature out of range");
    }
    else{
      Serial.print("Temperature: ");
      Serial.print(elMountTempData);
      Serial.println(" deg F");
    }

    if(isnan(elMountHumData)){
      Serial.println("DHT22 Sensor Error: Humidity cannot be found");
    }
    else if(elMountHumData < 0 | elMountHumData > 100){
      Serial.println("DHT22 Sensor Error: Humidity out of range");
    }
    else{
      Serial.print("Humidity: ");
      Serial.print(elMountHumData);
      Serial.println("%");
    }
  }

  //check if elevation encoder is ready to be read. Read every 20ms
  if(encodercounter >= encoderthreshold){
    encodercounter = 0;
    
    if(InitElEncoderFlag){
      elEncoder.procElEnEvent();
    }
    if(InitAzEncoderFlag){
      azEncoder.procAzEnEvent();         
    }
    //Send only the encoder information to the Control Room
    uint32_t dataSize = calcTransitSize(0, 0, 0, 0, 0, elEncoder.buffer.size(), azEncoder.buffer.size()); // determine the size of the array that needs to be alocated
    uint8_t *dataToSend;
    dataToSend = (uint8_t *)malloc(dataSize * sizeof(uint8_t)); 

    // Sensor status is either okay or errored 
    uint8_t sensorStatus = (adxlEl.status << 7 | adxlAz.status << 6 | adxlCb.status << 5 | 
                            tempSensorEl1.status << 4 | tempSensorEl2.status << 3 | tempSensorAz1.status << 2 | tempSensorAz2.status << 1 | azEncoder.status);
    // Adxl self test plus error codes for temp sensors and Azimuth encoder. First byte has self tests, second has adxl error codes and azimuth error code, third has temp error codes
    uint32_t sensorErrors = (adxlEl.self_test << 18 | adxlAz.self_test << 17 | adxlCb.self_test << 16 | adxlEl.error_code << 14 | adxlAz.error_code << 12 | adxlCb.error_code << 10 | azEncoder.error_code << 8 |
                             tempSensorEl1.error_code << 6 | tempSensorEl2.error_code << 4 | tempSensorAz1.error_code << 2 | tempSensorAz2.error_code);
    // Create packet to send to Control Room
    prepairTransit(dataToSend, dataSize, &emptyAccBuff, &emptyAccBuff, &emptyAccBuff, &emptyBuff, &emptyBuff, &elEncoder.buffer, &azEncoder.buffer, sensorStatus, sensorErrors);
    // Send packet to Control Room
    SendDataToControlRoom(dataToSend, dataSize, ControlRoomIP, TCPPORT, client);
    
    free(dataToSend);
  }

  if(ElAccelEventFlag){
    
    ElAccelEventFlag = false;
    adxlEl.status = ADXL345_OK;    // if we hit the watermark that means the adxl is collecting samples
    adxlEl.error_code = ADXL345_NO_ERROR;
    adxlEl.emptyFifo(elAccelTimeStamp);      // gets the x y and z cordnates and prints them to the serial port.
  }
  
  if(AzAccelEventFlag){
    
    AzAccelEventFlag = false;
    adxlAz.status = ADXL345_OK;    // if we hit the watermark that means the adxl is collecting samples
    adxlAz.error_code = ADXL345_NO_ERROR;
    adxlAz.emptyFifo(azAccelTimeStamp);      // gets the x y and z cordnates and prints them to the serial port.
  }

  if(CbAccelEventFlag){
    
    CbAccelEventFlag = false;
    adxlCb.status = ADXL345_OK;    // if we hit the watermark that means the adxl is collecting samples
    adxlCb.error_code = ADXL345_NO_ERROR;
    adxlCb.emptyFifo(cbAccelTimeStamp);      // gets the x y and z cordnates and prints them to the serial port.
  }

  // Send all sensor data except encoder
  if(ethernetcounter >= ethernetthreshold){
    ethernetcounter = 0;
    
    // Check if ADXLs stopped working and power cycle them if so
    if(adxlEl.buffer.size() == 0 && InitElAccelFlag){
      uint8_t numSamples = adxlEl.getSampleBufSize();
      if(numSamples == 32){
        Serial.println("El ADXL Watermark missed");
        adxlEl.error_code = ADXL345_WATERMARK_MISSED;
        adxlEl.clearAccel(); 
      }
      else if (numSamples == 0){
        Serial.println("No El ADXL Data");
        adxlEl.error_code = ADXL345_NO_SAMPLES;
      }
      adxlEl.status = ADXL345_ERROR;
      
    }
    if(adxlAz.buffer.size() == 0 && InitAzAccelFlag){
      uint8_t numSamples = adxlAz.getSampleBufSize();
      if(numSamples == 32){
        Serial.println("Az ADXL Watermark missed");
        adxlAz.error_code = ADXL345_WATERMARK_MISSED;
        adxlAz.clearAccel(); 
      }
      else if (numSamples == 0){
        Serial.println("No Az ADXL Data");
        adxlAz.error_code = ADXL345_NO_SAMPLES;
      }
      adxlAz.status = ADXL345_ERROR;
    }
    if(adxlCb.buffer.size() == 0 && InitCbAccelFlag){
      uint8_t numSamples = adxlCb.getSampleBufSize();
      if(numSamples == 32){
        Serial.println("Cb ADXL Watermark missed");
        adxlCb.error_code = ADXL345_WATERMARK_MISSED;
        adxlCb.clearAccel(); 
      }
      else if (numSamples == 0){
        Serial.println("No Cb ADXL Data");
        adxlCb.error_code = ADXL345_NO_SAMPLES;
      }
      adxlCb.status = ADXL345_ERROR;
    }

    uint32_t dataSize = calcTransitSize(adxlEl.data_size, adxlAz.data_size, adxlCb.data_size, tempSensorElBuffer.size(), tempSensorAzBuffer.size(),0,0); // determine the size of the array that needs to be alocated
    
    // Clear accelerometer data sizes
    adxlEl.data_size = 0;
    adxlAz.data_size = 0;
    adxlCb.data_size = 0;

    uint8_t *dataToSend;
    dataToSend = (uint8_t *)malloc(dataSize * sizeof(uint8_t));
    
    // Sensor status is either okay or errored 
    uint8_t sensorStatus = (adxlEl.status << 7 | adxlAz.status << 6 | adxlCb.status << 5 | 
                            tempSensorEl1.status << 4 | tempSensorEl2.status << 3 | tempSensorAz1.status << 2 | tempSensorAz2.status << 1 | azEncoder.status);
    // Adxl self test plus error codes for temp sensors and Azimuth encoder. First byte has self tests, second has adxl error codes and azimuth error code, third has temp error codes
    uint32_t sensorErrors = (adxlEl.self_test << 18 | adxlAz.self_test << 17 | adxlCb.self_test << 16 | adxlEl.error_code << 14 | adxlAz.error_code << 12 | adxlCb.error_code << 10 | azEncoder.error_code << 8 |
                             tempSensorEl1.error_code << 6 | tempSensorEl2.error_code << 4 | tempSensorAz1.error_code << 2 | tempSensorAz2.error_code);
    
    prepairTransit(dataToSend, dataSize, &adxlEl.buffer, &adxlAz.buffer, &adxlCb.buffer, &tempSensorElBuffer, &tempSensorAzBuffer, &emptyBuff, &emptyBuff, sensorStatus, sensorErrors);

    // The Control Room has the option to close the server to cause the embeded system to reset
    if(client.connected()){
      SendDataToControlRoom(dataToSend, dataSize, ControlRoomIP, TCPPORT, client);
    }
    else{
      Serial.println("Software Reset");
      SCB_AIRCR = 0x05FA0004;  // does a software reset
    }

    free(dataToSend);

    // Reset temp sensors if both fail. There is a possible that they will eventually recover
    if(tempSensorEl1.status == TEMPERATURE_ERROR && tempSensorEl2.status == TEMPERATURE_ERROR){
      tempSensorEl1.status = TEMPERATURE_OK;
      tempSensorEl2.status = TEMPERATURE_OK;
    }
    if(tempSensorAz1.status == TEMPERATURE_ERROR && tempSensorAz2.status == TEMPERATURE_ERROR){
      tempSensorAz1.status = TEMPERATURE_OK;
      tempSensorAz2.status = TEMPERATURE_OK;
    }

    wdog1.feed(); //reset watchdog
    HeartBeatLED = !HeartBeatLED;
    digitalWrite(LED1, HeartBeatLED);
    // We need this but I'm not sure why it's hanging here :(
    //client.flush();
  }

}

