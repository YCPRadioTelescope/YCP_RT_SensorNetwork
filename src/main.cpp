
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
#include <list>

#include <Arduino.h>
#include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives

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
#define FanControl 15
#define ElEncAvgNumSamp 200
#define LED1 38
#define LED2 37
#define LED3 36
#define LED4 35
#define OTAResetPin 27

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
DHT tempSensorAmb(TempElMount, DHT22);
ADXL345 adxlEl = ADXL345(Wire);
ADXL345 adxlAz = ADXL345(Wire1);
ADXL345 adxlCb = ADXL345(Wire2);
ElevationEncoder elEncoder = ElevationEncoder();
AzimuthEncoder azEncoder = AzimuthEncoder();
int azEncSampleCounter = 0;

//ethernet data
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = { 192, 168, 0, 197 }; // The IP will need reset for every different PC the Teensy is connected to
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);



IPAddress ControlRoomIP = IPAddress(192, 168, 0, 10);


EthernetClient client;
//ethernet server
EthernetServer server(CRPORT);

//size_t bytes;
//uint8_t *ptr;
//uint8_t data[];
EthernetClient controlRoomClient;

std::queue <int16_t> tempSensorElBuffer;
std::queue <int16_t> tempSensorAzBuffer;

std::queue <float> tempAmbBuffer;
std::queue <float> humidityAmbBuffer;

std::queue <int16_t> emptyBuff;
std::queue <accDump> emptyAccBuff;
std::queue <float> emptyDhtBuff;

std::list<int16_t> azEnModeData;

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
bool InitAmbTempFlag;

int TimerPeriod;
int EthernetPeriod;
int TemperaturePeriod;
int EncoderPeriod;

int azModeCounterTemp;

byte ElAccelSamplingFreq;
byte ElAccelGRange;
byte ElAccelFIFOSize;
byte ElAccelXOffset;
byte ElAccelYOffset;
byte ElAccelZOffset;
byte ElAccResolution;

byte AzAccelSamplingFreq;
byte AzAccelGRange;
byte AzAccelFIFOSize;
byte AzAccelXOffset;
byte AzAccelYOffset;
byte AzAccelZOffset;
byte AzAccResolution; 

byte CbAccelSamplingFreq;
byte CbAccelGRange;
byte CbAccelFIFOSize;
byte CbAccelXOffset;
byte CbAccelYOffset;
byte CbAccelZOffset;
byte CbAccResolution;

bool FanCommandFlag = false;

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
int elEncCounter = 0;
int elEncRunAvgFlg = 0;
static int dhtDataCollectionTimer = 1;
static int fanTimeout = 0;

int elEncRunAvg [ElEncAvgNumSamp];

// Keeps track of the ms passed since control room connection
uint64_t connectionTimeElapsed = 0;

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
//byte cbAccelXOffset = -1;
//byte cbAccelYOffset = -4;
//byte cbAccelZOffset = 4;

// OTA communication pins and timer
Stream *serial = &Serial7;	// Serial (USB) or Serial1, Serial2, etc. (UART)
elapsedMillis timer;

//******************************************************************************
// hex_info_t	struct for hex record and hex file info
//******************************************************************************
typedef struct {	// 
  char *data;		// pointer to array allocated elsewhere
  unsigned int addr;	// address in intel hex record
  unsigned int code;	// intel hex record type (0=data, etc.)
  unsigned int num;	// number of data bytes in intel hex record
 
  uint32_t base;	// base address to be added to intel hex 16-bit addr
  uint32_t min;		// min address in hex file
  uint32_t max;		// max address in hex file
  
  int eof;		// set true on intel hex EOF (code = 1)
  int lines;		// number of hex records received  
} hex_info_t;

void read_ascii_line( Stream *serial, char *line, int maxbytes );
int  parse_hex_line( const char *theline, char *bytes,
	unsigned int *addr, unsigned int *num, unsigned int *code );
int  process_hex_record( hex_info_t *hex );
void update_firmware( Stream *serial, uint32_t buffer_addr, uint32_t buffer_size );
void CheckForUserInput();

uint32_t buffer_addr, buffer_size;

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
  if(InitAmbTempFlag){
    elmounttempcounter++;
  }

  fanTimeout++;
  
  // Increment the time elapsed
  connectionTimeElapsed++;
}

/********************* ISR *********************/
/* Look for ADXL Interrupts     */
void ADXLEL_ISR() {
  ElAccelEventFlag = true;
  elAccelTimeStamp = connectionTimeElapsed;
}
void ADXLAZ_ISR() {
  AzAccelEventFlag = true;
  azAccelTimeStamp = connectionTimeElapsed;
}
void ADXLCB_ISR() {
  CbAccelEventFlag = true;
  cbAccelTimeStamp = connectionTimeElapsed;
}

void setup() {
  
  // Setup LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(OTAResetPin, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, LOW);  // Turn LED on to indicate power
  digitalWrite(LED4, HIGH);
  digitalWrite(OTAResetPin, LOW); // Bluetooth module On

  // Setup and start Teensy Ethernet
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  }
  else{
    Serial.println("Hardware found");
  }

  // initialize communication with Bluetooth module
  ((HardwareSerial*)serial)->begin( 115200 );
  serial->printf( "\nFlasherX OTA firmware update v2 %s %s\n", __DATE__, __TIME__ );
  serial->printf( "WARNING: this can ruin your device\n" );
  serial->printf( "target = %s (%dK flash in %dK sectors)\n",
			FLASH_ID, FLASH_SIZE/1024, FLASH_SECTOR_SIZE/1024);

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
        if (Serial7.available()) // test to enter OTA
        {
          CheckForUserInput();
        }
    }
  }

  digitalWrite(OTAResetPin, HIGH); // Bluetooth module Off

  // start listening for clients
  server.begin();
  Serial.print("ethernet server address:");
  Serial.println(Ethernet.localIP());
  Serial.print("ethernet server port:");
  Serial.println(CRPORT);
  
  // wait for a Control Room client:
  Serial.println("Waiting for client");
  controlRoomClient = server.available();
  
  //Wait till we recieve data from the Control Room
  while(!controlRoomClient){
    delay(1);
    controlRoomClient = server.available();
  }
  Serial.println("Client found");
  digitalWrite(LED4, LOW);  // Turn on LED4 to show connection to Control Room

  wdog1.init();   // ~1.5s watchdog timeout

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
  InitAmbTempFlag = data[7] == 0 ? false : true;

  TimerPeriod = ((data[11]) << 24) | ((data[10]) << 16) | ((data[9]) << 8) | (data[8]);
  EthernetPeriod = ((data[15]) << 24) | ((data[14]) << 16) | ((data[13]) << 8) | (data[12]);
  TemperaturePeriod = ((data[19]) << 24) | ((data[18]) << 16) | ((data[17]) << 8) | (data[16]);
  EncoderPeriod = ((data[23]) << 24) | ((data[22]) << 16) | ((data[21]) << 8) | (data[20]);

  ElAccelSamplingFreq = data[24];
  ElAccelGRange = data[25];
  ElAccelFIFOSize = data[26];
  ElAccelXOffset = data[27];
  ElAccelYOffset = data[28];
  ElAccelZOffset = data[29];
  ElAccResolution = data[30];

  AzAccelSamplingFreq = data[31];
  AzAccelGRange = data[32];
  AzAccelFIFOSize = data[33];
  AzAccelXOffset = data[34];
  AzAccelYOffset = data[35];
  AzAccelZOffset = data[36];
  AzAccResolution = data[37]; 

  CbAccelSamplingFreq = data[38];
  CbAccelGRange = data[39];
  CbAccelFIFOSize = data[40];
  CbAccelXOffset = data[41];
  CbAccelYOffset = data[42];
  CbAccelZOffset = data[43];
  CbAccResolution = data[44];

  // Send acknoledgement to Control Room
  controlRoomClient.write("acknoledge");
  // give time to receive the data
  delay(2);
  // close the connection:
  controlRoomClient.stop();


  tempSensorAmb.begin();


  // Setup ADXLs
  pinMode(AdxlElPowerPin, OUTPUT);
  if(InitElAccelFlag){
    digitalWrite(AdxlElPowerPin, HIGH);
    delay(10);
    adxlEl.init(ElAccelSamplingFreq, ElAccelXOffset, ElAccelYOffset, ElAccelZOffset, ElAccelFIFOSize);                                    // initialize an ADXL345 to communicate using I2C
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
    adxlAz.init(AzAccelSamplingFreq, AzAccelXOffset, AzAccelYOffset, AzAccelZOffset, AzAccelFIFOSize);                               // initialize an ADXL345 to communicate using I2C
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
    adxlCb.init(CbAccelSamplingFreq, CbAccelXOffset, CbAccelYOffset, CbAccelZOffset, CbAccelFIFOSize); // initialize an ADXL345 to communicate using I2C
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

  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond

  tempSensorAmb.status = TEMPERATURE_NO_ERROR;
  tempSensorAmb.status = TEMPERATURE_OK;

  pinMode(FanControl, OUTPUT);
  //Temporary initialization to OK for DHT22
  //Will eventually need to implement test at initialization for DHT22
  
  wdog1.feed(); //reset watchdog


}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {
  Serial.println("Top of loop");
  // Read the azimuth absolute encoder every iteration of the superloop. It adds every reading to its queue. The queue will be analyzed when the
  // az encoder timer interrupt happens, in which we'll take the mode of the readings

  if(InitAzEncoderFlag){
    if(azEncSampleCounter < 100){
      azEncoder.procAzEnEvent();
      azEncSampleCounter++;
    }

  }
  
  //Serial.println("Az enc successfully read");
  //int elEnDigData = analogRead(40);

  if(InitElEncoderFlag){
    if(elEncCounter == ElEncAvgNumSamp){
    elEncCounter = 0;
    if(elEncRunAvgFlg == 0){
      elEncRunAvgFlg = 1;
    }
  }
    elEncRunAvg[elEncCounter] = analogRead(40);
    elEncCounter++;
  }

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

    elMountTempData = tempSensorAmb.readTemperature(true);
    elMountHumData = tempSensorAmb.readHumidity();

    //ofstream dhtTempFile ("azMountTemp.txt");
    //ofstream dhtHumFile ("azMountHum.txt");
    //Serial.print("DHT22 Sample Number: ");
    //Serial.println(dhtDataCollectionTimer);
    dhtDataCollectionTimer++;
    
    if(isnan(elMountTempData) || isnan(elMountHumData) ){
      Serial.println("DHT22 Sensor Error: Temperature cannot be found");
      tempSensorAmb.error_code = TEMPERATURE_NO_DATA;
      tempSensorAmb.status = TEMPERATURE_ERROR;
    }
    else if((elMountTempData < -40 || elMountTempData > 257)||(elMountHumData < 0 || elMountHumData > 100)){
      Serial.println("DHT22 Sensor Error: Temperature out of range");
      tempSensorAmb.error_code = TEMPERATURE_OUT_OF_RANGE;
      tempSensorAmb.status = TEMPERATURE_ERROR;
    }
    else{
      tempSensorAmb.error_code = TEMPERATURE_NO_ERROR;
      tempSensorAmb.status = TEMPERATURE_OK;
      tempAmbBuffer.push(elMountTempData);
      humidityAmbBuffer.push(elMountHumData);
    }
  }

  //check if elevation encoder is ready to be read. Read every 20ms
  if(encodercounter >= EncoderPeriod){
    encodercounter = 0;
    
    if(InitElEncoderFlag){
      //elEncoder.procElEnEvent();
      int sum = 0;
      int average;

      if(elEncRunAvgFlg != 0){
        for(int i = 0; i < ElEncAvgNumSamp; i++){
          sum = sum + elEncRunAvg[i];
        }
        average = sum/ElEncAvgNumSamp;
      }
      else{
        for(int i = 0; i < elEncCounter; i++){
          sum = sum + elEncRunAvg[i];
        }
        average = sum/elEncCounter;
      }

      elEncoder.buffer.empty();
      elEncoder.buffer.push(average);

      Serial.print("Average: ");

      Serial.println(average);

    }
    if(InitAzEncoderFlag){
      //azEncoder.procAzEnEvent();
      while(azEncoder.buffer.size() != 0){
          azEnModeData.push_front(azEncoder.buffer.front());
          azEncoder.buffer.pop();
      }

      azEnModeData.sort();

      Serial.println("Az Enc Data List");

      int number = azEnModeData.front();
      int16_t mode = number;
      int count = 1;
      int countMode = 1;

      azEnModeData.erase(azEnModeData.begin());

      static int azEnModeSize = azEnModeData.size();

      for(int i = 0; i < azEnModeSize; i++){
        if (azEnModeData.front() == number){
          count++;
          azEnModeData.erase(azEnModeData.begin());
        }
        else{
          if(count > countMode){
            countMode = count;
            mode = number;
          }
          count = 1;
          number = azEnModeData.front();
          azEnModeData.erase(azEnModeData.begin());
        }
      }
      azEncoder.buffer.empty();
      azEnModeData.empty();

      Serial.print("MODE: ");

      Serial.println(mode);

      azEncoder.buffer.push(mode);
      azEncSampleCounter = 0;
      mode = 0;
    
    

    }



    //Send only the encoder information to the Control Room
    uint32_t dataSize = calcTransitSize(0, 0, adxlCb.data_size, 0, 0, elEncoder.buffer.size(), azEncoder.buffer.size(), 0, 0); // determine the size of the array that needs to be alocated
    uint8_t *dataToSend;
    dataToSend = (uint8_t *)malloc(dataSize * sizeof(uint8_t)); 

    // Sensor status is either okay or errored 
    uint16_t sensorStatus = (FanCommandFlag << 9 | tempSensorAmb.status << 8 | adxlEl.status << 7 | adxlAz.status << 6 | adxlCb.status << 5 | 
                            tempSensorEl1.status << 4 | tempSensorEl2.status << 3 | tempSensorAz1.status << 2 | tempSensorAz2.status << 1 | azEncoder.status);
    // Adxl self test plus error codes for temp sensors and Azimuth encoder. First byte has self tests, second has adxl error codes and azimuth error code, third has temp error codes
    uint32_t sensorErrors = (tempSensorAmb.error_code << 19 | adxlEl.self_test << 18 | adxlAz.self_test << 17 | adxlCb.self_test << 16 | adxlEl.error_code << 14 | adxlAz.error_code << 12 | adxlCb.error_code << 10 | azEncoder.error_code << 8 |
                             tempSensorEl1.error_code << 6 | tempSensorEl2.error_code << 4 | tempSensorAz1.error_code << 2 | tempSensorAz2.error_code);
    // Create packet to send to Control Room
    prepairTransit(dataToSend, dataSize, &emptyAccBuff, &emptyAccBuff, &adxlCb.buffer, &emptyBuff, &emptyBuff, &elEncoder.buffer, &azEncoder.buffer, &emptyDhtBuff, &emptyDhtBuff, sensorStatus, sensorErrors);
    // Send packet to Control Room
    SendDataToControlRoom(dataToSend, dataSize, ControlRoomIP, TCPPORT, client);
    
    size_t bytes = client.available();

    Serial.println("Waiting for client");

    // Wait till we recieve data from the Control Room
    fanTimeout = 0;
    while((!bytes) && (fanTimeout < 500)){
     delay(1);
     bytes = client.available();
    }

    if(fanTimeout < 500){
      Serial.println("Client found");

      // Read the fan control packet
      bytes = client.available();
      uint8_t *ptr;
      uint8_t data[bytes] = {0};
      ptr = data;
      client.read(ptr, bytes);

      FanCommandFlag = data[0] == 0 ? false : true;

      if(FanCommandFlag == true){
        Serial.println("FAN ON");
        digitalWrite(FanControl, HIGH);
      }
      else if(FanCommandFlag == false){
        Serial.println("FAN OLD");
        digitalWrite(FanControl, LOW);
      }

    }
    else{
      digitalWrite(FanControl, LOW);
    }
    Serial.println("about to free data...");
    free(dataToSend);

    adxlCb.data_size = 0;     //BADA BING

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

    uint32_t dataSize = calcTransitSize(adxlEl.data_size, adxlAz.data_size, 0, tempSensorElBuffer.size(), tempSensorAzBuffer.size(),0,0, tempAmbBuffer.size(), humidityAmbBuffer.size()); // determine the size of the array that needs to be alocated
    
    // Clear accelerometer data sizes
    adxlEl.data_size = 0;
    adxlAz.data_size = 0;

    uint8_t *dataToSend;
    dataToSend = (uint8_t *)malloc(dataSize * sizeof(uint8_t));
    
    // Sensor status is either okay or errored 
    uint16_t sensorStatus = (FanCommandFlag << 9 | tempSensorAmb.status << 8 | adxlEl.status << 7 | adxlAz.status << 6 | adxlCb.status << 5 | 
                            tempSensorEl1.status << 4 | tempSensorEl2.status << 3 | tempSensorAz1.status << 2 | tempSensorAz2.status << 1 | azEncoder.status);
    // Adxl self test plus error codes for temp sensors and Azimuth encoder. First byte has self tests, second has adxl error codes and azimuth error code, third has temp error codes
    uint32_t sensorErrors = (tempSensorAmb.error_code << 19 | adxlEl.self_test << 18 | adxlAz.self_test << 17 | adxlCb.self_test << 16 | adxlEl.error_code << 14 | adxlAz.error_code << 12 | adxlCb.error_code << 10 | azEncoder.error_code << 8 |
                             tempSensorEl1.error_code << 6 | tempSensorEl2.error_code << 4 | tempSensorAz1.error_code << 2 | tempSensorAz2.error_code);
    
    prepairTransit(dataToSend, dataSize, &adxlEl.buffer, &adxlAz.buffer, &emptyAccBuff, &tempSensorElBuffer, &tempSensorAzBuffer, &emptyBuff, &emptyBuff, &tempAmbBuffer, &humidityAmbBuffer, sensorStatus, sensorErrors);

    // The Control Room has the option to close the server to cause the embeded system to reset
    if(client.connected()){
      SendDataToControlRoom(dataToSend, dataSize, ControlRoomIP, TCPPORT, client);
    }
    else{
      Serial.println("Software Reset");
      SCB_AIRCR = 0x05FA0004;  // does a software reset
    }


    // wait for a Control Room client:
    Serial.println("Waiting for client");
    size_t bytes = client.available();

    // Wait till we recieve data from the Control Room
    fanTimeout = 0;
    while((!bytes) && (fanTimeout < 500)){
     delay(1);
     bytes = client.available();
    }

    if(fanTimeout < 500){
      Serial.println("Client found");

      // Read the fan control packet
      bytes = client.available();
      uint8_t *ptr;
      uint8_t data[bytes] = {0};
      ptr = data;
      client.read(ptr, bytes);

      FanCommandFlag = data[0] == 0 ? false : true;

      if(FanCommandFlag == true){
        Serial.println("FAN ON");
        digitalWrite(FanControl, HIGH);
      }
      else if(FanCommandFlag == false){
        Serial.println("FAN OLD");
        digitalWrite(FanControl, LOW);
      }

    }
    else{
      digitalWrite(FanControl, LOW);
    }
    Serial.println("about to free data...");
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

// Additional OTA updating methods
void CheckForUserInput()
{
  const int bufflen = 80;
  char buff[bufflen];
  memset(buff, 0, bufflen);
  //const char s[2] = ",";

timer = 0;

  if (Serial7.available() > 0)
  {
    // read the incoming byte:
    int incomingByte = Serial7.read();

    // say what you got:
    Serial7.print("I received: ");
    Serial7.println(incomingByte, HEX); //chg to HEX 02/18/20

    //02/18/20 experiment with multiple commands
    switch (incomingByte)
    {
    case 0x55: //ASCII 'U'
    case 0x75: //ASCII 'u'
      Serial7.println(F("Start Program Update - Send new HEX file!"));


      //09/20/21 copied from FlasherX - loop()
      if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
      {
        Serial7.printf("unable to create buffer\n");
        Serial7.flush();
        for (;;) {}
      }

      Serial7.printf("buffer = %1luK %s (%08lX - %08lX)\n",
        buffer_size / 1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
        buffer_addr, buffer_addr + buffer_size);

      //09/20/21 clear the serial buffer
      while (Serial7.available())
      {
        Serial7.read();
      }

      // receive hex file via serial, write new firmware to flash, clean up, reboot
      update_firmware(&Serial7, buffer_addr, buffer_size); // no return if success

      // return from update_firmware() means error or user abort, so clean up and
      // reboot to ensure that static vars get boot-up initialized before retry
      Serial7.printf("erase FLASH buffer / free RAM buffer...\n");
      firmware_buffer_free(buffer_addr, buffer_size);
      Serial7.flush();
      REBOOT;
      break;
    }
  }
}

//******************************************************************************
// update_firmware()	read hex file and write new firmware to program flash
//******************************************************************************
void update_firmware( Stream *serial, uint32_t buffer_addr, uint32_t buffer_size )
{
  static char line[96];					// buffer for hex lines
  static char data[32] __attribute__ ((aligned (8)));	// buffer for hex data
  hex_info_t hex = {					// intel hex info struct
    data, 0, 0, 0,					//   data,addr,num,code
    0, 0xFFFFFFFF, 0, 					//   base,min,max,
    0, 0						//   eof,lines
  };

  serial->printf( "waiting for hex lines...%d\n",&timer );


  // read and process intel hex lines until EOF or error
  while (!hex.eof)  {

    read_ascii_line( serial, line, sizeof(line) );
    // reliability of transfer via USB is improved by this printf/flush
    if (serial == (Stream*)&Serial) {
      serial->printf( "%s\n", line );
      serial->flush();
    }

    if (parse_hex_line( (const char*)line, hex.data, &hex.addr, &hex.num, &hex.code ) == 0) {
      serial->printf( "abort - bad hex line %s\n", line );
      return;
    }
    else if (process_hex_record( &hex ) != 0) { // error on bad hex code
      serial->printf( "abort - invalid hex code %d\n", hex.code );
      return;
    }
    else if (hex.code == 0) { // if data record
      uint32_t addr = buffer_addr + hex.base + hex.addr - FLASH_BASE_ADDR;
      if (hex.max > (FLASH_BASE_ADDR + buffer_size)) {
        serial->printf( "abort - max address %08lX too large\n", hex.max );
        return;
      }
      else if (!IN_FLASH(buffer_addr)) {
        memcpy( (void*)addr, (void*)hex.data, hex.num );
      }
      else if (IN_FLASH(buffer_addr)) {
        // add
        Serial.printf("addr = %d count = %d\n", addr, hex.num);
        int error = flash_write_block( addr, hex.data, hex.num );
        if (error) {
          serial->printf( "abort - error %02X in flash_write_block()\n", error );
	  return;
        }
      }
    }
    hex.lines++;
  }
    
  serial->printf( "\nhex file: %1d lines %1lu bytes (%08lX - %08lX)\n",
			hex.lines, hex.max-hex.min, hex.min, hex.max );

  // check FSEC value in new code -- abort if incorrect
  #if defined(KINETISK) || defined(KINETISL)
  uint32_t value = *(uint32_t *)(0x40C + buffer_addr);
  if (value == 0xfffff9de) {
    serial->printf( "new code contains correct FSEC value %08lX\n", value );
  }
  else {
    serial->printf( "abort - FSEC value %08lX should be FFFFF9DE\n", value );
    return;
  } 
  #endif

  // check FLASH_ID in new code - abort if not found
  if (check_flash_id( buffer_addr, hex.max - hex.min )) {
    serial->printf( "new code contains correct target ID %s\n", FLASH_ID );
  }
  else {
    serial->printf( "abort - new code missing string %s\n", FLASH_ID );
    return;
  }
  
  // get user input to write to flash or abort
  int user_lines = -1;
  while (user_lines != hex.lines && user_lines != 0) {
    serial->printf( "enter %d to flash or 0 to abort\n", hex.lines );
    read_ascii_line( serial, line, sizeof(line) );
    sscanf( line, "%d", &user_lines );
  }
  
  if (user_lines == 0) {
    serial->printf( "abort - user entered 0 lines\n" );
    return;
  }
  
  // move new program from buffer to flash, free buffer, and reboot
  flash_move( FLASH_BASE_ADDR, buffer_addr, hex.max-hex.min );
  

  // should not return from flash_move(), but put REBOOT here as reminder
  REBOOT;
}

//******************************************************************************
// read_ascii_line()	read ascii characters until '\n', '\r', or max bytes
//******************************************************************************
void read_ascii_line( Stream *serial, char *line, int maxbytes )
{
  int c=0, nchar=0;
  while (nchar < maxbytes && !(c == '\n' || c == '\r')) {
    if (serial->available()) {
      c = serial->read();
      line[nchar++] = c;
    }
  }
  line[nchar-1] = 0;	// null-terminate
}

//******************************************************************************
// process_hex_record()		process record and return okay (0) or error (1)
//******************************************************************************
int process_hex_record( hex_info_t *hex )
{
  if (hex->code==0) { // data -- update min/max address so far
    if (hex->base + hex->addr + hex->num > hex->max)
      hex->max = hex->base + hex->addr + hex->num;
    if (hex->base + hex->addr < hex->min)
      hex->min = hex->base + hex->addr;
  }
  else if (hex->code==1) { // EOF (:flash command not received yet)
    hex->eof = 1;
  }
  else if (hex->code==2) { // extended segment address (top 16 of 24-bit addr)
    hex->base = ((hex->data[0] << 8) | hex->data[1]) << 4;
  }
  else if (hex->code==3) { // start segment address (80x86 real mode only)
    return 1;
  }
  else if (hex->code==4) { // extended linear address (top 16 of 32-bit addr)
    hex->base = ((hex->data[0] << 8) | hex->data[1]) << 16;
  }
  else if (hex->code==5) { // start linear address (32-bit big endian addr)
    hex->base = (hex->data[0] << 24) | (hex->data[1] << 16)
              | (hex->data[2] <<  8) | (hex->data[3] <<  0);
  }
  else {
    return 1;
  }

  return 0;
}

//******************************************************************************
// Intel Hex record foramt:
//
// Start code:  one character, ASCII colon ':'.
// Byte count:  two hex digits, number of bytes (hex digit pairs) in data field.
// Address:     four hex digits
// Record type: two hex digits, 00 to 05, defining the meaning of the data field.
// Data:        n bytes of data represented by 2n hex digits.
// Checksum:    two hex digits, computed value used to verify record has no errors.
//
// Examples:
//  :10 9D30 00 711F0000AD38000005390000F5460000 35
//  :04 9D40 00 01480000 D6
//  :00 0000 01 FF
//******************************************************************************

/* Intel HEX read/write functions, Paul Stoffregen, paul@ece.orst.edu */
/* This code is in the public domain.  Please retain my name and */
/* email address in distributed copies, and let me know about any bugs */

/* I, Paul Stoffregen, give no warranty, expressed or implied for */
/* this software and/or documentation provided, including, without */
/* limitation, warranty of merchantability and fitness for a */
/* particular purpose. */

// type modifications by Jon Zeeff

/* parses a line of intel hex code, stores the data in bytes[] */
/* and the beginning address in addr, and returns a 1 if the */
/* line was valid, or a 0 if an error occured.  The variable */
/* num gets the number of bytes that were stored into bytes[] */

#include <stdio.h>		// sscanf(), etc.
#include <string.h>		// strlen(), etc.

int parse_hex_line( const char *theline, char *bytes, 
		unsigned int *addr, unsigned int *num, unsigned int *code )
{
  unsigned sum, len, cksum;
  const char *ptr;
  int temp;

  *num = 0;
  if (theline[0] != ':')
    return 0;
  if (strlen (theline) < 11)
    return 0;
  ptr = theline + 1;
  if (!sscanf (ptr, "%02x", &len))
    return 0;
  ptr += 2;
  if (strlen (theline) < (11 + (len * 2)))
    return 0;
  if (!sscanf (ptr, "%04x", (unsigned int *)addr))
    return 0;
  ptr += 4;
  // add
  Serial.printf("Line: length=%d Addr=%d\n", len, *addr);
  if (!sscanf (ptr, "%02x", code))
    return 0;
  ptr += 2;
  sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + (*code & 255);
  while (*num != len)
  {
    if (!sscanf (ptr, "%02x", &temp))
      return 0;
    bytes[*num] = temp;
    ptr += 2;
    sum += bytes[*num] & 255;
    (*num)++;
    if (*num >= 256)
      return 0;
  }
  if (!sscanf (ptr, "%02x", &cksum))
    return 0;

  if (((sum & 255) + (cksum & 255)) & 255)
    return 0;     /* checksum error */
  return 1;
}