#include <SPI.h>
#include <procAzEnEvent.h>

//constants
const int zeroReset = 21;       //pin for the zeroReset wire
const int zeroSet = 20;         //pin for the zeroSet wire

AzimuthEncoder::AzimuthEncoder(){
    
}


void AzimuthEncoder::init() {
  //set pin mode for zero pins
  pinMode (zeroReset, OUTPUT);
  pinMode (zeroSet, OUTPUT);
  
  //initilize the spi bus 0
  SPI.begin();
  //give time to do so
  delay(100);
}

void AzimuthEncoder::procAzEnEvent() {
  //create and initilze variables and arrays
  byte buff[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint16_t angleRaw = 0;               //using a long to be safe, max output data is 22 bits but we only have 11 bits of resolution
  //float angleFinal = 0.0;
  bool dataGood = false;
  bool dataZero = false;
  bool dataValid = false;
  bool dataSync = false;
  bool dataStale = false;

  //int the spi bus at 200kHz with MSB first and SPI_MODE 1
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));

  //transfer data over spi bus
  SPI.transfer(buff, 6);

  //end the SPI communications
  SPI.endTransaction();  

  //print the raw data collected for debugging
  /*
  Serial.print("Raw Data: ");
  for(int i=0; i<6; i++) {
  Serial.print(buff[i], HEX);
  Serial.print(" ");
  }
  Serial.println();
  */
  
  //first want to check what the flags are to ensure data is valid
  //check that first 15 bits are 0s
  if (buff[0] == 0) {
      dataGood = false;
      //Serial.println("here false");
  } else {
      dataGood = true;
      //Serial.println("here true");
  }
  if (buff[1] >> 1 == 0) {
      dataGood = false;
      //Serial.println("here false");
  } else {
      dataGood = true;
      //Serial.println("here true");
  }
  if (dataGood == true) {
      //Serial.println("DATA IN 0 BITS, CORRUPT DATA");
  }

  //check zero flag
  dataZero = buff[1]  & 0x01;
  if (dataZero == false) {
      //Serial.println("NOT AT FACTORY ZERO");
  }

  //check valid flag
  dataValid = buff[2] >> 7;
  if (dataValid == false) {
      //Serial.println("BAD DATA, NOT VALID");
  }

  //check sync flag
  dataSync = (buff[2] >> 6) & 0x01;
  if (dataSync == false) {
      //Serial.println("MEASUREMENT TIMEOUT");
      //not sure why this is set low, ignore as data seems accurate
  }

  dataStale = buff[5] >> 7;
  if (dataStale == true) {
      //Serial.println("STALE DATA");
  }
  
  //data is good
  //now extract the raw angle data
  //potential change, only need the last 11 bits of resolution, for debugging, want to operate on the entire set
  buff[2] = buff[2] & 0x3F;
  angleRaw = (buff[2] << 16) + (buff[3] << 8) + buff[4];

  buffer.push(angleRaw);
  //print raw angle for debugging
  //Serial.print("Raw angle: ");
  //Serial.println(angleRaw);

  //map the digital count to an angle based off a scaling function, decimal is result of 360/2047
  //angleFinal = .175867 * angleRaw;

  //print final angle
  //Serial.print("Az angle: ");
  //Serial.println(angleFinal);
}

