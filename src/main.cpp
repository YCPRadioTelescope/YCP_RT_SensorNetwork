
#include <OneWire.h>
#include <Wire.h>
#include "DS18B20.hpp"
#include "ADXL345_Accelerometer.hpp"
#include "procElEnEvent.h"
#include "procAzEnEvent.h"

#define Temp1Pin 10
#define Adxl0IntPin 7
#define Adxl1IntPin 8
#define Adxl2IntPin 9
// Create an IntervalTimer object 
IntervalTimer myTimer;

TemperatureSensor tempsensor1(Temp1Pin);
ADXL345 adxl0 = ADXL345(Wire);
ADXL345 adxl1 = ADXL345(Wire1);
ADXL345 adxl2 = ADXL345(Wire2);
ElevationEncoder elencoder = ElevationEncoder();
AzimuthEncoder azencoder = AzimuthEncoder();

int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

// Event flags that are check in the main loop to see what processes should be run
bool TimerEventFlag = false;
bool TempEventFlag = false;
bool ElEncoderEventFlag = false;
bool AZEncoderEventFlag = false;
bool Accel0EventFlag = true;         // init as true to empty 
bool Accel1EventFlag = true;         // init as true to empty 
bool Accel2EventFlag = true;         // init as true to empty 

// counters for each clock driven interrupt
int tempcounter = 0;
int elcodercounter =0;
int azencoercounter =0;
int accelcounter =0;    //This is for testing purposes. Will need to be removed

// Timer interrupt
void TimerEvent_ISR(){
  TimerEventFlag = true;
  
}

/********************* ISR *********************/
/* Look for ADXL Interrupts     */
void ADXL0_ISR() {
  Accel0EventFlag = true;

}
void ADXL1_ISR() {
  Accel1EventFlag = true;

}
void ADXL2_ISR() {
  Accel2EventFlag = true;

}

void setup() {
  Serial.begin(9600);
  adxl0.init();                               // initialize an ADXL345 to communicate using I2C
  adxl1.init();                               // initialize an ADXL345 to communicate using I2C
  adxl2.init();                               // initialize an ADXL345 to communicate using I2C
  azencoder.init();                          // initialize azimuth encoder to communicate using SPI
  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond

  attachInterrupt(digitalPinToInterrupt(Adxl0IntPin), ADXL0_ISR, RISING);   // Attach ADXL345 Interrupt
  attachInterrupt(digitalPinToInterrupt(Adxl1IntPin), ADXL1_ISR, RISING);   // Attach ADXL345 Interrupt
  attachInterrupt(digitalPinToInterrupt(Adxl2IntPin), ADXL2_ISR, RISING);   // Attach ADXL345 Interrupt

}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {

  if(TimerEventFlag){

    TimerEventFlag = false; 

    //increment each clock event counter by 1
    tempcounter++;
    elcodercounter++;
    azencoercounter++;

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

  }

  if(Accel0EventFlag){
    
    Accel0EventFlag = false;
    adxl0.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }
  
  if(Accel1EventFlag){
    
    Accel1EventFlag = false;
    adxl1.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }

  if(Accel2EventFlag){
    
    Accel2EventFlag = false;
    adxl2.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }

  if(TempEventFlag){
    
    TempEventFlag = false;
    tempsensor1.getTemp();         // gets the temperature and prints it to the serial port. TODO: add return so data can be sent to the control room
    
  }
  
  if(ElEncoderEventFlag){
    
    ElEncoderEventFlag = false;
    elencoder.procElEnEvent();         
    
  }

  if(AZEncoderEventFlag){
    
    AZEncoderEventFlag = false;
    azencoder.procAzEnEvent();         

  }

}