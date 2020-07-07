#include <OneWire.h>
//#include <DallasTemperature.h>
#include "DS18B20.hpp"
#include "ADXL345_Accelerometer.hpp"

#define Temp1Pin 10

// Create an IntervalTimer object 
IntervalTimer myTimer;

TemperatureSensor Tempsensor1(Temp1Pin);
Accelerometer Accelerometer1;

int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

bool TimerEventFlag = false;
bool TempEventFlag = false;
bool EncoderEventFlag = false;
bool AccelEventFlag = false;

int tempcounter = 0;
int encodercounter =0;
int accelcounter =0;    //This is for testing purposes. Will need to be removed

void TimerEvent_ISR(){
  TimerEventFlag = true;
  
}

void setup() {
  Serial.begin(9600);
  Accelerometer1.init();                     // setup an accelerometer to communicate using I2C
  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond
}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {

  if(TimerEventFlag){

    TimerEventFlag = false; 
    tempcounter++;
    encodercounter++;

    accelcounter++;         //TODO: remove

    // TODO: use interupts from accelerometer to set flag
    if(accelcounter >= 10000){
      accelcounter =0;
      AccelEventFlag = true;
    }

    //check if temp sensors are ready to be read
    if(tempcounter >= 100){

      tempcounter = 0;
      TempEventFlag = true;
    }
  }

  if(AccelEventFlag){
    AccelEventFlag = false;
    Accelerometer1.getCords();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }

  if(TempEventFlag){

    TempEventFlag = false;

    //testing testing

    Tempsensor1.getTemp();         // gets the temperature and prints it to the serial port. TODO: add return so data can be sent to the control room

    
  }
}