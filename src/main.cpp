
#include <OneWire.h>
#include "DS18B20.hpp"
#include "ADXL345_Accelerometer.hpp"

#define Temp1Pin 10

// Create an IntervalTimer object 
IntervalTimer myTimer;

TemperatureSensor Tempsensor1(Temp1Pin);
ADXL345 adxl = ADXL345();


/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
int interruptPin = 7;                 // Setup pin 7 to be the interrupt pin

int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

bool TimerEventFlag = false;
bool TempEventFlag = false;
bool EncoderEventFlag = false;
bool AccelEventFlag = true;

int tempcounter = 0;
int encodercounter =0;
int accelcounter =0;    //This is for testing purposes. Will need to be removed

void TimerEvent_ISR(){
  TimerEventFlag = true;
  
}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  AccelEventFlag = true;

}

void setup() {
  Serial.begin(9600);
  adxl.init();                     // setup an accelerometer to communicate using I2C
  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond

  attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt

}

// This is the super loop where we will be keeping track of counters, setting eventflags and calling proccess base on if any event flags were set
void loop() {

  //Serial.println("starting loop");
  if(TimerEventFlag){

    TimerEventFlag = false; 
    tempcounter++;
    encodercounter++;

    //check if temp sensors are ready to be read
    if(tempcounter >= 100){
      
      tempcounter = 0;
      TempEventFlag = true;
    }
  }

  if(AccelEventFlag){
    //Serial.println("reading accel");
    AccelEventFlag = false;
    adxl.emptyFifo();      // gets the x y and z cordnates and prints them to the serial port. TODO: add return so data can be sent to the control room
  }

  if(TempEventFlag){
    
    TempEventFlag = false;
    Tempsensor1.getTemp();         // gets the temperature and prints it to the serial port. TODO: add return so data can be sent to the control room

    
  }
  
}