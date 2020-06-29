#include <OneWire.h>
//#include <DallasTemperature.h>
#include "DS18B20.hpp"

#define Temp1Pin 10

// Create an IntervalTimer object 
IntervalTimer myTimer;

TemperatureSensor Tempsensor1(Temp1Pin);

int const TIMER_1MS = 1000;

float Celsius = 0;
float Fahrenheit = 0;

bool TimerEventFlag = false;
bool TempEventFlag = false;
bool EncoderEventFlag = false;

int tempcounter = 0;
int encodercounter =0;
//TODO: implement other clock events
void TimerEvent_ISR(){
  TimerEventFlag = true;
  
}

void setup() {
  Serial.begin(9600);
  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond
}


void loop() {

  if(TimerEventFlag){

    TimerEventFlag = false;
    tempcounter++;
    encodercounter++;

    //check if timer is over 1 millisecond
    if(tempcounter >= 1){

      tempcounter = 0;
      TempEventFlag = true;
    }
  }
  if(TempEventFlag){
    
    TempEventFlag = false;
    Tempsensor1.getTemp();
    
    /*
    sensors.requestTemperatures();
  
    Celsius = sensors.getTempCByIndex(0);
    Fahrenheit = sensors.toFahrenheit(Celsius);
  
    Serial.print(Celsius);
    Serial.print(" C  ");
    Serial.print(Fahrenheit);
    Serial.println(" F");
    */
  }
}