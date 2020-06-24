#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10

// Create an IntervalTimer object 
IntervalTimer myTimer;

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

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
  sensors.begin();
  Serial.begin(9600);
  myTimer.begin(TimerEvent_ISR, TIMER_1MS);  // TimerEvent to run every millisecond
}


void loop() {

  if(TimerEventFlag){
    
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
    sensors.requestTemperatures();
  
    Celsius = sensors.getTempCByIndex(0);
    Fahrenheit = sensors.toFahrenheit(Celsius);
  
    Serial.print(Celsius);
    Serial.print(" C  ");
    Serial.print(Fahrenheit);
    Serial.println(" F");
  }
}