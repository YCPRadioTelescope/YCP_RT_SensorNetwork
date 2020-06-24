#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10

// Create an IntervalTimer object 
IntervalTimer myTimer;

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float Celsius = 0;
float Fahrenheit = 0;

bool TempEventFlag = false;

void SetTempFlag(){
  TempEventFlag = true;
}

void setup() {
  sensors.begin();
  Serial.begin(9600);
  myTimer.begin(SetTempFlag, 1000000);  // SetTempFlag to run every second
}


void loop() {
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