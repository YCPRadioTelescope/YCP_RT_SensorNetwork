#include "DS18B20.hpp"
#include <OneWire.h>
#include <queue>

//constructor for temperature sensor 
TemperatureSensor::TemperatureSensor(uint8_t line){
    this->sensor = OneWire(line);
}

int TemperatureSensor::getTemp(){
    byte i;
    byte data[12];
    //float celsius, fahrenheit;
    
    if ( !this->sensor.search(this->addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    this->sensor.reset_search();
    delay(250);
    return 0;
  }

  if (OneWire::crc8(this->addr, 7) != this->addr[7]) {
      //Serial.println("CRC is not valid!");
      return 0;
  }

  this->sensor.reset();
  this->sensor.select(this->addr);
  this->sensor.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  //delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  this->sensor.reset();
  this->sensor.select(this->addr);    
  this->sensor.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = this->sensor.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (this->type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  //celsius = (float)raw / 16.0;
  //fahrenheit = celsius * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit");
  //Serial.println(raw);
  buffer.push(raw);
  return 1;
}