#include <Wire.h>  // Wire library - used for I2C communication
#include "ADXL345_Accelerometer.hpp"
#define accel_module (0x53)


Accelerometer::Accelerometer(){

}


int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out;  // Outputs
int Accelerometer::init(){
    Wire.begin();
    //Serial.begin(9600);

    Wire.beginTransmission(accel_module);
    Wire.write(0x2D);
    Wire.write(0);
    Wire.write(16);
    Wire.endTransmission();
    Wire.beginTransmission(accel_module);
    Wire.write(0x2D);
    Wire.write(8);
    Wire.endTransmission();

    return 0;
}
int Accelerometer::getCords() {
  int xyzregister = 0x32;
  int x,y,z;
  Wire.beginTransmission(accel_module);
  Wire.write(xyzregister);
  Wire.endTransmission();

  Wire.beginTransmission(accel_module);
  Wire.requestFrom(accel_module,6);

  int i =0;
  while(Wire.available()){
    values[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();

  x = (((int)values[1]) << 8) | values[0];
  y = (((int)values[3]) << 8) | values[2];
  z = (((int)values[5]) << 8) | values[4];

  sprintf(output, "%d %d %d", x, y, z);
  Serial.print(output);
  Serial.write(10);

  return 0;
  //delay(1000);
}