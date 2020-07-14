#include <Arduino.h>
#include <procElEnEvent.h>

//constants
const int elEnPin = 17;
const int fromLow = 49;
const int fromHigh = 449;
const int toLow = 167;
const int toHigh = 1538;

ElevationEncoder::ElevationEncoder(){
    
}

//function will later need to return the angle, can stay void while still printing output

void ElevationEncoder::procElEnEvent(void) {
    //read data from elEn 
    int elEnDigData = analogRead(elEnPin);
    float elEnMapVolt = 0;
    float outputAngle = 0;

    //print digital number for debugging
    Serial.print("Raw Digital Data: ");
    Serial.println(elEnDigData);

    //transform digital data into a usable angle
    //using linear equation to map the output voltage to an angle
    //need to change the output count back to a voltage

    //easier and cleaner way of doing this, use map function to map the count directly to angle
    //can round to nearest angle or to fractions of angles then

    //first map count to output voltage range
    elEnMapVolt = map(elEnDigData, fromLow, fromHigh, toLow, toHigh);
    Serial.println(elEnMapVolt);

    //divide by 100 for the decimal value
    elEnMapVolt = elEnMapVolt/1000;

    //plug into the equation to get angle
    outputAngle = 70.62*elEnMapVolt - 19.79;

    Serial.print("El En Angle: ");
    Serial.println(outputAngle);
}