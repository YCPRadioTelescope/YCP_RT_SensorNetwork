#include <Arduino.h>
#include <procElEnEvent.h>

//constants
const int elEnPin = 17;

ElevationEncoder::ElevationEncoder(){
    
}

//function will later need to return the angle, can stay void while still printing output

void ElevationEncoder::procElEnEvent(void) {
   //declare a variable for the final angle
    float outputAngle = 0; 

    //read data from elEn 
    int elEnDigData = analogRead(elEnPin);
    
    //print digital number for debugging
    //Serial.print("Raw Digital Data: ");
    //Serial.println(elEnDigData);

    

    //equation translate the digital count to an angle, 49-50 means -8 degrees and 449-450 means 92 degrees
    outputAngle = .25 * elEnDigData - 20.375;

    //print the angle translated for debugging
    Serial.print("El En Angle: ");
    Serial.println(outputAngle);

    //all code below is legacy, first attempt at mapping 
/*

    const int fromLow = 49;
    const int fromHigh = 449;
    const int toLow = 167;
    const int toHigh = 1538;

    float elEnMapVolt = 0;

    //transform digital data into a usable angle
    //using linear equation to map the output voltage to an angle
    //need to change the output count back to a voltage

    //first map count to output voltage range
    elEnMapVolt = map(elEnDigData, fromLow, fromHigh, toLow, toHigh);
    Serial.println(elEnMapVolt);

    //divide by 100 for the decimal value
    elEnMapVolt = elEnMapVolt/1000;
*/
}