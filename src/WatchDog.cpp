#include <Arduino.h>
#include <WatchDog.hpp>

#define WDOG1_WICR_WTIS ((uint16_t)(1<<14))
#define WDOG1_WICR_WIE ((uint16_t)(1<<15))

// Initilize the watchdog timer
void WatchDog::init() {
  WDOG1_WMCR = 0;   // disable power down PDE
  uint8_t wt = 2;  //  ~1.5 sec reset timeout
  WDOG1_WCR |=  (wt << 8) | WDOG_WCR_WDE | WDOG_WCR_WDT | WDOG_WCR_SRE;
}

// Reset the software
void WatchDog::reset() {
   WDOG1_WCR &= ~WDOG_WCR_SRS;
  // SCB_AIRCR = 0x05FA0004;  // does reset too, reported as TOUT
}

// Reset the watchdog timer
void WatchDog::feed() {
  WDOG1_WICR |= WDOG1_WICR_WTIS;
  // feed the dog
  WDOG1_WSR = 0x5555;
  WDOG1_WSR = 0xAAAA;
  //Serial.printf("fed the dog\n");
}