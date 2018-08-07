#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-08-07 23:08:08

#include "Arduino.h"
#include "Arduino.h"

void setup() ;
ISR(ADC_vect) ;
void loop() ;
void calculateOfsetsAndEnergy(unsigned char phase) ;
void printDebug(unsigned char phase) ;
void calculateEnergyInBucket(unsigned char phase) ;
void calculateFiringDelay(byte phase) ;
void phaseAngleTriacControl() ;
void processSamplePair(byte phase) ;
void controlPhaseSwichRellay(unsigned char phase) ;

#include "Mk2_PV_phaseAngle.ino"


#endif
