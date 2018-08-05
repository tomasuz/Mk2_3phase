#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-08-05 17:16:55

#include "Arduino.h"
#include "Arduino.h"

void setup() ;
ISR(ADC_vect) ;
void loop() ;
void calculateEnergyInBucket(unsigned char phase, float realEnergy) ;
void calculateFiringDelay(byte phase) ;
void phaseAngleTriacControl() ;
void processSamplePair(byte phase) ;
void controlPhaseSwichRellay(unsigned char phase, float realPower) ;
void checkLedStatus() ;

#include "Mk2_PV_phaseAngle.ino"


#endif
