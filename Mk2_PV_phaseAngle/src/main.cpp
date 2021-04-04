// This is a cut-down version of my stand-alone sketch for diverting suplus 
// PV power to a dump load using a triac.  The original version, which includes 
// more explanation and 'debug' code to support off-line working, may be found at
// http://openenergymonitor.org/emon/node/841
// 
// Further modified to use phase-angle control of the triac rather than
// burst mode
//
//                  Robin Emley (calypso_rae on Open Energy Monitor Forum)
//                  October 2012

/* 
Circuit for monitoring pulses from a supply meter using checkLedStatus():
 
                  ----------------> +5V
                  |
                  /
                  \ 8K2
                  /
                  |
             ---------------------> dig 2
             |       |
        -->  /       |
        -->  \       _
        LDR  /       -  0.01uF
             |       |
             ----------------------> GND
*/ 

#include "Arduino.h"

#define POSITIVE 1
#define NEGATIVE 0
#define ON 0  // for use with trigger device (active low)
#define OFF 1
//#define ON 1  // for use with LED( active high)
//#define OFF 0

byte outputPinForLed = 5;
byte outputPinForTrigger = 9;
byte outputPinForPAcontrol = 8;
byte voltageSensorPin = 0;
byte currentSensorPin = 1;
byte ledDetectorPin = 2;  // digital 
byte ledRepeaterPin = 10;  // digital 

float safetyMargin_watts = 0;  // <<<------ increase for more export
long cycleCount = 0;
int samplesDuringThisMainsCycle = 0;
byte nextStateOfTriac;
float cyclesPerSecond = 50; // use float to ensure accurate maths

long noOfSamplePairs = 0;
byte polarityNow;

boolean triggerNeedsToBeArmed = false;
boolean beyondStartUpPhase = false;

float energyInBucket = 0; // mimics the operation of a meter at the grid connection point.
//float energyInBucket_4trial = 0; // as entered by used for p-a control trials

int capacityOfEnergyBucket = 3600; // 0.001 kWh = 3600 Joules
int sampleV,sampleI;   // voltage & current samples are integers in the ADC's input range 0 - 1023
int lastSampleV;     // stored value from the previous loop (HP filter is for voltage samples only)
float lastFilteredV,filteredV;  //  voltage values after HP-filtering to remove the DC offset

float prevDCoffset;          // <<--- for LPF 
float DCoffset;              // <<--- for LPF 
float cumVdeltasThisCycle;   // <<--- for LPF 
float sampleVminusDC;         // <<--- for LPF
float sampleIminusDC;         // <<--- used with LPF
float lastSampleVminusDC;     // <<--- used with LPF
float sumP;   //  cumulative sum of power calculations within each mains cycle
float PHASECAL; 
float POWERCAL;  // To convert the product of raw V & I samples into Joules.  
float VOLTAGECAL; // To convert raw voltage samples into volts.  Used for determining when
                  // the trigger device can be safely armed

// items for LED monitoring
byte ledState, prevLedState;
boolean ledRecentlyOnFlag = false;
unsigned long ledOnAt;
float energyInBucket_4led = 0;
float energyLevelAtLastLedPulse;   

// items for phase-angle control of triac
boolean firstLoopOfHalfCycle;
boolean phaseAngleTriggerActivated;
unsigned long timeAtStartOfHalfCycleInMicros;
unsigned long firingDelayInMicros;


void setup()
{
  // Serial.begin(230400);
  Serial.begin(500000);
  Serial.setTimeout(20); // for rapid input of data (default is 1000ms)

  pinMode(outputPinForTrigger, OUTPUT); 
  pinMode(outputPinForPAcontrol, OUTPUT); 
  pinMode(outputPinForLed, OUTPUT);  
    
  POWERCAL = 0.042; // Units are Joules per ADC-level squared.  Used for converting the product of 
                    // voltage and current samples into Joules.
                    //    To determine this value, note the rate that the energy bucket's
                    // level increases when a known load is being measured at a convenient
                    // test location (e.g  using a mains extention with the outer cover removed so that 
                    // the current-clamp can fit around just one core.  Adjust POWERCAL so that
                    // 'measured value' = 'expected value' for various loads.  The value of
                    // POWERCAL is not critical as any absolute error will cancel out when 
                    // import and export flows are balanced.  
                    
  VOLTAGECAL = (float)679 / 471; // Units are Volts per ADC-level.
                        // This value is used to determine when the voltage level is suitable for 
                        // arming the external trigger device.  To set this value, note the min and max
                        // numbers that are seen when measuring 240Vac via the voltage sensor, which 
                        // is 678.8V p-t-p.  The range on my setup is 471 meaning that I'm under-reading 
                        // voltage by 471/679.  VOLTAGECAL therefore need to be the inverse of this, i.e.
                        // 679/471 or 1.44

  PHASECAL = 1.0;  // the default or 'do nothing' value 
}


void loop() // each loop is for one pair of V & I measurements
{
  noOfSamplePairs++;              // for stats only
  samplesDuringThisMainsCycle++;  // for power calculation at the start of each mains cycle

  // store values from previous loop
  lastSampleV=sampleV;            // for digital high-pass filter
  lastFilteredV = filteredV;      // for HPF, used to identify the start of each mains cycle
  lastSampleVminusDC = sampleVminusDC;  // for phasecal calculation 
 
// Get the next pair of raw samples.  Because the CT generally adds more phase-advance 
// than the voltage sensor, it makes sense to sample current before voltage
  sampleI = analogRead(currentSensorPin);
  sampleV = analogRead(voltageSensorPin);

  // remove the DC offset from these samples as determined by a low-pass filter
  sampleVminusDC = sampleV - DCoffset;
  sampleIminusDC = sampleI - DCoffset;

  // a high-pass filter is used just for determining the start of each mains cycle  
  filteredV = 0.996*(lastFilteredV+sampleV-lastSampleV);   

  // Establish the polarities of the latest and previous filtered voltage samples
  byte polarityOfLastReading = polarityNow;
  if(filteredV >= 0) 
    polarityNow = POSITIVE;
  else 
    polarityNow = NEGATIVE;


  if (polarityNow == POSITIVE)
  {
    if (polarityOfLastReading != POSITIVE)
    {
      // This is the start of a new mains cycle (just after the +ve going z-c point)
      cycleCount++; // for stats only
      firstLoopOfHalfCycle = true;

//      checkLedStatus(); // a really useful function, but can be commented out if not required

      // update the Low Pass Filter for DC-offset removal
      prevDCoffset = DCoffset;
      DCoffset = prevDCoffset + (0.01 * cumVdeltasThisCycle); 

      //  Calculate the real power of all instantaneous measurements taken during the 
      //  previous mains cycle, and determine the gain (or loss) in energy.
      float realPower = POWERCAL * sumP / (float)samplesDuringThisMainsCycle;
      float realEnergy = realPower / cyclesPerSecond;


      //----------------------------------------------------------------
      // WARNING - Serial statements can interfere with time-critical code, but
      //           they can be really useful for calibration trials!
      // ----------------------------------------------------------------        
      
      if((cycleCount % 100) == 5) // display once per second
      {
//        Serial.print("energy in bucket = "); Serial.println(); 
//        Serial.println(energyInBucket_4led); // has no upper or lower limits
//        energyInBucket_4led = 0; // for calibration purposes only

//        Serial.print("energyInBucket = ");
//        Serial.println(energyInBucket);
//        Serial.print(", firingDelay = ");
        Serial.print(firingDelayInMicros); Serial.println();
//        Serial.print(", samples = ");
//        Serial.println(samplesDuringThisMainsCycle);

      }

      if (beyondStartUpPhase == true)
      {  
        // Providing that the DC-blocking filters have had sufficient time to settle,    
        // add this power contribution to the energy bucket
        energyInBucket += realEnergy;   
        energyInBucket_4led += realEnergy;   

        // Reduce the level in the energy bucket by the specified safety margin.
        // This allows the system to be positively biassed towards export or import
        energyInBucket -= safetyMargin_watts / cyclesPerSecond;       

        // Apply max and min limits to bucket's level
        if (energyInBucket > capacityOfEnergyBucket)
          energyInBucket = capacityOfEnergyBucket;  
        if (energyInBucket < 0)
          energyInBucket = 0;    
      }
      else
      {  
        // wait until the DC-blocking filters have had time to settle
        if(cycleCount > 100) // 100 mains cycles is 2 seconds
          beyondStartUpPhase = true;
      }
      
//      checkForUserInput(); // user can change the energy bucket's level
//      energyInBucket = energyInBucket_4trial; // over-ride the measured value
      
      triggerNeedsToBeArmed = true;   // the trigger is armed every mains cycle
      
      // ********************************************************
      // start of section to support phase-angle control of triac 
      // determines the correct firing delay for a direct-acting trigger

      // never fire if energy level is below lower threshold (zero power)  
      if (energyInBucket <= 1300) { firingDelayInMicros = 99999;}
      else  
      // fire immediately if energy level is above upper threshold (full power)
      if (energyInBucket >= 2300) { firingDelayInMicros = 0;}
      else
      // determine the appropriate firing point for the bucket's level
      // by using either of the following algorithms
      { 
        // simple algorithm (with non-linear power response across the energy range)
//        firingDelayInMicros = 10 * (2300 - energyInBucket); 
        
        // complex algorithm which reflects the non-linear nature of phase-angle control.  
        firingDelayInMicros = (asin((-1 * (energyInBucket - 1800) / 500)) + (PI/2)) * (10000/PI);
        
        // Suppress firing at low energy levels to avoid complications with 
        // logic near the end of each half-cycle of the mains.  
        // This cut-off affects approximately the bottom 5% of the energy range.
        if (firingDelayInMicros > 8500) { firingDelayInMicros = 99999;} // never fire
      }
            
      // end of section to support phase-angle control of triac
      //*******************************************************
     
      // clear the per-cycle accumulators for use in this new mains cycle.  
      sumP = 0;
      samplesDuringThisMainsCycle = 0;
      cumVdeltasThisCycle = 0;
    } // end of processing that is specific to the first +ve Vsample in each new mains cycle
   
    // still processing POSITIVE Vsamples ...
    // this next block is for burst mode control of the triac, the output 
    // pin for its trigger being on digital pin 9
    //
    if (triggerNeedsToBeArmed == true)
    {
      // check to see whether the trigger device can now be reliably armed
      if((sampleVminusDC * VOLTAGECAL) > 50) // 20V min for Motorola trigger
      {
        // It's now safe to arm the trigger.  So ...
        
        // first check the level in the energy bucket to determine whether the 
        // triac should be fired or not at the next opportunity
        //
        if (energyInBucket > (capacityOfEnergyBucket / 2))        
        {
          nextStateOfTriac = ON;  // the external trigger device is active low
          digitalWrite(outputPinForLed, 1);  // active high
        } 
        else
        {
          nextStateOfTriac = OFF; 
          digitalWrite(outputPinForLed, 0);  
        } 
                  
        // then set the Arduino's output pin accordingly, 
        digitalWrite(outputPinForTrigger, nextStateOfTriac);   
        
        // and clear the flag.
        triggerNeedsToBeArmed = false;
        
      }
    }    
  }  // end of processing that is specific to positive Vsamples
  
  else
  {
    if (polarityOfLastReading != NEGATIVE)
      {
        firstLoopOfHalfCycle = true;
      }
  } // end of processing that is specific to positive Vsamples

  
  // Processing for ALL Vsamples, both positive and negative
  //------------------------------------------------------------
   
  // ********************************************************
  // start of section to support phase-angle control of triac 
  // controls the signal for firing the direct-acting trigger. 
  
  unsigned long timeNowInMicros = micros(); // occurs every loop, for consistent timing
  
  if (firstLoopOfHalfCycle == true)
  {
    timeAtStartOfHalfCycleInMicros = timeNowInMicros;
    firstLoopOfHalfCycle = false;
    phaseAngleTriggerActivated = false;
    // Unless dumping full power, release the trigger on the first loop in each 
    // half cycle.  Ensures that trigger can't get stuck 'on'.  
    if (firingDelayInMicros > 100) {     
      digitalWrite(outputPinForPAcontrol, OFF);}
  }
  
  if (phaseAngleTriggerActivated == true)
  {
    // Unless dumping full power, release the trigger on all loops in this 
    // half cycle after the one during which the trigger was set.  
    if (firingDelayInMicros > 100) {     
      digitalWrite(outputPinForPAcontrol, OFF);}
  }
  else
  {  
    if (timeNowInMicros >= (timeAtStartOfHalfCycleInMicros + firingDelayInMicros))
    {
      digitalWrite(outputPinForPAcontrol, ON); 
      phaseAngleTriggerActivated = true;    
    }
  }
  // end of section to support phase-angle control of triac
  //*******************************************************
  
 
  // Apply phase-shift to the voltage waveform to ensure that the system measures a
  // resistive load with a power factor of unity.
  float  phaseShiftedVminusDC = 
                lastSampleVminusDC + PHASECAL * (sampleVminusDC - lastSampleVminusDC);  
  float instP = phaseShiftedVminusDC * sampleIminusDC; //  power contribution for this pair of V&I samples 
  sumP +=instP;     // cumulative power contributions for this mains cycle 

  cumVdeltasThisCycle += (sampleV - DCoffset); // for use with LP filter
} // end of loop()


// helper function, to process LED events:
// can be conveniently called every 20ms, at the start of each mains cycle
void checkLedStatus()
{
#ifdef DEBUG
  ledState = OFF;
#else
  ledState = digitalRead (ledDetectorPin);
#endif

  if (ledState != prevLedState)
  {
    // led has changed state
    if (ledState == ON)
    {
      // led has just gone on
      ledOnAt = millis();
      ledRecentlyOnFlag = true;
    }
    else
    {
      // led has just gone off   
      if (ledRecentlyOnFlag == true)
      {
        ledRecentlyOnFlag = false;        
        Serial.print ("** LED PULSE ** "); // this is a chargeable event  
      }
      else
      {
        Serial.print ("** LED OFF ** "); // 'no longer exporting' is also a chargeable event
      }
      Serial.println(millis()/1000);
      
//      Serial.print (",  energy change = "); 
//      Serial.println((long)(energyInBucket_4led - energyLevelAtLastLedPulse)); // imported energy is -ve
//      Serial.print (" J, energyInBucket_4led = ");  
//      Serial.println ((long)energyInBucket_4led);          
//      energyLevelAtLastLedPulse = energyInBucket_4led; // also applicable to LED OFF events

    }    
  }
  else
  {
    // the LED state has not changed
    if (ledState == ON)
    {
      if (ledRecentlyOnFlag == true)
      { 
        // check to see if the known duration of a pulse has been exceeded    
        unsigned long timeNow = millis();      
        if ((timeNow - ledOnAt) > 50)
        {
          Serial.print ("** LED ON **");  // 'exporting' is a non-chargeable state     
          Serial.print (",  energy in bucket = "); 
          Serial.println((long)(energyInBucket_4led)); 
          ledRecentlyOnFlag = false;   
        }     
      }
    }
  } 

  prevLedState = ledState;   
}

void checkForUserInput()
{
  if (Serial.available() )
  {
    char inbuf[8] = {0,0,0,0,0,0,0,0}; 
    Serial.readBytesUntil('\n', inbuf, 8); 
    float value = atof(inbuf);    
    if ((value == 0) && (inbuf[0] != '0'))
    {
      // invalid input
    }  
    else
    {
//      energyInBucket_4trial = value;
      energyInBucket = value; // override the measured value
    }
  }
}
