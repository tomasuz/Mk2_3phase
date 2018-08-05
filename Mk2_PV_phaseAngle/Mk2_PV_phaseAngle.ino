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

#define REQUIRED_EXPORT_IN_WATTS 0 // when set to a negative value, this acts as a PV generator
#define NO_OF_PHASES 3

//  WORKLOAD_CHECK is available for determining how much spare processing time there
//  is.  To activate this mode, the #define line below should be included:
// #define WORKLOAD_CHECK

// const byte noOfDumploads = 3; 
const byte noOfDumploads = 1;

enum loadPriorityModes {LOAD_1_HAS_PRIORITY, LOAD_0_HAS_PRIORITY};
// enum loadPriorityModes {LOAD_0_HAS_PRIORITY};

// enum loadStates {LOAD_ON, LOAD_OFF}; // for use if loads are active low (original PCB)
enum loadStates {LOAD_OFF, LOAD_ON}; // for use if loads are active high (Rev 2 PCB)
enum loadStates logicalLoadState[noOfDumploads]; 
enum loadStates physicalLoadState[noOfDumploads]; 

// array defining to which phase which load is connected. 
const byte loadPhases[noOfDumploads] = {0};

// ----------- Pinout assignments  -----------

byte outputPinForLed = 13;
byte outputPinForTrigger = 5;
byte outputPinForPAcontrol[noOfDumploads] = {10};
// byte voltageSensorPin = 2;
// byte currentSensorPin = 1;
byte ledDetectorPin = 2;  // digital
byte ledRepeaterPin = 10;  // digital

// analogue input pins
// const byte sensorV[NO_OF_PHASES] = {B00000000, B00000010, B00000100}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001, B00000011, B00000101}; // for 3-phase PCB
const byte sensorV[3] = {B00000000, B00000010, B00000100}; // for 3-phase PCB
const byte sensorI[3] = {B00000001, B00000011, B00000101}; // for 3-phase PCB
// 1 phase:
// const byte sensorV[NO_OF_PHASES] = {B00000000}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001}; // for 3-phase PCB
// 2 phase
// const byte sensorV[NO_OF_PHASES] = {B00000000, B00000010}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001, B00000011}; // for 3-phase PCB


float safetyMargin_watts = 0;  // <<<------ increase for more export
long cycleCount[NO_OF_PHASES];
long loopCount = 0;
// long samplesindex = 0;

// int samplesDuringThisMainsCycle[NO_OF_PHASES] = {0,0,0};
// Initial values setting moved to setup().....
int samplesDuringThisMainsCycle[NO_OF_PHASES];
byte nextStateOfTriac;
float cyclesPerSecond = 50; // use float to ensure accurate maths

long noOfSamplePairs = 0;
byte polarityNow[NO_OF_PHASES];

// boolean triggerNeedsToBeArmed = false;
// boolean beyondStartUpPhase = false;

float energyInBucket = 0; // mimics the operation of a meter at the grid connection point.
//float energyInBucket_4trial = 0; // as entered by used for p-a control trials

int capacityOfEnergyBucket = 3600; // 0.001 kWh = 3600 Joules
// int sampleV,sampleI;   // voltage & current samples are integers in the ADC's input range 0 - 1023
int lastSampleV[NO_OF_PHASES];     // stored value from the previous loop (HP filter is for voltage samples only)
float lastFilteredV[NO_OF_PHASES], filteredV[NO_OF_PHASES]; //  voltage values after HP-filtering to remove the DC offset

// int lastSampleI[NO_OF_PHASES];     // stored value from the previous loop (HP filter is for voltage samples only)
// float lastFilteredI[NO_OF_PHASES], filteredI[NO_OF_PHASES]; //  voltage values after HP-filtering to remove the DC offset

float prevVDCoffset[NO_OF_PHASES];          // <<--- for LPF
float VDCoffset[NO_OF_PHASES];              // <<--- for LPF
float cumVdeltasThisCycle[NO_OF_PHASES];   // <<--- for LPF

float prevIDCoffset[NO_OF_PHASES];          // <<--- for LPF
float IDCoffset[NO_OF_PHASES];              // <<--- for LPF
float cumIdeltasThisCycle[NO_OF_PHASES];   // <<--- for LPF

float sampleVminusDC[NO_OF_PHASES];         // <<--- for LPF
float sampleIminusDC[NO_OF_PHASES];         // <<--- used with LPF
float lastSampleVminusDC[NO_OF_PHASES];     // <<--- used with LPF
// float lastSampleIminusDC[NO_OF_PHASES];     // <<--- used with LPF
float sumP[NO_OF_PHASES];   //  cumulative sum of power calculations within each mains cycle
float PHASECAL;
float POWERCAL;  // To convert the product of raw V & I samples into Joules.
float VOLTAGECAL; // To convert raw voltage samples into volts.  Used for determining when
// the trigger device can be safely armed


// for interaction between the main processor and the ISR
volatile byte dataReadyForPhase = 3; // Use byte for data ready from ADC and store phase to it. 3 - no data ready.
volatile int sampleV[NO_OF_PHASES];
volatile int sampleI[NO_OF_PHASES];


// Calibration values
//-------------------
// Three calibration values are used in this sketch: powerCal, phaseCal and voltageCal.
// With most hardware, the default values are likely to work fine without
// need for change.  A compact explanation of each of these values now follows:

// When calculating real power, which is what this code does, the individual
// conversion rates for voltage and current are not of importance.  It is
// only the conversion rate for POWER which is important.  This is the
// product of the individual conversion rates for voltage and current.  It
// therefore has the units of ADC-steps squared per Watt.  Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).
//
// powerCal is the RECIPR0CAL of the power conversion rate.  A good value
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)
//
// const float powerCal[NO_OF_PHASES] = {0.043, 0.043, 0.043};
// Initial values setting moved to setup().....
float powerCal[NO_OF_PHASES];

// phaseCal is used to alter the phase of the voltage waveform relative to the
// current waveform.  The algorithm interpolates between the most recent pair
// of voltage samples according to the value of phaseCal.
//
//    With phaseCal = 1, the most recent sample is used.
//    With phaseCal = 0, the previous sample is used
//    With phaseCal = 0.5, the mid-point (average) value in used
//
// NB. Any tool which determines the optimal value of phaseCal must have a similar
// scheme for taking sample values as does this sketch.
//
// Initial values seting moved to setup().....
float  phaseCal[NO_OF_PHASES];
// const float  phaseCal[NO_OF_PHASES] = {0.5, 0.5, 0.5}; // <- nominal values only
int phaseCal_int[NO_OF_PHASES];           // to avoid the need for floating-point maths

// For datalogging purposes, voltageCal has been added too.  Because the range of ADC values is
// similar to the actual range of volts, the optimal value for this cal factor is likely to be
// close to unity.
// Initial values seting moved to setup().....
// const float voltageCal[NO_OF_PHASES] = {1.03, 1.03, 1.03}; // compared with Fluke 77 meter
float voltageCal[NO_OF_PHASES]; // compared with Fluke 77 meter

// items for LED monitoring
byte ledState, prevLedState;
boolean ledRecentlyOnFlag = false;
unsigned long ledOnAt;
float energyInBucket_4led = 0;
float energyLevelAtLastLedPulse;

// items for phase-angle control of triac
boolean firstLoopOfHalfCycle[NO_OF_PHASES];
boolean phaseAngleTriggerActivated[noOfDumploads];
unsigned long timeAtStartOfHalfCycleInMicros[NO_OF_PHASES];
unsigned long firingDelayInMicros[NO_OF_PHASES];

// byte mux[6];
// int samplesV[NO_OF_PHASES][50];
int samplesI[NO_OF_PHASES][50];
unsigned long firingDelaysInMicros[NO_OF_PHASES][50];

void setup()
{
  //  Serial.begin(9600);
  // Serial.begin(230400);
  Serial.begin(115200);
//  Serial.begin(500000);
  Serial.setTimeout(20); // for rapid input of data (default is 1000ms)

  pinMode(outputPinForTrigger, OUTPUT);

  for (byte load = 0; load < noOfDumploads; load++) {
    pinMode(outputPinForPAcontrol[load], OUTPUT);
  }
  pinMode(outputPinForLed, OUTPUT);

  for(int i = 0; i< noOfDumploads; i++) {
    logicalLoadState[i] = LOAD_OFF;
  }

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

  Serial.println ("ADC mode:       free-running");
  Serial.print ("requiredExport in Watts = ");
  Serial.println (REQUIRED_EXPORT_IN_WATTS);

   // Set the ADC's clock to system clock / 128
  ADCSRA  = (1 << ADPS0) + (1 << ADPS1) + (1 << ADPS2);
  // Set up the ADC to be free-running
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  ADCSRB &= B11111000; 
  ADCSRA |= (1 << ADEN);                 // Enable the ADC

  ADCSRA |= (1 << ADATE); // set the Auto Trigger Enable bit in the ADCSRA register.  Because
  // bits ADTS0-2 have not been set (i.e. they are all zero), the
  // ADC's trigger source is set to "free running mode".

  ADCSRA |= (1 << ADIE); // set the ADC interrupt enable bit. When this bit is written
  // to one and the I-bit in SREG is set, the
  // ADC Conversion Complete Interrupt is activated.

  ADCSRA |= (1 << ADSC); // start ADC manually first time
  sei();                 // Enable Global Interrupts

  for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  {
	cycleCount[phase] = 0;
    samplesDuringThisMainsCycle[phase] = 0;
    powerCal[phase] = 0.043;
    phaseCal[phase] = 0.5; // <- nominal values only
    voltageCal[phase] = 1.03;
    Serial.print ( "powerCal for L"); Serial.print(phase + 1);
    Serial.print (" =    "); Serial.println (powerCal[phase], 4);
    Serial.print ( "phaseCal for L"); Serial.print(phase + 1);
    Serial.print (" =     "); Serial.println (phaseCal[phase]);
    Serial.print ( "voltageCal for L"); Serial.print(phase + 1);
    Serial.print (" =    "); Serial.println (voltageCal[phase], 3);
  }
  Serial.println ("----");
#ifdef WORKLOAD_CHECK
   Serial.println ("WELCOME TO WORKLOAD_CHECK ");

//   <<- start of commented out section, to save on RAM space!
/*
   Serial.println ("  This mode of operation allows the spare processing capacity of the system");
   Serial.println ("to be analysed.  Additional delay is gradually increased until all spare time");
   Serial.println ("has been used up.  This value (in uS) is noted and the process is repeated.  ");
   Serial.println ("The delay setting is increased by 1uS at a time, and each value of delay is ");
   Serial.println ("checked several times before the delay is increased. ");
 */
//  <<- end of commented out section, to save on RAM space!

   Serial.println ("  The displayed value is the amount of spare time, per set of V & I samples, ");
   Serial.println ("that is available for doing additional processing.");
   Serial.println ();
 #endif
}

// An Interrupt Service Routine is now defined which instructs the ADC to perform a conversion
// for each of the voltage and current sensors in turn.  A "data ready" flag is set after
// each set of converstions has been completed.
//   This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished, approx every 104 us.  In
// free-running mode, the ADC has already started its next conversion by the time that
// the ISR is executed.  The ISR therefore needs to "look ahead".
//   At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR
// which runs at this point therefore needs to capture the results of conversion Type N,
// and set up the conditions for conversion Type N+2, and so on.
//
// ADC Multiplexer Source:

// Because we only have a single ADC but 6 ADC pins to choose from we need to feed the signal into the ADC using the built in multiplexer.
// To do this by setting the MUXn bits in the ADMUX register.  
// The neat thing is that if you change these values while the conversion is running, the conversion will finish before the change takes 
// (so if you ever find yourself having problems with bad data you might be switching before the ADC finishes).

ISR(ADC_vect) {
  static unsigned char sample_index = 0;
/*  
  static int sample_I_raw;

  byte tmp;            // temp register for storage of misc data
  byte ad_mux;

  ad_mux = ADMUX;      // read the value of ADMUX register. Represents conversion is running now.
  ad_mux &= 0x0F;      // AND the first 4 bits (value of ADC pin being used)

  
  for ( byte iphase = 0; iphase < NO_OF_PHASES; iphase++ ) {
    if (ad_mux == sensorI[ iphase ]) {
      if ( iphase == 0 ) {
        tmp = NO_OF_PHASES - 1;
        sampleV[ tmp ] = ADC;
        sampleI[ tmp ] = sample_I_raw;
        dataReady = tmp;
      } else {
        tmp = iphase - 1;
        sampleV[ tmp ] = ADC; // capture the results of preveous conversion. Because now converting I, results are form V of preveaus phase.
        sampleI[ tmp ] = sample_I_raw;
        dataReady = tmp;
      }
      ADMUX = 0x40 + sensorV[ iphase ]; // set up the next-but-one conversion of current phase V.
      mux[sample_index] = ad_mux;
      sample_index++;
      break;
    } else if ( ad_mux == sensorV[ iphase ] ) {
      sample_I_raw = ADC; // capture the results of preveous conversion. Because now converting V, results are from I.
      mux[sample_index] = ad_mux;
      tmp = iphase + 1;
      if ( tmp >= NO_OF_PHASES ) {
        ADMUX = 0x40 + sensorI[ 0 ]; // Last phase, set up the next-but-one conversion of first phase I.
        sample_index = 0;
      } else {
        ADMUX = 0x40 + sensorI[ tmp ];  // set up the next-but-one conversion of next phase I.
        sample_index++;
      }
      break;
    } 
  }
*/
  static int sample_I0_raw;
  static int sample_I1_raw;
  static int sample_I2_raw;
  
  switch(sample_index)
  {
    case 0:
      sample_I0_raw = ADC;
      if (NO_OF_PHASES == 1) {
        ADMUX = 0x40 + sensorI[0];
      } else {
        ADMUX = 0x40 + sensorI[1]; // set up the next-but-one conversion
      }
      sample_index++; // advance the control flag             
      break;
    case 1:
      sampleV[0] = ADC;
      sampleI[0] = sample_I0_raw;
      if (NO_OF_PHASES == 1) {
        ADMUX = 0x40 + sensorV[0]; // for the next-but-one conversion
        sample_index = 0;
      } else {
        ADMUX = 0x40 + sensorV[1]; // for the next-but-one conversion
        sample_index++; // advance the control flag                
      }
      dataReadyForPhase = 0;
      break;
    case 2:
      sample_I1_raw = ADC;
      if (NO_OF_PHASES == 2) {
        ADMUX = 0x40 + sensorI[0];
      } else {
        ADMUX = 0x40 + sensorI[2]; // for the next-but-one conversion
      }
      sample_index++; // advance the control flag                
      break;
    case 3:
      sampleV[1] = ADC;
      sampleI[1] = sample_I1_raw;
      if (NO_OF_PHASES == 2) {
        ADMUX = 0x40 + sensorV[0];
        sample_index = 0;
      } else {
        ADMUX = 0x40 + sensorV[2]; // for the next-but-one conversion
        sample_index++; // advance the control flag
      }
      dataReadyForPhase = 1;
      break;
    case 4:
      sample_I2_raw = ADC; 
      ADMUX = 0x40 + sensorI[0]; // for the next-but-one conversion
      sample_index++; // advance the control flag                 
      break;
    case 5:
      sampleV[2] = ADC;
      sampleI[2] = sample_I2_raw;
      ADMUX = 0x40 + sensorV[0]; // for the next-but-one conversion
      sample_index = 0; // reset the control flag                
      dataReadyForPhase = 2;
      break;
    default:
      sample_index = 0;                 // to prevent lockup (should never get here)      
  }
}

void loop() {

#ifdef WORKLOAD_CHECK
  static int del = 0; // delay, as passed to delayMicroseconds()
  static int res = 0; // result, to be displayed at the next opportunity
  static byte count = 0; // to allow multiple runs per setting
  static byte displayFlag = 0; // to determine when printing may occur
#endif
  // each loop is for one pair of V & I measurements
  if (dataReadyForPhase < NO_OF_PHASES) {  // flag is set after every pair of ADC conversions
	byte phase  = dataReadyForPhase;
	dataReadyForPhase = NO_OF_PHASES; 		// clear dataready flag.

//    samplesindex++;
//    if ( samplesindex >= 50 ) {
//      samplesindex = 0;
//    }
    noOfSamplePairs++;              // for stats only
    samplesDuringThisMainsCycle[phase]++;  // for power calculation at the start of each mains cycle

    // store values from previous loop
    //    lastSampleV[phase]=sampleV[phase];            // for digital high-pass filter
    //    lastFilteredV[phase] = filteredV[phase];      // for HPF, used to identify the start of each mains cycle
    //    lastSampleVminusDC[phase] = sampleVminusDC[phase];  // for phasecal calculation

    // remove the DC offset from these samples as determined by a low-pass filter
    sampleVminusDC[phase] = sampleV[phase] - VDCoffset[phase];
    sampleIminusDC[phase] = sampleI[phase] - IDCoffset[phase];

    // a high-pass filter is used just for determining the start of each mains cycle
    filteredV[phase] = 0.996 * (lastFilteredV[phase] + sampleV[phase] - lastSampleV[phase]);

    // Establish the polarities of the latest and previous filtered voltage samples
    byte polarityOfLastReading = polarityNow[phase];
    if (filteredV[phase] >= 0)
      polarityNow[phase] = POSITIVE;
    else
      polarityNow[phase] = NEGATIVE;

    // Block executed once then
    if (polarityNow[phase] == POSITIVE) {
      if (polarityOfLastReading != POSITIVE) {

        // This is the start of a new mains cycle (just after the +ve going z-c point)
        cycleCount[phase]++; // for stats only
        firstLoopOfHalfCycle[phase] = true;

        //      checkLedStatus(); // a really useful function, but can be commented out if not required

        // update the Low Pass Filter for DC-offset removal
        prevVDCoffset[phase] = VDCoffset[phase];
        VDCoffset[phase] = prevVDCoffset[phase] + (0.01 * cumVdeltasThisCycle[phase]);

        prevIDCoffset[phase] = IDCoffset[phase];
        IDCoffset[phase] = prevIDCoffset[phase] + (0.01 * cumIdeltasThisCycle[phase]);

        //  Calculate the real power of all instantaneous measurements taken during the
        //  previous mains cycle, and determine the gain (or loss) in energy.
//        float realPower = POWERCAL * sumP[phase] / (float)samplesDuringThisMainsCycle[phase];
//        float realEnergy = realPower / cyclesPerSecond;
        // Debug output.
/*        if (phase == 0 &&  (cycleCount[0] % 10) == 5) {
          Serial.print("realPower = ");
          Serial.println(realPower);
        } */
        //----------------------------------------------------------------
        // WARNING - Serial statements can interfere with time-critical code, but
        //           they can be really useful for calibration trials!
        // ----------------------------------------------------------------

        if ((cycleCount[phase] % 1000) == 5) // display once per second
        {
          Serial.println (samplesDuringThisMainsCycle[phase]);
//          Serial.print (" ");
        }
/*          Serial.println(cycleCount[phase]);
          Serial.print(" loopCount between samples = ");
          Serial.println (loopCount);
          
//          for (byte i_phase = 0; i_phase < NO_OF_PHASES; i_phase++) {
            Serial.print(" phase = ");
            Serial.println(phase); */
/*            for (long i = 0; i < 100; i++) {
                      Serial.print (samplesV[i_phase][i]);
                      Serial.print (" ");
            }
            Serial.println ("");
            for (long i = 0; i < 100; i++) {
                      Serial.print (samplesI[i_phase][i]);
                      Serial.print (" ");
            }
            Serial.println ("");
          } */
/*          Serial.print(" samplesDuringThisMainsCycle = ");
          Serial.println(samplesDuringThisMainsCycle[phase]);
          // Serial.print("energy in bucket = "); Serial.println(energyInBucket);
          //        Serial.println(energyInBucket_4led); // has no upper or lower limits
          //        energyInBucket_4led = 0; // for calibration purposes only

          Serial.print("realEnergy = ");
          Serial.print(realEnergy);
          Serial.print(", energyInBucket = ");
          Serial.print(energyInBucket);
          Serial.print(", firingDelay = ");
          Serial.println(firingDelayInMicros[phase]);
          //        Serial.print(", samples = ");


        }  */
        // This is the start of a new mains cycle (just after the +ve going z-c point)
/*
        if (beyondStartUpPhase == true) {
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
        } else {
          // wait until the DC-blocking filters have had time to settle
          if ( cycleCount[0] > 100) // 100 mains cycles is 2 seconds
            beyondStartUpPhase = true;
        }

        //      energyInBucket = energyInBucket_4trial; // over-ride the measured value

        triggerNeedsToBeArmed = true;   // the trigger is armed every mains cycle
*/
        calculateFiringDelay(phase);

        // clear the per-cycle accumulators for use in this new mains cycle.
        sumP[phase] = 0;
        samplesDuringThisMainsCycle[phase] = 0;
        cumVdeltasThisCycle[phase] = 0;
        cumIdeltasThisCycle[phase] = 0;
      } // end of processing that is specific to the first +ve Vsample in each new mains cycle

      // still processing POSITIVE Vsamples ...
      // this next block is for burst mode control of the triac, the output
      // pin for its trigger being on digital pin 9
/*
      if (triggerNeedsToBeArmed == true) {
        // check to see whether the trigger device can now be reliably armed
        if ((sampleVminusDC[phase] * VOLTAGECAL) > 50) // 20V min for Motorola trigger
        {
          // It's now safe to arm the trigger.  So ...

          // first check the level in the energy bucket to determine whether the
          // triac should be fired or not at the next opportunity
          //
          if (energyInBucket > (capacityOfEnergyBucket / 2))
          {
            nextStateOfTriac = ON;  // the external trigger device is active low
            digitalWrite(outputPinForLed, 1);  // active high
          } else {
            nextStateOfTriac = OFF;
            digitalWrite(outputPinForLed, 0);
          }

          // then set the Arduino's output pin accordingly,
          digitalWrite(outputPinForTrigger, nextStateOfTriac);

          // and clear the flag.
          triggerNeedsToBeArmed = false;

        }
      }
*/
    }  // end of processing that is specific to positive Vsamples
    else {
      if (polarityOfLastReading != NEGATIVE) {
        firstLoopOfHalfCycle[phase] = true;
      }
    } // end of processing that is specific to positive Vsamples

    processSamplePair(phase);

#ifdef WORKLOAD_CHECK
    delayMicroseconds(del); // <--- to assess how much spare time there is
    if (dataReady < NO_OF_PHASES) {      // if data is ready again, delay was too long
      res = del;             // note the exact value
      del = 1;               // and start again with 1us delay
      count = 0;
      displayFlag = 0;
    } else {
      count++;          // to give several runs with the same value
      if (count > 50) {
        count = 0;
        del++;          //  increase delay by 1uS
      }
    }
#endif
    loopCount = 0;
    // End of each loop is for one pair of V & I measurements
  } else {
    loopCount++;
  }

  // Executed on every loop.
  //------------------------------------------------------------
  phaseAngleTriacControl();

#ifdef WORKLOAD_CHECK
  switch (displayFlag)
  {
    case 0: // the result is available now, but don't display until the next loop
      displayFlag++;
      break;
    case 1: // with minimum delay, it's OK to print now
      Serial.print(res);
      displayFlag++;
      break;
    case 2: // with minimum delay, it's OK to print now
      Serial.println("uS");
      displayFlag++;
      break;
    default:; // for most of the time, displayFlag is 3
  }
#endif
} // end of loop()


void calculateFiringDelay(byte phase) {
    // ********************************************************
     // start of section to support phase-angle control of triac
     // determines the correct firing delay for a direct-acting trigger

     // never fire if energy level is below lower threshold (zero power)
     if (energyInBucket <= 1300) {
       firingDelayInMicros[phase] = 99999;
     } else {
       // fire immediately if energy level is above upper threshold (full power)
       if (energyInBucket >= 2300) {
         firingDelayInMicros[phase] = 0;
       } else {
         // determine the appropriate firing point for the bucket's level
         // by using either of the following algorithms

     	// simple algorithm (with non-linear power response across the energy range)
         //        firingDelayInMicros = 10 * (2300 - energyInBucket);

         // complex algorithm which reflects the non-linear nature of phase-angle control.
         firingDelayInMicros[phase] = (asin((-1 * (energyInBucket - 1800) / 500)) + (PI / 2)) * (10000 / PI);

         // Suppress firing at low energy levels to avoid complications with
         // logic near the end of each half-cycle of the mains.
         // This cut-off affects approximately the bottom 5% of the energy range.
         if (firingDelayInMicros[phase] > 8500) {
           firingDelayInMicros[phase] = 99999; // never fire
         }
       }
     }
     // end of section to support phase-angle control of triac
     //*******************************************************
}

void phaseAngleTriacControl() {
    // ********************************************************
    // start of section to support phase-angle control of triac
    // controls the signal for firing the direct-acting trigger.
  for (byte load = 0; load < noOfDumploads; load++) {
    unsigned long timeNowInMicros = micros(); // occurs every loop, for consistent timing
    byte loadphase = loadPhases[load];

    if (firstLoopOfHalfCycle[loadphase] == true) {
      timeAtStartOfHalfCycleInMicros[loadphase] = timeNowInMicros;
      firstLoopOfHalfCycle[loadphase] = false;
      phaseAngleTriggerActivated[load] = false;
      // Unless dumping full power, release the trigger on the first loop in each
      // half cycle.  Ensures that trigger can't get stuck 'on'.
      if (firingDelayInMicros[loadphase] > 100) {
        digitalWrite(outputPinForPAcontrol[load], OFF);
      }
    }

    if (phaseAngleTriggerActivated[load] == true) {
      // Unless dumping full power, release the trigger on all loops in this
      // half cycle after the one during which the trigger was set.
      if (firingDelayInMicros > 100) {
        digitalWrite(outputPinForPAcontrol[load], OFF);
      }
    } else {
      if (timeNowInMicros >= (timeAtStartOfHalfCycleInMicros[loadphase] + firingDelayInMicros[loadphase])) {
        digitalWrite(outputPinForPAcontrol[load], ON);
        phaseAngleTriggerActivated[load] = true;
      }
    }
    // end of section to support phase-angle control of triac
    //*******************************************************
  }
}

void processSamplePair(byte phase) {
    // Apply phase-shift to the voltage waveform to ensure that the system measures a
    // resistive load with a power factor of unity.
    float phaseShiftedVminusDC = lastSampleVminusDC[phase] + PHASECAL * (sampleVminusDC[phase] - lastSampleVminusDC[phase]);
    float instP = phaseShiftedVminusDC * sampleIminusDC[phase]; //  power contribution for this pair of V&I samples
    sumP[phase] += instP;    // cumulative power contributions for this mains cycle

    cumVdeltasThisCycle[phase] += (sampleV[phase] - VDCoffset[phase]); // for use with LP filter
    cumIdeltasThisCycle[phase] += (sampleI[phase] - IDCoffset[phase]); // for use with LP filter

    // store values from previous loop
    lastSampleV[phase] = sampleV[phase];            // for digital high-pass filter
    lastFilteredV[phase] = filteredV[phase];      // for HPF, used to identify the start of each mains cycle
    lastSampleVminusDC[phase] = sampleVminusDC[phase];  // for phasecal calculation

//    lastSampleI[phase] = sampleI[phase];            // for digital high-pass filter
//    lastFilteredI[phase] = filteredI[phase];      // for HPF, used to identify the start of each mains cycle
//    lastSampleIminusDC[phase] = sampleIminusDC[phase];  // for phasecal calculation
}

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
      Serial.println(millis() / 1000);

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
