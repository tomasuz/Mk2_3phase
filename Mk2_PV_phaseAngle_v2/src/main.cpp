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

// #define DEBUG

#ifdef DEBUG
// #include <app_api.h>
// #include "avr8-stub.h"
#endif

#define POSITIVE 1
#define NEGATIVE 0
#define ON 1  // for use with LED( active high) or relay
#define OFF 0

#define NO_OF_PHASES 3
#define NO_OF_PHASE_RELAYS 2
#define CYCLES_PER_SECOND 50
#define DATALOG_PERIOD_IN_SECONDS 1
#define RELAY_CONTROL_DELAY_IN_SECONDS 20

#define DELAYINMAINSCYCLES 6

// const byte noOfDumploads = 3; 
const byte noOfDumploads = 0;
const byte noOfPWMControlledDumploads = 1;

// enum loadPriorityModes {LOAD_1_HAS_PRIORITY, LOAD_0_HAS_PRIORITY};
// enum loadPriorityModes {LOAD_0_HAS_PRIORITY};

// for use with trigger(load) device:
// enum loadStates {LOAD_ON, LOAD_OFF}; // for use if loads are active low (original PCB)
enum loadStates {LOAD_OFF, LOAD_ON}; // for use if loads are active high (Rev 2 PCB)
enum loadStates logicalLoadState[noOfDumploads]; 
enum loadStates logicalPWMLoadState[noOfPWMControlledDumploads];
unsigned char PWMLoadValue[noOfPWMControlledDumploads];

// array defining to which phase which load is connected. 
const unsigned char loadPhases[noOfPWMControlledDumploads] = {0};

// ----------- Pinout assignments  -----------

// byte outputPinForLed = 13;
// byte outputPinForTrigger = 5;
const byte outputPinsForPAcontrol[noOfDumploads] = {};
const byte outputPinsForPWMcontrol[noOfPWMControlledDumploads] = {9};
const unsigned char outputPinForPhaseRelayControl[NO_OF_PHASE_RELAYS] = {3, 4};
const unsigned char phaseWhichRelayControl[NO_OF_PHASE_RELAYS] = {1, 2};
const unsigned char phaseToWhichRelaysConnect[NO_OF_PHASE_RELAYS] = {0, 0};
const unsigned int relayControlDelay = RELAY_CONTROL_DELAY_IN_SECONDS * CYCLES_PER_SECOND;
// byte ledDetectorPin = 2;  // digital
// byte ledRepeaterPin = 10;  // digital

// analogue input pins
// ADC0 (pin 23) phase 1 V
// ADC1 (pin 24) phase 1 I
// ADC2 (pin 25) phase 2 V
// ADC3 (pin 26) phase 2 I
// ADC4 (pin 27) phase 3 V
// ADC5 (pin 28) phase 3 I
const byte sensorV[NO_OF_PHASES] = {B00000000, B00000010, B00000100}; // for 3-phase PCB
const byte sensorI[NO_OF_PHASES] = {B00000001, B00000011, B00000101}; // for 3-phase PCB
// const byte sensorV[3] = {B00000000, B00000010, B00000100}; // for 3-phase PCB
// const byte sensorI[3] = {B00000001, B00000011, B00000101}; // for 3-phase PCB
// 1 phase:
// const byte sensorV[NO_OF_PHASES] = {B00000000}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001}; // for 3-phase PCB
// 2 phase
// const byte sensorV[NO_OF_PHASES] = {B00000000, B00000010}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001, B00000011}; // for 3-phase PCB


const float safetyMargin_watts = 4000;  // <<<------ increase for more export
unsigned long cycleCount[NO_OF_PHASES];
unsigned long loopCount = 0;
unsigned int delayInMainsCycles[NO_OF_PHASES];

// Initial values setting moved to setup().....
int samplesDuringThisMainsCycle[NO_OF_PHASES];
int samplesDuringLastMainsCycle[NO_OF_PHASES];
// byte nextStateOfTriac;
unsigned char stateOfRelays[NO_OF_PHASE_RELAYS];

const int maxDatalogCountInMainsCycles = DATALOG_PERIOD_IN_SECONDS * CYCLES_PER_SECOND;

unsigned char polarityNow[NO_OF_PHASES];

// boolean triggerNeedsToBeArmed = false;
boolean beyondStartUpPhase = false;

float energyInBucket[NO_OF_PHASES]; // mimics the operation of a meter at the grid connection point.

// int sampleV,sampleI;   // voltage & current samples are integers in the ADC's input range 0 - 1023
// int lastSampleV[NO_OF_PHASES];     // stored value from the previous loop (HP filter is for voltage samples only)
// float lastFilteredV[NO_OF_PHASES], filteredV[NO_OF_PHASES]; //  voltage values after HP-filtering to remove the DC offset

// int lastSampleI[NO_OF_PHASES];     // stored value from the previous loop (HP filter is for voltage samples only)
// float lastFilteredI[NO_OF_PHASES], filteredI[NO_OF_PHASES]; //  voltage values after HP-filtering to remove the DC offset

// float VDCoffset[NO_OF_PHASES];              // <<--- for LPF
long DCoffset_V_long[NO_OF_PHASES];
long cumVdeltasThisCycle_long[NO_OF_PHASES];   // <<--- for LPF
  // Define operating limits for the LP filters which identify DC offset in the voltage 
  // sample streams.  By limiting the output range, these filters always should start up 
  // correctly.
static long DCoffset_V_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
static long DCoffset_V_max = (long)(512L + 100) * 256; // mid-point of ADC plus a working margin

//float prevIDCoffset[NO_OF_PHASES];          // <<--- for LPF
long DCoffset_I_long[NO_OF_PHASES];              // <<--- for LPF
long cumIdeltasThisCycle_long[NO_OF_PHASES];   // <<--- for LPF
  // Define operating limits for the LP filters which identify DC offset in the current 
  // sample streams.  By limiting the output range, these filters always should start up 
  // correctly.
static long DCoffset_I_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
static long DCoffset_I_max = (long)(512L + 100) * 256; // mid-point of ADC plus a working margin


long sampleV_minusDC_long[NO_OF_PHASES];         // <<--- for LPF
long sampleI_minusDC_long[NO_OF_PHASES];         // <<--- used with LPF
long lastSampleV_minusDC_long[NO_OF_PHASES];
// float lastSampleVminusDC[NO_OF_PHASES];     // <<--- used with LPF
// float lastSampleIminusDC[NO_OF_PHASES];     // <<--- used with LPF
long sumP[NO_OF_PHASES];   //  cumulative sum of power calculations within each mains cycle
long lastSumP[7][NO_OF_PHASES];   //  cumulative sum of power calculations within each mains cycle
long sumV[NO_OF_PHASES];   //  cumulative sum of voltage calculations within each mains cycle
long sumVOfLastMainsCycle[NO_OF_PHASES];
float realV[NO_OF_PHASES];
float realPower[NO_OF_PHASES];
float realLastPower[NO_OF_PHASES];
float realFilteredPower[NO_OF_PHASES];

const float POWERCAL = 0.06;  // To convert the product of raw V & I samples into Joules.
// float VOLTAGECAL; // To convert raw voltage samples into volts.  Used for determining when
// the trigger device can be safely armed
// Units are Joules per ADC-level squared.  Used for converting the product of
// voltage and current samples into Joules.
//    To determine this value, note the rate that the energy bucket's
// level increases when a known load is being measured at a convenient
// test location (e.g  using a mains extention with the outer cover removed so that
// the current-clamp can fit around just one core.  Adjust POWERCAL so that
// 'measured value' = 'expected value' for various loads.  The value of
// POWERCAL is not critical as any absolute error will cancel out when
// import and export flows are balanced.


// for interaction between the main processor and the ISR
volatile byte dataReadyForPhase = NO_OF_PHASES; // Use byte for data ready from ADC and store phase to it. 3 - no data ready.
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
// float  phaseCal[NO_OF_PHASES];
const float phaseCal[NO_OF_PHASES] = {0.17, 0.17, 0.17}; // <- nominal values only
int phaseCal_int[NO_OF_PHASES];           // to avoid the need for floating-point maths

// For datalogging purposes, voltageCal has been added too.  Because the range of ADC values is
// similar to the actual range of volts, the optimal value for this cal factor is likely to be
// close to unity.
// Initial values seting moved to setup().....
const float voltageCal[NO_OF_PHASES] = {0.94, 0.94, 0.94}; // compared with Fluke 77 meter

// items for LED monitoring
// byte ledState, prevLedState;
// boolean ledRecentlyOnFlag = false;
// unsigned long ledOnAt;
// float energyInBucket_4led = 0;
// float energyLevelAtLastLedPulse;

// items for phase-angle control of triac
boolean firstLoopOfHalfCycle[NO_OF_PHASES];
boolean phaseAngleTriggerActivated[noOfDumploads];

// Arrays for debugging
// int samplesV[NO_OF_PHASES][50];
// int samplesI[NO_OF_PHASES][50];
// float phaseShiftedVminusDCs[NO_OF_PHASES][50];
// float sampleVminusDCs[NO_OF_PHASES][50];

void setup() {
#ifdef DEBUG
  // initialize GDB stub
  // debug_init();
#else
  // Serial.begin(9600);
  // Serial.begin(115200);
  // Serial.begin(230400);
  Serial.begin(500000);
  Serial.setTimeout(4); // for rapid input of data (default is 1000ms)
#endif

//  VOLTAGECAL = (float)679 / 471; // Units are Volts per ADC-level.
  // This value is used to determine when the voltage level is suitable for
  // arming the external trigger device.  To set this value, note the min and max
  // numbers that are seen when measuring 240Vac via the voltage sensor, which
  // is 678.8V p-t-p.  The range on my setup is 471 meaning that I'm under-reading
  // voltage by 471/679.  VOLTAGECAL therefore need to be the inverse of this, i.e.
  // 679/471 or 1.44
  for (byte load = 0; load < noOfDumploads; load++) {
    pinMode(outputPinsForPAcontrol[load], OUTPUT);
    logicalLoadState[load] = LOAD_OFF;
#ifndef DEBUG
    Serial.println ((String) "Output Pin For PA control: " + outputPinsForPAcontrol[load]);
#endif
  }

  for (byte load = 0; load < noOfPWMControlledDumploads; load++) {
    analogWrite(outputPinsForPWMcontrol[load], 0);
    logicalPWMLoadState[load] = LOAD_OFF;
    PWMLoadValue[load] = 0;
#ifndef DEBUG
    Serial.println ((String) "Output Pin For PWM PA control: " + outputPinsForPWMcontrol[load]);
#endif
  }

  for (byte load = 0; load < NO_OF_PHASE_RELAYS; load++) {
    pinMode(outputPinForPhaseRelayControl[load], OUTPUT);
    stateOfRelays[load] = OFF;
#ifndef DEBUG
    Serial.println ((String) "Output Pin For Phase Relay control: " + outputPinForPhaseRelayControl[load]);
#endif
  }
#ifndef DEBUG
  Serial.println ("ADC mode:       free-running");
#endif
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

  for (byte phase = 0; phase < NO_OF_PHASES; phase++) {
	  firstLoopOfHalfCycle[phase] = false;
	  sumP[phase] = 0;
    for (byte i = 0; i < 7; i++) {
      lastSumP[i][phase] = 0;
    }
	  sumV[phase] = 0;
	  sumVOfLastMainsCycle[phase] = 0;
	  realV[phase] = 0.0;
    realPower[phase] = 0.0;
    realLastPower[phase] = 0.0;
    cycleCount[phase] = 0;
    energyInBucket[phase] = 0.0;
    samplesDuringThisMainsCycle[phase] = 0;
    samplesDuringLastMainsCycle[phase] = 32;
    delayInMainsCycles[phase] = 0;
    powerCal[phase] = POWERCAL;
    phaseCal_int[phase] = phaseCal[phase] * 256;  // for integer maths
    DCoffset_V_long[phase] = 512L * 256;  // nominal mid-point value of ADC @ x256 scale  
    DCoffset_I_long[phase] = 512L * 256;
//    voltageCal[phase] = 1.03;
#ifndef DEBUG
	  Serial.print ( "powerCal for L"); Serial.print(phase + 1);
	  Serial.print (" =    "); Serial.println (powerCal[phase], 4);
    Serial.print ((String) "phaseCal for L" + (phase + 1));
    Serial.print (" =     "); Serial.println (phaseCal[phase]);
    Serial.print ( "voltageCal for L"); Serial.print(phase + 1);
    Serial.print (" =    "); Serial.println(voltageCal[phase], 3);
#endif
  }

#ifndef DEBUG
  Serial.println ("----");
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
	static int sample_I0_raw;
	static int sample_I1_raw;
	static int sample_I2_raw;
  
  switch(sample_index) {
    case 0:
      sample_I0_raw = ADC;
      if (NO_OF_PHASES == 1) {
        ADMUX = 0x40 + sensorI[0];
      } else {
        ADMUX = 0x40 + sensorI[1]; // set up the next-but-one conversion
      }
      sample_index++; 		// advance the control flag
      break;
    case 1:
      sampleV[0] = ADC;
      sampleI[0] = sample_I0_raw;
      if (NO_OF_PHASES == 1) {
        ADMUX = 0x40 + sensorV[0]; // for the next-but-one conversion
        sample_index = 0;
      } else {
        ADMUX = 0x40 + sensorV[1]; // for the next-but-one conversion
        sample_index++; 	// advance the control flag
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
      sample_index++; 		// advance the control flag
      break;
    case 3:
      sampleV[1] = ADC;
      sampleI[1] = sample_I1_raw;
      if (NO_OF_PHASES == 2) {
        ADMUX = 0x40 + sensorV[0];
        sample_index = 0;
      } else {
        ADMUX = 0x40 + sensorV[2]; // for the next-but-one conversion
        sample_index++; 	// advance the control flag
      }
      dataReadyForPhase = 1;
      break;
    case 4:
      sample_I2_raw = ADC; 
      ADMUX = 0x40 + sensorI[0]; // for the next-but-one conversion
      sample_index++; 		// advance the control flag
      break;
    case 5:
      sampleV[2] = ADC;
      sampleI[2] = sample_I2_raw;
      ADMUX = 0x40 + sensorV[0]; // for the next-but-one conversion
      sample_index = 0; 	// reset the control flag
      dataReadyForPhase = 2;
      break;
    default:
      sample_index = 0;		// to prevent lockup (should never get here)
  }
}

void calculateOfsetsAndEnergy(unsigned char phase) {
  // update the Low Pass Filter for DC-offset removal
  DCoffset_V_long[phase] += (cumVdeltasThisCycle_long[phase]>>12);
  DCoffset_I_long[phase] += (cumIdeltasThisCycle_long[phase]>>12);

  if (DCoffset_V_long[phase] < DCoffset_V_min) {  
    DCoffset_V_long[phase] = DCoffset_V_min; }
  else if (DCoffset_V_long[phase] > DCoffset_V_max) {
    DCoffset_V_long[phase] = DCoffset_V_max; }

  if (DCoffset_I_long[phase] < DCoffset_I_min) {  
    DCoffset_I_long[phase] = DCoffset_I_min; }
  else if (DCoffset_I_long[phase] > DCoffset_I_max) {
    DCoffset_I_long[phase] = DCoffset_I_max; }

  //  Calculate the real power of all instantaneous measurements taken during the
  //  previous mains cycle, and determine the gain (or loss) in energy.
  realV[phase] = sumVOfLastMainsCycle[phase] / (float)samplesDuringLastMainsCycle[phase];

  long temp = sumP[phase];
  for (byte i = 0; i < 7; i++) {
    temp = temp + lastSumP[i][phase];
  }
  realFilteredPower[phase] = powerCal[phase] * (temp >> 3) / (float)samplesDuringLastMainsCycle[phase];
  realLastPower[phase] = realFilteredPower[phase];
  realPower[phase] = powerCal[phase] * sumP[phase] / (float)samplesDuringLastMainsCycle[phase];
  // a high-pass filter is used just for determining the start of each mains cycle
}

void controllPWMFiringDelay(unsigned char phase) {
  // ********************************************************
  // start of section to support phase-angle control of triac
  // determines the correct firing delay for a direct-acting trigger

  for (unsigned char load = 0; load < noOfPWMControlledDumploads; load++) {
    if ( loadPhases[load] == phase && delayInMainsCycles[phase] == 0 ) {
      // The idea is to have auto regulation to minimize power output to grid.
      // Power must be only imported or 0
      // Assume realEnergy[phase] is negative if we exporting.
      if (realFilteredPower[phase] < -500.0) {
        logicalPWMLoadState[load] = LOAD_ON;
    //    if PWMLoadValue[load] = 255 - PWM is 5V, trac is fully opened.
        if (PWMLoadValue[load] <= 235) {
          PWMLoadValue[load] = PWMLoadValue[load] + 20;
          delayInMainsCycles[phase] = DELAYINMAINSCYCLES;
          analogWrite(outputPinsForPWMcontrol[load], PWMLoadValue[load]);
        }
      } else if (realFilteredPower[phase] < -250.0) {
        logicalPWMLoadState[load] = LOAD_ON;
    //    if PWMLoadValue[load] = 255 - PWM is 5V, trac is fully opened.
        if (PWMLoadValue[load] <= 245) {
          PWMLoadValue[load] = PWMLoadValue[load] + 10;
          delayInMainsCycles[phase] = DELAYINMAINSCYCLES;
          analogWrite(outputPinsForPWMcontrol[load], PWMLoadValue[load]);
        }
      } else if (realFilteredPower[phase] < -125.0) {
        logicalPWMLoadState[load] = LOAD_ON;
    //    if PWMLoadValue[load] = 255 - PWM is 5V, trac is fully opened.
        if (PWMLoadValue[load] <= 250) {
          PWMLoadValue[load] = PWMLoadValue[load] + 5;
          delayInMainsCycles[phase] = DELAYINMAINSCYCLES;
          analogWrite(outputPinsForPWMcontrol[load], PWMLoadValue[load]);
        }
      } else if (realFilteredPower[phase] < -75.0) {
        logicalPWMLoadState[load] = LOAD_ON;
    //    if PWMLoadValue[load] = 255 - PWM is 5V, trac is fully opened.
        if (PWMLoadValue[load] <= 252) {
          PWMLoadValue[load] = PWMLoadValue[load] + 3;
          delayInMainsCycles[phase] = DELAYINMAINSCYCLES;
          analogWrite(outputPinsForPWMcontrol[load], PWMLoadValue[load]);
        }
      } else if (realFilteredPower[phase] < 1.0) {
        logicalPWMLoadState[load] = LOAD_ON;
    //    if PWMLoadValue[load] = 255 - PWM is 5V, trac is fully opened.
        if (PWMLoadValue[load] < 255) {
          PWMLoadValue[load] = PWMLoadValue[load] + 1;
          delayInMainsCycles[phase] = DELAYINMAINSCYCLES;
          analogWrite(outputPinsForPWMcontrol[load], PWMLoadValue[load]);
        }
      } else if (realFilteredPower[phase] >= 30.0) {
         // now we importing enegry from grid, so lover diverting increasing delay:
    //     if (realPower[phase] > realLastPower[phase]) {
        if (PWMLoadValue[load] > 0) {
           PWMLoadValue[load] = PWMLoadValue[load] - 1;
           delayInMainsCycles[phase] = DELAYINMAINSCYCLES;
           analogWrite(outputPinsForPWMcontrol[load], PWMLoadValue[load]);
        } else {
          logicalPWMLoadState[load] = LOAD_OFF;
        }
      }
    } else {
      delayInMainsCycles[phase]--;
    }
  }

  // end of section to support phase-angle control of triac
  //*******************************************************
}

//void phaseAngleTriacControl() {
//    // ********************************************************
//    // start of section to support phase-angle control of triac
//    // controls the signal for firing the direct-acting trigger.
//  for (unsigned char load = 0; load < noOfDumploads; load++) {
//    unsigned long timeNowInMicros = micros(); // occurs every loop, for consistent timing
//    unsigned char loadphase = loadPhases[load];
//
//    if (firstLoopOfHalfCycle[loadphase] == true) {
//      timeAtStartOfHalfCycleInMicros[loadphase] = timeNowInMicros;
//      firstLoopOfHalfCycle[loadphase] = false;
//      phaseAngleTriggerActivated[load] = false;
//      // Unless dumping full power, release the trigger on the first loop in each
//      // half cycle.  Ensures that trigger can't get stuck 'on'.
////      if (firingDelayInMicros[loadphase] > 100) {
//        digitalWrite(outputPinsForPAcontrol[load], LOAD_OFF);
////      }
//    }
//
//    if (phaseAngleTriggerActivated[load] == true) {
//      // Unless dumping full power, release the trigger on all loops in this
//      // half cycle after the one during which the trigger was set.
//      if (firingDelayInMicros[loadphase] > 100) {
//        digitalWrite(outputPinsForPAcontrol[load], LOAD_OFF);
//      }
//    } else {
//      if (timeNowInMicros > (timeAtStartOfHalfCycleInMicros[loadphase] + firingDelayInMicros[loadphase])) {
//        digitalWrite(outputPinsForPAcontrol[load], LOAD_ON);
//        phaseAngleTriggerActivated[load] = true;
//      }
//    }
//    // end of section to support phase-angle control of triac
//    //*******************************************************
//  }
//}

void processSamplePair(byte phase) {
	// Apply phase-shift to the voltage waveform to ensure that the system measures a
  // resistive load with a power factor of unity.
  long phaseShiftedSampleV_minusDC_long = lastSampleV_minusDC_long[phase]
         + (((sampleV_minusDC_long[phase] - lastSampleV_minusDC_long[phase]) * phaseCal_int[phase])>>8);

  // float phaseShiftedVminusDC = lastSampleVminusDC[phase] + phaseCal[phase] * (sampleVminusDC[phase] - lastSampleVminusDC[phase]);

  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4 = phaseShiftedSampleV_minusDC_long>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4 = sampleI_minusDC_long[phase]>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
  instP = instP>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP[phase] += instP; // cumulative power contributions for this mains cycle, scaling as for Mk2 (V_ADC x I_ADC)


  //float instP = phaseShiftedVminusDC * sampleIminusDC[phase]; //  power contribution for this pair of V&I samples
  // sumV[phase] += abs(phaseShiftedVminusDC);
  // for the Vrms calculation (for datalogging only)
  long inst_Vsquared = filtV_div4 * filtV_div4; // 32-bits (now x4096, or 2^12)
  inst_Vsquared = inst_Vsquared>>12;     // scaling is now x1 (V_ADC x I_ADC)
  sumV[phase] += inst_Vsquared; // cumulative V^2 (V_ADC x I_ADC), squared

  cumVdeltasThisCycle_long[phase] += sampleV_minusDC_long[phase]; // for use with LP filter
  cumIdeltasThisCycle_long[phase] += sampleI_minusDC_long[phase]; // for use with LP filter

  // store values from previous loop
  // lastSampleV[phase] = sampleV[phase];            // for digital high-pass filter
  // lastFilteredV[phase] = filteredV[phase];      // for HPF, used to identify the start of each mains cycle
  lastSampleV_minusDC_long[phase] = sampleV_minusDC_long[phase]; // for phasecal calculation

//  lastSampleI[phase] = sampleI[phase];            // for digital high-pass filter
//  lastFilteredI[phase] = filteredI[phase];      // for HPF, used to identify the start of each mains cycle
//  lastSampleIminusDC[phase] = sampleIminusDC[phase];  // for phasecal calculation

  // Clear summed values of phase with low voltage (no connected) after 50 cycles:
  if ( samplesDuringThisMainsCycle[phase] >= 50 ) {
    realV[phase] = sumV[phase] / (float)samplesDuringThisMainsCycle[phase];
    sumVOfLastMainsCycle[phase] = sumV[phase];
    // Count cycles even if they are not detected because of low value.
    // cycleCount[phase]++;
    for (byte i = 0; i < 6; i++) {
      lastSumP[i][phase] = lastSumP[i+1][phase];
    }
    lastSumP[6][phase] = sumP[phase];
    sumP[phase] = 0;
    sumV[phase] = 0;
    samplesDuringLastMainsCycle[phase] = samplesDuringThisMainsCycle[phase];
    samplesDuringThisMainsCycle[phase] = 0;
    cumVdeltasThisCycle_long[phase] = 0;
    cumIdeltasThisCycle_long[phase] = 0;
  }
}

void controlPhaseSwichRellay(unsigned char phase) {

//	unsigned char outputPinForPhaseRelayControl[NO_OF_PHASE_RELAYS] = {0, 1};
//	unsigned char phaseWhichRelayControl[NO_OF_PHASE_RELAYS] = {1, 2};
//	unsigned char phaseToWhichRelaysConnect[NO_OF_PHASE_RELAYS] = {0, 0};
//	stateOfRelays[load] = LOAD_OFF;
//	enum loadStates logicalLoadState[noOfDumploads];
  if ((cycleCount[phase] % 100) == 0) {// control once per second
	  // First ensure phase used power form grid is less then safety margin:
	  if (realPower[phase] > safetyMargin_watts) {
		  for (unsigned char relay = 0; relay < NO_OF_PHASE_RELAYS; relay++) {
			  if (phase == phaseToWhichRelaysConnect[relay]) {
				  // Switch relay off to the main phase:
				  if (stateOfRelays[relay] == ON) {
					  digitalWrite(outputPinForPhaseRelayControl[relay], OFF);
					  stateOfRelays[relay] = OFF;
					  // Only switch one phase per cycle:
					  break;
				  }
			  }
		  }
	  }

    // Check power usage for phase:
    if (realPower[phase] < 10.0 ) {
      // If power usage is low - check is this phase in list is phases To Which Relays Connect ( phaseToWhichRelaysConnect )
      for (unsigned char relay = 0; relay < NO_OF_PHASE_RELAYS; relay++) {
        if (phaseToWhichRelaysConnect[relay] == phase) {
//          if (realPower[phaseWhichRelayControl[relay]] > 10.0 && stateOfRelays[relay] == OFF) {
          if (stateOfRelays[relay] == OFF) {
            // Switch relay on to the main phase:
            digitalWrite(outputPinForPhaseRelayControl[relay], ON);
            stateOfRelays[relay] = ON;
#ifndef DEBUG
            Serial.println((String)"Connect phase " + phaseWhichRelayControl[relay] + " Relay to load phase " + phase);
#endif
            // Only switch one phase per cycle:
            break;
          }
        }
      }
    }
	  //for (unsigned char load = 0; load < noOfDumploads; load++) {
	  // If phase of load is diverting (has enough power to divert) - switch relay ON to reconnect power consumers to this phase:
	    // if (loadPhases[noOfDumploads] == phase && logicalLoadState[load] == ON) {

		// Second variant:
		// If phase of load uses small amount of power from grid - switch relay ON to reconnect power consumers to this phase:
		//if (loadPhases[load] == phase && realPower[phase] < 10.0 ) {
	  //  	for (unsigned char relay = 0; relay < NO_OF_PHASE_RELAYS; relay++) {
	    		// Only switch relay if there are voltage in it and it is off:
	  //  		if (phaseToWhichRelayConnect[relay] == phase && realPower[phaseWhichRelayControl[relay]] > 0.0 && stateOfRelays[relay] == OFF) {
	    			// Switch relay on to the main phase:
	  // 				digitalWrite(outputPinForPhaseRelaycontrol[relay], ON);
	  // 				stateOfRelays[relay] = ON;
    //        Serial.println((String)"Connect phase " + loadPhases[load] + " Relay to load phase " + phase);
					// Only switch one phase per cycle:
		//			break;
	  //  		}
	  //  	}
	  //  }
	  //}
  }
}

#ifndef DEBUG
void printInfo(unsigned char phase) {
    switch(samplesDuringThisMainsCycle[0]) {
          case 0:
            break;
          case 1:
//            Serial.println((String) "VDCoffset:\t" + VDCoffset[phase]);
            break;
          case 2:
//            Serial.println((String) "DCoffset_V_long of phase 0:\t" + DCoffset_V_long[0]);
            Serial.println ((String) "realPower of phase\t"+ phase + " :\t" + realPower[phase]);
            break;
          case 3:
//            Serial.println((String) "DCoffset_I_long of phase 0:\t" + DCoffset_I_long[0]);
            Serial.println((String) "Filtered Real Last Power phase\t"+ phase + " :\t" + realLastPower[phase]);
            break;
          case 4:
            Serial.println((String) "Filtered Real Power phase\t"+ phase + " :\t" + realFilteredPower[phase]);
            break;
          case 5:
              Serial.println ((String) "Samples of phase\t"+ phase + " :\t" + samplesDuringLastMainsCycle[phase]);
              break;
          case 6:
              Serial.println((String) "realV of phase\t\t"+ phase + " :\t" + ((int)(voltageCal[phase] * sqrt(realV[phase]))));
    	        break;
    	    case 7:
//    	        Serial.print ("realPower of phase\t0:\t");
//    	        Serial.println (realPower[0]);
            break;
          case 8:
            Serial.print ("State Of Relays:\t");
              for (unsigned char relay = 0; relay < NO_OF_PHASE_RELAYS; relay++) {
                Serial.print ((String) stateOfRelays[relay] + "\t");
              }
              Serial.println("");
            break;
          case 9:
              Serial.println("PWMLoadValue \t");
              for (unsigned char load = 0; load < noOfPWMControlledDumploads; load++) {
                Serial.print((String) PWMLoadValue[load] + "\t");
              }
              Serial.println("");
        	    break;
    } // end of case
}
#endif

void loop() {
  static unsigned int datalogCountInMainsCycles;
//  static unsigned int datalogCountInMainsCycles

  // each loop is for one pair of V & I measurements
  if (dataReadyForPhase < NO_OF_PHASES) {  // flag is set after every pair of ADC conversions
    unsigned char phase = dataReadyForPhase;
    dataReadyForPhase = NO_OF_PHASES; // clear dataready flag.

    samplesDuringThisMainsCycle[phase]++;  // for power calculation at the start of each mains cycle

    // remove the DC offset from these samples as determined by a low-pass filter
    sampleV_minusDC_long[phase] = ((long)sampleV[phase]<<8) - DCoffset_V_long[phase];
    sampleI_minusDC_long[phase] = ((long)sampleI[phase]<<8) - DCoffset_I_long[phase];

    // Establish the polarities of the latest and previous filtered voltage samples
    byte polarityOfLastReading = polarityNow[phase];
    // if (filteredV[phase] >= 0)
    if (sampleV_minusDC_long[phase] >= 0)
      polarityNow[phase] = POSITIVE;
    else
      polarityNow[phase] = NEGATIVE;

    // Only detect the start of a new mains cycle if phase voltage is greater then some noise voltage:
    if (realV[phase] > 10.0) {
      // Block executed once then
      if (polarityNow[phase] == POSITIVE) {
        if (polarityOfLastReading != POSITIVE) {
    			// This is the start of a new mains cycle (just after the +ve going z-c point)
    			cycleCount[phase]++; // for stats only
    			firstLoopOfHalfCycle[phase] = true;
//          if (phase == 0) {
//            Serial.println((String) "samplesDuringThisMainsCycle of phase\t" + phase + ":\t" + samplesDuringThisMainsCycle[phase]);
//          }

    			// checkLedStatus(); // a really useful function, but can be commented out if not required
          calculateOfsetsAndEnergy(phase);
          if (beyondStartUpPhase != true) {
            // wait until the DC-blocking filters have had time to settle
            if ( cycleCount[0] > 100) // 100 mains cycles is 2 seconds
              beyondStartUpPhase = true;
          } else {
            controllPWMFiringDelay(phase);
          }

    			//      energyInBucket = energyInBucket_4trial; // over-ride the measured value
    			//        triggerNeedsToBeArmed = true;   // the trigger is armed every mains cycle
    			// clear the per-cycle accumulators for use in this new mains cycle.
    			sumVOfLastMainsCycle[phase] = sumV[phase];
    	    for (byte i = 0; i < 6; i++) {
    	      lastSumP[i][phase] = lastSumP[i+1][phase];
    	    }
          lastSumP[6][phase] = sumP[phase];
          sumP[phase] = 0;
    			sumV[phase] = 0;
    			samplesDuringLastMainsCycle[phase] = samplesDuringThisMainsCycle[phase];
    			samplesDuringThisMainsCycle[phase] = 0;
    			cumVdeltasThisCycle_long[phase] = 0;
    			cumIdeltasThisCycle_long[phase] = 0;
    			// end of processing that is specific to the first +ve Vsample in each new mains cycle
    		}
        // still processing POSITIVE Vsamples ...
        if ((phase == 0) && samplesDuringThisMainsCycle[0] == 1) {
          if (beyondStartUpPhase) {
            // This code is executed once per 20mS, shortly after the start of each new
            // mains cycle on phase 0.
            //
            datalogCountInMainsCycles++;
            if (datalogCountInMainsCycles >= maxDatalogCountInMainsCycles) {
              datalogCountInMainsCycles = 0;
            }
          } 
        }
    	// end of processing that is specific to positive Vsamples
    	} else { //polarityNow[phase] == NEGATIVE)
    		if (polarityOfLastReading != NEGATIVE) {
    			firstLoopOfHalfCycle[phase] = true;
    		}
    	} // end of processing ve going z-c point
    } // end of realV[phase] > 10.0

    processSamplePair(phase);
    loopCount = 0;
    if (datalogCountInMainsCycles == 0 && beyondStartUpPhase) {
      controlPhaseSwichRellay(phase);
    }
#ifndef DEBUG
    if (datalogCountInMainsCycles == 0 && beyondStartUpPhase) {
      printInfo(phase);
    }
#endif
    // End of each loop is for one pair of V & I measurements
  } else {
    loopCount++;
  }

  // Executed on every loop.
  //------------------------------------------------------------
  // phaseAngleTriacControl();

} // end of loop()

// helper function, to process LED events:
// can be conveniently called every 20ms, at the start of each mains cycle
/*
void checkLedStatus() {
#ifdef DEBUG
  ledState = OFF;
#else
  ledState = digitalRead (ledDetectorPin);
#endif

  if (ledState != prevLedState)
  {
    // led has changed state
    if (ledState == ON) {
      // led has just gone on
      ledOnAt = millis();
      ledRecentlyOnFlag = true;
    } else {
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
*/
