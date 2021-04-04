/* Mk2_3phase_RFdatalog_3.ino
 *
 * Issue 1 was released in January 2015.
 *
 * This sketch provides continuous monitoring of real power on three phases. 
 * Surplus power is diverted to multiple loads in sequential order.  A suitable
 * output-stage is required for each load; this can be either triac-based, or a 
 * Solid State Relay.  
 *
 * Datalogging of real power and Vrms is provided for each phase.
 * The presence or absence of the RFM12B needs to be set at compile time
 *
 * January 2016, renamed as Mk2_3phase_RFdatalog_2 with these changes:
 * - Improved control of multiple loads has been imported from the 
 *     equivalent 1-phase sketch, Mk2_multiLoad_wired_6.ino
 * - the ISR has been upgraded to fix a possible timing anomaly
 * - variables to store ADC samples are now declared as "volatile"
 * - for RF69 RF module is now supported
 * - a performance check has been added with the result being sent to the Serial port 
 * - control signals for loads are now active-high to suit the latest 3-phase PCB
 *
 * February 2016, renamed as Mk2_3phase_RFdatalog_3 with these changes:
 * - improvements to the start-up logic.  The start of normal operation is now 
 *    synchronised with the start of a new mains cycle.
 * - reduce the amount of feedback in the Low Pass Filter for removing the DC content
 *     from the Vsample stream. This resolves an anomaly which has been present since 
 *     the start of this project.  Although the amount of feedback has previously been 
 *     excessive, this anomaly has had minimal effect on the system's overall behaviour.
 * - The reported power at each of the phases has been inverted. These values are now in 
 *     line with the Open Energy Monitor convention, whereby import is positive and 
 *     export is negative.
 *
 *      Robin Emley
 *      www.Mk2PVrouter.co.uk
 */

#include <Arduino.h> // may not be needed, but it's probably a good idea to include this

// #define RF_PRESENT // <- this line should be commented out if the RFM12B module is not present

#ifdef RF_PRESENT
#define RF69_COMPAT 0 // for the RFM12B
// #define RF69_COMPAT 1 // for the RF69
#include <JeeLib.h>     
#endif

// In this sketch, the ADC is free-running with a cycle time of ~104uS.

//  WORKLOAD_CHECK is available for determining how much spare processing time there 
//  is.  To activate this mode, the #define line below should be included: 
// #define WORKLOAD_CHECK  

#define DEBUG

#define ON 1  // for use with LED( active high) or relay
#define OFF 0

#define CYCLES_PER_SECOND 50
//#define JOULES_PER_WATT_HOUR 3600 // may be needed for datalogging
#define WORKING_ZONE_IN_JOULES 3600
#define REQUIRED_EXPORT_IN_WATTS 0 // when set to a negative value, this acts as a PV generator 
#define NO_OF_PHASES 3
#define NO_OF_PHASE_RELAYS 2
#define DATALOG_PERIOD_IN_SECONDS 2

// const byte noOfDumploads = 3; 
const byte noOfDumploads = 1; 

enum polarities {NEGATIVE, POSITIVE};
enum outputModes {ANTI_FLICKER, NORMAL};
enum loadPriorityModes {LOAD_1_HAS_PRIORITY, LOAD_0_HAS_PRIORITY};
// enum loadPriorityModes {LOAD_0_HAS_PRIORITY};

// enum loadStates {LOAD_ON, LOAD_OFF}; // for use if loads are active low (original PCB)
enum loadStates {LOAD_OFF, LOAD_ON}; // for use if loads are active high (Rev 2 PCB)
enum loadStates logicalLoadState[noOfDumploads]; 
enum loadStates physicalLoadState[noOfDumploads]; 

// array defining to which phase which load is connected. 
const byte loadPhases[noOfDumploads] = {0};

// For this multi-load version, the same mechanism has been retained but the 
// output mode is hard-coded as below:
enum outputModes outputMode = ANTI_FLICKER;    

// In this multi-load version, the external switch is re-used to determine the load priority
enum loadPriorityModes loadPriorityMode = LOAD_0_HAS_PRIORITY;                                                   

#ifdef RF_PRESENT
#define freq RF12_433MHZ // Use the freq to match the module you have.

const int nodeID = 10;                                          
const int networkGroup = 210;                        
const int UNO = 1;
#endif
                            
// typedef struct { 
//   int power_L1;
//   int power_L2;
//   int power_L3; 
//   int Vrms_L1;
//   int Vrms_L2;
//   int Vrms_L3;} Tx_struct;    // revised data for RF comms
// Tx_struct tx_data;

typedef struct { 
  int power[NO_OF_PHASES];
  int Vrms[NO_OF_PHASES];} Tx_struct;    // revised data for RF comms
Tx_struct tx_data;


// ----------- Pinout assignments  -----------
//
// digital pins:
const byte loadPrioritySelectorPin = 3; // // for 3-phase PCB  
// D4 is not in use
// const byte physicalLoad_0_pin = 5; // for 3-phase PCB, Load #1 (Rev 2 PCB)
// const byte physicalLoad_1_pin = 6; // for 3-phase PCB, Load #2 (Rev 2 PCB)
// const byte physicalLoad_2_pin = 7; // for 3-phase PCB, Load #3 (Rev 2 PCB) 
const byte outputPinsForPAcontrol[noOfDumploads] = {5};
const unsigned char outputPinForPhaseRelayControl[NO_OF_PHASE_RELAYS] = {8, 9};
const unsigned char phaseWhichRelayControl[NO_OF_PHASE_RELAYS] = {1, 2};
const unsigned char phaseToWhichRelaysConnect[NO_OF_PHASE_RELAYS] = {0, 0};
unsigned char stateOfRelays[NO_OF_PHASE_RELAYS];

// D8 is not in use
// D9 is not in use

// analogue input pins
// ADC0 (pin 23) phase 1 V
// ADC1 (pin 24) phase 1 I
// ADC2 (pin 25) phase 2 V
// ADC3 (pin 26) phase 2 I
// ADC4 (pin 27) phase 3 V
// ADC5 (pin 28) phase 3 I
const byte sensorV[NO_OF_PHASES] = {B00000000, B00000010, B00000100}; // for 3-phase PCB
const byte sensorI[NO_OF_PHASES] = {B00000001, B00000011, B00000101}; // for 3-phase PCB 
// 1 phase:
// const byte sensorV[NO_OF_PHASES] = {B00000000}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001}; // for 3-phase PCB
// 2 phase
// const byte sensorV[NO_OF_PHASES] = {B00000000, B00000010}; // for 3-phase PCB
// const byte sensorI[NO_OF_PHASES] = {B00000001, B00000011}; // for 3-phase PCB


// --------------  general global variables -----------------
//
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, some variables need to be 'long'
//
boolean beyondStartUpPeriod = false;    // start-up delay, allows things to settle
byte initialDelay = 3;  // in seconds, to allow time to open the Serial monitor
byte startUpPeriod = 3;  // in seconds, to allow LP filter to settle

long DCoffset_V_long[NO_OF_PHASES];              // <--- for LPF
long DCoffset_V_min;               // <--- for LPF (min limit)
long DCoffset_V_max;               // <--- for LPF (max limit)
int DCoffset_I_nom;               // nominal mid-point value of ADC @ x1 scale

// for 3-phase use, with units of Joules * CYCLES_PER_SECOND
// float capacityOfEnergyBucket_main; 
// float energyInBucket_main; 
// float midPointOfEnergyBucket_main;
// float lowerThreshold_default;  
// float lowerEnergyThreshold;
// float upperThreshold_default;
// float upperEnergyThreshold;
// float offsetOfEnergyThresholdsInAFmode = 0.1; // <-- must not exceeed 0.4

int lastSampleV[NO_OF_PHASES];     // stored value from the previous loop (HP filter is for voltage samples only)
// float lastFilteredV[NO_OF_PHASES], filteredV[NO_OF_PHASES]; //  voltage values after HP-filtering to remove the DC offset

// float realPower[NO_OF_PHASES];
long realPower_long[NO_OF_PHASES];

// for improved control of multiple loads
boolean recentTransition = false;
byte postTransitionCount;
#define POST_TRANSITION_MAX_COUNT 3 // <-- allows each transition to take effect
//#define POST_TRANSITION_MAX_COUNT 50 // <-- for testing only
byte activeLoad = 0;

// for datalogging
int datalogCountInMainsCycles;
const int maxDatalogCountInMainsCycles = DATALOG_PERIOD_IN_SECONDS * CYCLES_PER_SECOND;
float energyStateOfPhase[NO_OF_PHASES]; // only used for datalogging
long energyStateOfPhase_long[NO_OF_PHASES]; // only used for datalogging
int samplesDuringThisMainsCycle[NO_OF_PHASES];
int lastSamplesDuringThisMainsCycle[NO_OF_PHASES];


// for interaction between the main processor and the ISR 
volatile byte dataReadyForPhase = NO_OF_PHASES; // Use byte for data ready from ADC and store phase to it. 3 - no data ready.
volatile int sampleV[NO_OF_PHASES];
volatile int sampleI[NO_OF_PHASES];
volatile unsigned long timeAtSampleVInMicros[NO_OF_PHASES];
volatile unsigned long lastTimeAtSampleVInMicros[NO_OF_PHASES];

long samplesV[100]; //for loging
long samplesI[100];

// items for phase-angle control of triac
boolean firstLoopOfHalfCycle[NO_OF_PHASES];
boolean phaseAngleTriggerActivated[noOfDumploads];
unsigned long timeAtTriggerActivated[noOfDumploads];
unsigned long timeAtStartOfHalfCycleInMicros[NO_OF_PHASES];
long firingDelayInMicros[NO_OF_PHASES];

// For a mechanism to check the continuity of the sampling sequence
// #define CONTINUITY_CHECK_MAXCOUNT 250 // mains cycles
// int mainsCycles_forContinuityChecker;
// int lowestNoOfSampleSetsPerMainsCycle;
unsigned long loopCount = 0;
unsigned long lastLoopCount = 0;
unsigned long loopCountOfHalfCycle = 0;
unsigned long lastLoopCountOfHalfCycle = 0;

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
const float powerCal[NO_OF_PHASES] = {0.043, 0.043, 0.043};
                        
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
const float phaseCal[NO_OF_PHASES] = {0.17, 0.17, 0.17}; // <- nominal values only
int phaseCal_int[NO_OF_PHASES];           // to avoid the need for floating-point maths

// For datalogging purposes, voltageCal has been added too.  Because the range of ADC values is 
// similar to the actual range of volts, the optimal value for this cal factor is likely to be
// close to unity. 
const float voltageCal[NO_OF_PHASES] = {1.00, 1.00, 1.00}; // compared with Fluke 77 meter

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// Although this sketch always operates in ANTI_FLICKER mode, it was convenient
// to leave this mechanism in place.
//
void configureParamsForSelectedOutputMode()
{  
  Serial.print(">>free RAM = ");
  Serial.println(freeRam());  // a useful value to keep an eye on
}
 
void setup()
{  
  delay (initialDelay * 1000); // allows time to open the Serial Monitor
  
//  Serial.begin(9600);   // initialize Serial interface
  Serial.begin(230400);   // initialize Serial interface
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("----------------------------------");
  Serial.println("Sketch ID:  Mk2_3phase_RFdatalog_3.ino");

  for(byte load = 0; load< noOfDumploads; load++) {
    pinMode(outputPinsForPAcontrol[load], OUTPUT);
    logicalLoadState[load] = LOAD_OFF;
#ifndef DEBUG
    Serial.println ((String) "Output Pin For PA control: " + outputPinsForPAcontrol[load]);
#endif
  }
//  updatePhysicalLoadStates(); // allows the logical-to-physical mapping to be changed
  
  for (byte load = 0; load < NO_OF_PHASE_RELAYS; load++) {
    pinMode(outputPinForPhaseRelayControl[load], OUTPUT);
    stateOfRelays[load] = OFF;
#ifndef DEBUG
    Serial.println ((String) "Output Pin For Phase Relay control: " + outputPinForPhaseRelayControl[load]);
#endif
  }

  for(byte load = 0; load< noOfDumploads; load++) {
    digitalWrite(outputPinsForPAcontrol[load], physicalLoadState[0]); // update the local load's state.      
  }

  pinMode(loadPrioritySelectorPin, INPUT); 
  digitalWrite(loadPrioritySelectorPin, HIGH); // enable the internal pullup resistor
  delay (100); // allow time to settle
  int pinState = digitalRead(loadPrioritySelectorPin);  // initial selection and
  loadPriorityMode = (enum loadPriorityModes)pinState;  //   assignment of priority mode
        
  for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  {
	  firstLoopOfHalfCycle[phase] = false;
    // When using integer maths, calibration values that have been supplied in  
    // floating point form need to be rescaled.  
    phaseCal_int[phase] = phaseCal[phase] * 256;  // for integer maths
    DCoffset_V_long[phase] = 512L * 256;  // nominal mid-point value of ADC @ x256 scale
//    realPower[phase] = 0.0;
    realPower_long[phase] = 0;
    firingDelayInMicros[phase] = 9999;
  }
     
  // Define operating limits for the LP filters which identify DC offset in the voltage 
  // sample streams.  By limiting the output range, these filters always should start up 
  // correctly.
  DCoffset_V_min = (long)(512L - 50) * 256; // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 50) * 256; // mid-point of ADC plus a working margin
  DCoffset_I_nom = 512;        // nominal mid-point value of ADC @ x1 scale

  // for the main energy bucket
  // capacityOfEnergyBucket_main = (float)WORKING_ZONE_IN_JOULES * CYCLES_PER_SECOND;
  // midPointOfEnergyBucket_main = capacityOfEnergyBucket_main * 0.5; // for resetting flexible thresholds
  // energyInBucket_main = 0; 
    
  Serial.println ("ADC mode:       free-running");  
  Serial.print ("requiredExport in Watts = ");
  Serial.println (REQUIRED_EXPORT_IN_WATTS);
  
  // Set up the ADC to be free-running 
  ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                 // Enable the ADC 
  
  ADCSRA |= (1<<ADATE);  // set the Auto Trigger Enable bit in the ADCSRA register.  Because 
                         // bits ADTS0-2 have not been set (i.e. they are all zero), the 
                         // ADC's trigger source is set to "free running mode".
                         
  ADCSRA |=(1<<ADIE);    // set the ADC interrupt enable bit. When this bit is written 
                         // to one and the I-bit in SREG is set, the 
                         // ADC Conversion Complete Interrupt is activated. 

  ADCSRA |= (1<<ADSC);   // start ADC manually first time 
  sei();                 // Enable Global Interrupts  

     
  char flag = 0;
  Serial.print ( "Extra Features: ");  
#ifdef WORKLOAD_CHECK  
  Serial.print ( "WORKLOAD_CHECK ");
  flag++;
#endif
  if (flag == 0) {
    Serial.print ("none"); }
  Serial.println ();
   
  for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  {  
    Serial.print ( "powerCal for L"); Serial.print(phase +1); 
      Serial.print (" =    "); Serial.println (powerCal[phase],4);
    Serial.print ( "phaseCal for L"); Serial.print(phase +1); 
      Serial.print (" =     "); Serial.println (phaseCal[phase]);
    Serial.print ( "voltageCal for L"); Serial.print(phase +1); 
      Serial.print (" =    "); Serial.println (voltageCal[phase],3);
  }
  Serial.println ((String)"DCoffset_V_min:\t" + DCoffset_V_min);
  Serial.println ((String)"DCoffset_V_max:\t" + DCoffset_V_max);

  // timeAtStartOfHalfCycleInMicros[0] = micros();
  // samplesV[0] = timeAtStartOfHalfCycleInMicros[0];
  // for (int i = 1; i < 100; i++) {
  //   samplesV[i] = micros();
  //   samplesI[i] = samplesV[i] - samplesV[i-1];
  // }
  // Serial.println ((String) "200 loop time in micros:\t" + (micros() - timeAtStartOfHalfCycleInMicros[0]));
  // for (int i = 0; i < 100; i++) {
  //   Serial.println ((String) "delta in micros:\t" + i + "\t" + samplesI[i]);
  // }
  Serial.println ("----");    

#ifdef WORKLOAD_CHECK
   Serial.println ("WELCOME TO WORKLOAD_CHECK ");
  
//   <<- start of commented out section, to save on RAM space!
   
   Serial.println ("  This mode of operation allows the spare processing capacity of the system");
   Serial.println ("to be analysed.  Additional delay is gradually increased until all spare time");
   Serial.println ("has been used up.  This value (in uS) is noted and the process is repeated.  ");
   Serial.println ("The delay setting is increased by 1uS at a time, and each value of delay is ");
   Serial.println ("checked several times before the delay is increased. "); 
  
//  <<- end of commented out section, to save on RAM space!

   Serial.println ("  The displayed value is the amount of spare time, per set of V & I samples, ");
   Serial.println ("that is available for doing additional processing.");
   Serial.println ();
 #endif
 
   configureParamsForSelectedOutputMode(); 

   Serial.print ("loadPriority: ");
   if (loadPriorityMode == LOAD_0_HAS_PRIORITY) {
     Serial.println ( "load 0"); }
   else {  
     Serial.println ( "load 1"); }

   Serial.println();
   Serial.print ("free RAM = ");
   Serial.println (freeRam());
   
   Serial.print ("RF capability ");
   
#ifdef RF_PRESENT
   Serial.print ("IS present, freq = ");
   if (freq == RF12_433MHZ) { Serial.println ("433 MHz"); }
   if (freq == RF12_868MHZ) { Serial.println ("868 MHz"); }
  rf12_initialize(nodeID, freq, networkGroup);          // initialize RF
#else
   Serial.println ("is NOT present");
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
      lastTimeAtSampleVInMicros[0] = timeAtSampleVInMicros[0];
      timeAtSampleVInMicros[0] = micros();
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
      lastTimeAtSampleVInMicros[1] = timeAtSampleVInMicros[1];
      timeAtSampleVInMicros[1] = micros();
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
      lastTimeAtSampleVInMicros[2] = timeAtSampleVInMicros[2];
      timeAtSampleVInMicros[2] = micros();
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

void phaseAngleTriacControl() {
    // ********************************************************
    // start of section to support phase-angle control of triac
    // controls the signal for firing the direct-acting trigger.
  for (byte load = 0; load < noOfDumploads; load++) {
    unsigned long timeNowInMicros = micros(); // occurs every loop, for consistent timing
    byte loadphase = loadPhases[load];
    if (loopCountOfHalfCycle < 100) {
      samplesV[loopCountOfHalfCycle] = 0;
//      samplesV[samplesDuringThisMainsCycle[0]] = loopCountOfHalfCycle;
      samplesI[loopCountOfHalfCycle] = 0;
    }

    if (firstLoopOfHalfCycle[loadphase] == true) {
//      timeAtStartOfHalfCycleInMicros[loadphase] = timeNowInMicros;
      firstLoopOfHalfCycle[loadphase] = false;
      phaseAngleTriggerActivated[load] = false;
      // Unless dumping full power, release the trigger on the first loop in each
      // half cycle.  Ensures that trigger can't get stuck 'on'.
//      if (firingDelayInMicros[loadphase] > 100) {
        digitalWrite(outputPinsForPAcontrol[load], LOAD_OFF);
//        if (loopCount < 100) {
//          samplesI[loopCountOfHalfCycle] = loopCountOfHalfCycle;
//        }
//      }
//      Serial.println("FirstLoopOfHalfCycle");
//      if (samplesDuringThisMainsCycle[0] < 100) {
//        samplesV[samplesDuringThisMainsCycle[0]] = loopCountOfHalfCycle;
//      }
      // Break this load till at least next loop: 
      break;
    }

    if (phaseAngleTriggerActivated[load] == true) {
      // Unless dumping full power, release the trigger on all loops in this
      // half cycle after the one during which the trigger was set.
//      if (firingDelayInMicros[loadphase] > 100) {
      if (timeNowInMicros >= (timeAtTriggerActivated[load] + 2000)) {
        digitalWrite(outputPinsForPAcontrol[load], LOAD_OFF);
      }
        // if (loopCount < 100) {
        //   samplesI[loopCountOfHalfCycle] = timeNowInMicros - timeAtStartOfHalfCycleInMicros[loadphase];
        // }
//      }
    } else {
      if (timeNowInMicros >= (timeAtStartOfHalfCycleInMicros[loadphase] + firingDelayInMicros[loadphase])) {
        digitalWrite(outputPinsForPAcontrol[load], LOAD_ON);
        phaseAngleTriggerActivated[load] = true;
        timeAtTriggerActivated[load] = timeNowInMicros;
        // if (samplesDuringThisMainsCycle[0] < 100) {
        //   samplesV[samplesDuringThisMainsCycle[0]] = loopCountOfHalfCycle;
        // }
        // if (loopCountOfHalfCycle < 100) {
        //   samplesV[loopCountOfHalfCycle] = timeNowInMicros - timeAtStartOfHalfCycleInMicros[loadphase];
        // }
      }
    }
    // end of section to support phase-angle control of triac
    //*******************************************************
  }
}

void calculateFiringDelay(byte phase) {
    // ********************************************************
     // start of section to support phase-angle control of triac
     // determines the correct firing delay for a direct-acting trigger
	 enum loadStates phaseLoadState = LOAD_OFF;

	 // The idea is to have autoregulation to minimize power output to grid.
	 // Power must be only imported or 0
	 // Assume realEnergy[phase]  is negative if we exporting.
	 if (realPower_long[phase] < 0) {
		 phaseLoadState = LOAD_ON;
		 // if firingDelayInMicros[phase] = 0 - trac i sfully opened.
		 if (firingDelayInMicros[phase] > 0) {
			 // TODO dynamic step calculation.
			 firingDelayInMicros[phase] = firingDelayInMicros[phase] - 10;
		 }
	 } else {
		 // now we importing enegry from grid, so lover diverting increasing delay:
//		 if (realPower[phase] > realLastPower[phase]) {
			 // TODO dynamic step calculation.
			 firingDelayInMicros[phase] = firingDelayInMicros[phase] + 10;
//		 }
	 }

	 if (firingDelayInMicros[phase] < 0) {
		 firingDelayInMicros[phase] = 0;
	 }
	 if (firingDelayInMicros[phase] > 9999) {
		 firingDelayInMicros[phase] = 9999;
	 }
     // Set logicalLoadState of load to ON if load is diverting:
     for (unsigned char load = 0; load < noOfDumploads; load++) {
    	 if (loadPhases[load] == phase) {
    		 logicalLoadState[load] = phaseLoadState;
    	 }
     }
     // end of section to support phase-angle control of triac
     //*******************************************************
}

// This routine is called to process each set of V & I samples (3 pairs).  The main processor and 
// the ADC work autonomously, their operation being synchnonised only via the dataReady flag.  
//
void processRawSamples(byte phase) {
  static long sumP[NO_OF_PHASES];
  static enum polarities polarityOfLastSampleV[NO_OF_PHASES];  // for zero-crossing detection
  static long lastSampleV_minusDC_long[NO_OF_PHASES];     //    for the phaseCal algorithm
  static long phaseShiftedSampleV_minusDC_long[NO_OF_PHASES];
  static long lastPhaseShiftedSampleV_minusDC_long[NO_OF_PHASES];
  static long cumVdeltasThisCycle_long[NO_OF_PHASES];    // for the LPF which determines DC offset (voltage)
  static long sum_Vsquared_long[NO_OF_PHASES];
  static long lastSum_Vsquared_long[NO_OF_PHASES];

  static long sum_Vsquared[NO_OF_PHASES];
  static long lastSum_Vsquared[NO_OF_PHASES];
  static long samplesDuringThisDatalogPeriod;
  enum polarities polarityNow;  
  
  // The raw V and I samples are processed in "phase pairs"
  //for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  //{
    // remove DC offset from each raw voltage sample by subtracting the accurate value 
    // as determined by its associated LP filter.
    long sampleV_minusDC_long = (((long)sampleV[phase])<<8) - DCoffset_V_long[phase]; 

    // phase-shift the voltage waveform so that it aligns with the current when a 
    // resistive load is used
    phaseShiftedSampleV_minusDC_long[phase] = lastSampleV_minusDC_long[phase]
         + (((sampleV_minusDC_long - lastSampleV_minusDC_long[phase]) * phaseCal_int[phase])>>8); 

    // a high-pass filter is used just for determining the start of each mains cycle
    // filteredV[phase] = 0.996 * (lastFilteredV[phase] + sampleV[phase] - lastSampleV[phase]);

    // determine polarity, to aid the logical flow  
//    if (filteredV[phase] >= 0) {
//    if (sampleV_minusDC_long > 0) {
    if (phaseShiftedSampleV_minusDC_long[phase] >= 0) {
      polarityNow = POSITIVE; }
    else { 
      polarityNow = NEGATIVE; }

    if (polarityNow == POSITIVE) {                           
      if (polarityOfLastSampleV[phase] != POSITIVE) {
        // Find exact time then voltage crossed zero:
        int vDelta = (int)(lastPhaseShiftedSampleV_minusDC_long[phase] - phaseShiftedSampleV_minusDC_long[phase]);
        int timeDelta = (int)(lastTimeAtSampleVInMicros[phase] - timeAtSampleVInMicros[phase]);
        timeAtStartOfHalfCycleInMicros[phase] = lastTimeAtSampleVInMicros[phase] + (0 - lastPhaseShiftedSampleV_minusDC_long[phase]) * (timeDelta/vDelta); 
        // timeAtStartOfHalfCycleInMicros[phase] = lastTimeAtSampleVInMicros[phase];
        

        if (beyondStartUpPeriod) {  
          // This is the start of a new +ve half cycle, for this phase, just after the 
          // zero-crossing point.  Before the contribution from this phase can be added 
          // to the running total, the cal factor for this phase must be applied. 
          //
          firstLoopOfHalfCycle[phase] = true;
          if (phase == 0) {
            lastLoopCountOfHalfCycle = loopCountOfHalfCycle;
            loopCountOfHalfCycle = 0;
          }
          realPower_long[phase] = (sumP[phase] / samplesDuringThisMainsCycle[phase]) / 20;
//          realPower[phase] = (sumP[phase] / samplesDuringThisMainsCycle[phase]) * powerCal[phase];
          
          energyStateOfPhase_long[phase] += realPower_long[phase];  
          calculateFiringDelay(phase);
          // A performance check to monitor and display the minimum number of sets of
          // ADC samples per mains cycle, the expected number being 20ms / (104us * 6) = 32.05
          //
          // if (phase == 0)
          // {
          //   if (samplesDuringThisMainsCycle[phase] < lowestNoOfSampleSetsPerMainsCycle)
          //   {
          //     lowestNoOfSampleSetsPerMainsCycle = samplesDuringThisMainsCycle[phase];
          //   }
          //   mainsCycles_forContinuityChecker++;
          //   if (mainsCycles_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT)
          //   {
          //     mainsCycles_forContinuityChecker = 0;
          //     Serial.println(lowestNoOfSampleSetsPerMainsCycle);
          //     lowestNoOfSampleSetsPerMainsCycle = 999;
          //   }              
          // }
         
          sumP[phase] = 0;
          lastSamplesDuringThisMainsCycle[phase] = samplesDuringThisMainsCycle[phase];
          samplesDuringThisMainsCycle[phase] = 0;
          lastSum_Vsquared_long[phase] = sum_Vsquared_long[phase];
          sum_Vsquared_long[phase] = 0;
        } // end of processing that is specific to the first Vsample in each +ve half cycle   
        else
        {  
          // wait until the DC-blocking filters have had time to settle
          if(millis() > (initialDelay + startUpPeriod) * 1000) 
          {
            beyondStartUpPeriod = true;
            Serial.println ("Go!");
          }
        }
      }  // end of processing that is specific to the first Vsample in each +ve half cycle  
       
      // still processing samples where the voltage is POSITIVE ...
      // check to see whether the trigger device can now be reliably armed
      if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 2) && beyondStartUpPeriod) // lower value for larger sample set 
      {
        // This code is executed once per 20mS, shortly after the start of each new
        // mains cycle on phase 0.
        //
        datalogCountInMainsCycles++;
            
        if (datalogCountInMainsCycles >= maxDatalogCountInMainsCycles) {
          datalogCountInMainsCycles = 0;
          for (byte si = 0; si < NO_OF_PHASES; si++) {
            tx_data.power[si] = energyStateOfPhase_long[si] / maxDatalogCountInMainsCycles;
            tx_data.power[si] *= -1; // to match the OEM convention (import is =ve; export is -ve)
            tx_data.Vrms[si] = (int)(voltageCal[si] * sqrt(sum_Vsquared[si] / samplesDuringThisDatalogPeriod));
          }
#ifdef RF_PRESENT
          send_rf_data();         
#endif
          for (byte si = 0; si < NO_OF_PHASES; si++) {
            energyStateOfPhase_long[si] = 0;
            lastSum_Vsquared[si] = sum_Vsquared[si];
            sum_Vsquared[si] = 0;
          }
          samplesDuringThisDatalogPeriod = 0;
        }  
      }

#ifdef DEBUG
/*     <-- Warning - Unlike its 1-phase equivalent, this 3-phase code can be affected by Serial statements! */
      if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 3) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) { // lower value for larger sample set 
        Serial.println(firingDelayInMicros[0]);
        Serial.println(lastLoopCountOfHalfCycle);
        Serial.println(loopCountOfHalfCycle);
        // for (int i = 0; i < 100; i++) {
        //   Serial.print((String) "loop:\t" + i + " \t" + samplesV[i] + " \t");
        // }
      }
      // if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 3) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) { // lower value for larger sample set
      //   Serial.print("power:\t\t");
      //   for (byte si = 0; si < NO_OF_PHASES; si++) {
      //     Serial.print((String) "\tphase: " + si + " :\t" + tx_data.power[si]);
      //   }
      //   Serial.println();
      // }
      // if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 4) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) { // lower value for larger sample set
      //   for (byte si = 0; si < NO_OF_PHASES; si++) {
      //     Serial.print((String) "\tphase: " + si + "\tVrms : " + tx_data.Vrms[si]);
      //   }
      //   Serial.println();
      // }
      if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 4) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) { // lower value for larger sample set
        for (byte si = 0; si < NO_OF_PHASES; si++) {
          Serial.print((String) "\tphase: " + si + "\tsum_Vsquared : " + lastSum_Vsquared_long[si]/lastSamplesDuringThisMainsCycle[si]);
        }
        Serial.println();
      }
      // if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 4) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) // lower value for larger sample set 
      // {
      //   Serial.print("firingDelayInMicros");
      //   for (byte i = 0; i < NO_OF_PHASES; i++)
      //   {            
      //     Serial.print((String)"\tphase: " + i + " :\t" + firingDelayInMicros[i]);
      //   } 
      //   Serial.println();
      // }
      // if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 5) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) // lower value for larger sample set 
      // {
      //   Serial.print("lastSamplesDuringThisMainsCycle:");
      //   for (byte si = 0; si < NO_OF_PHASES; si++) {
      //     Serial.print((String) "\tphase: " + si + "\tsamples: " + lastSamplesDuringThisMainsCycle[si]);
      //   }
      //   Serial.println();
      // }
      if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 5) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) { // lower value for larger sample set
        Serial.print("realPower_long\t");
        for (byte si = 0; si < NO_OF_PHASES; si++) {            
          Serial.print((String)"\tphase: " + si + " :\t" + realPower_long[si]);
        } 
        Serial.println();
      }
      if ((phase == 0) && (samplesDuringThisMainsCycle[0] == 6) && beyondStartUpPeriod && (datalogCountInMainsCycles == 0)) { // lower value for larger sample set
        for (int i = 0; i < noOfDumploads; i++)
        {            
          Serial.println((String)"logicalLoadState of Dumpload\t" + i + " :\t" + logicalLoadState[i]);
        } 
        Serial.println();
      }
#endif           
    } // end of processing that is specific to samples where the voltage is positive
    else // the polarity of this sample is negative
    {     
      if (polarityOfLastSampleV[phase] != NEGATIVE) {
        int vDelta = (int)(lastPhaseShiftedSampleV_minusDC_long[phase] - phaseShiftedSampleV_minusDC_long[phase]);
        int timeDelta = (int)(lastTimeAtSampleVInMicros[phase] - timeAtSampleVInMicros[phase]);
        timeAtStartOfHalfCycleInMicros[phase] = lastTimeAtSampleVInMicros[phase] + (0 - lastPhaseShiftedSampleV_minusDC_long[phase]) * (timeDelta/vDelta); 
        // timeAtStartOfHalfCycleInMicros[phase] = lastTimeAtSampleVInMicros[phase];
        if (beyondStartUpPeriod) {  
          firstLoopOfHalfCycle[phase] = true;
          if (phase == 0) {
            lastLoopCountOfHalfCycle = loopCountOfHalfCycle;
            loopCountOfHalfCycle = 0;
          }
        }
        // This is the start of a new -ve half cycle (just after the zero-crossing point)
        // This is a convenient point to update the Low Pass Filter for removing the DC
        // component from the phase that is being processed.  
        // The portion which is fed back into the integrator is approximately one percent
        // of the average offset of all the Vsamples in the previous mains cycle.  
        //
        DCoffset_V_long[phase] += (cumVdeltasThisCycle_long[phase]>>12); 
        cumVdeltasThisCycle_long[phase] = 0;
      
        // To ensure that this LP filter will always start up correctly when 240V AC is 
        // available, its output value needs to be prevented from drifting beyond the likely range 
        // of the voltage signal.  
        //
        if (DCoffset_V_long[phase] < DCoffset_V_min) {  
          DCoffset_V_long[phase] = DCoffset_V_min;
        } else if (DCoffset_V_long[phase] > DCoffset_V_max) {
          DCoffset_V_long[phase] = DCoffset_V_max;
        }
      } // end of processing that is specific to the first Vsample in each -ve half cycle
    } // end of processing that is specific to samples where the voltage is negative
  
    // Processing for EVERY pair of samples. Most of this code is not used during the 
    // start-up period, but it does no harm to leave it in place.  Accumulated values 
    // are cleared when the beyondStartUpPhase flag is set to true.
    //
    // remove most of the DC offset from the current sample (the precise value does not matter)
    long sampleIminusDC_long = ((long)(sampleI[phase] - DCoffset_I_nom))<<8;
   
    // calculate the "real power" in this sample pair and add to the accumulated sum
    long filtV_div4 = phaseShiftedSampleV_minusDC_long[phase]>>2;  // reduce to 16-bits (now x64, or 2^6)
    long filtI_div4 = sampleIminusDC_long>>2; // reduce to 16-bits (now x64, or 2^6)
    long instP = filtV_div4 * filtI_div4;  // 32-bits (now x4096, or 2^12)
    instP = instP>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)       
    sumP[phase] +=instP; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
 
    // for the Vrms calculation (for datalogging only)
    long inst_Vsquared = filtV_div4 * filtV_div4; // 32-bits (now x4096, or 2^12)
    inst_Vsquared = inst_Vsquared>>12;     // scaling is now x1 (V_ADC x I_ADC)
    sum_Vsquared[phase] += inst_Vsquared; // cumulative V^2 (V_ADC x I_ADC)
    sum_Vsquared_long[phase] += inst_Vsquared; 
    if (phase == 0) {
      samplesDuringThisDatalogPeriod ++;
      // if (samplesDuringThisMainsCycle[0] < 50) {
      //   samplesV[samplesDuringThisMainsCycle[0]] = phaseShiftedSampleV_minusDC_long;
      //   samplesI[samplesDuringThisMainsCycle[0]] = (long)(sampleI[0] - DCoffset_I_nom);
      // }
    } // no need to keep separate counts for each phase
   
    // general housekeeping
    cumVdeltasThisCycle_long[phase] += sampleV_minusDC_long; // for use with LP filter
    samplesDuringThisMainsCycle[phase] ++;
    
    // store items for use during next loop
    lastSampleV[phase] = sampleV[phase];            // for digital high-pass filter
    lastPhaseShiftedSampleV_minusDC_long[phase] = phaseShiftedSampleV_minusDC_long[phase];
    // lastFilteredV[phase] = filteredV[phase];      // for HPF, used to identify the start of each mains cycle
    lastSampleV_minusDC_long[phase] = sampleV_minusDC_long;  // required for phaseCal algorithm
    polarityOfLastSampleV[phase] = polarityNow;  // for identification of half cycle boundaries
//  }
}
// end of processRawSamples()


// The main processor waits in loop() until the DataReady flag has been set by the ADC.  
// Once this flag has been set, the main processor clears the flag and proceeds with 
// the processing for a complete set of 3 pairs of V & I samples.  It then returns to 
// loop() to wait for the next set to become available.
//   If the next set of samples become available before the processing of the previous set 
// has been completed, data could be lost.  This situation can be avoided by prior use of 
// the WORKLOAD_CHECK mode.  Using this facility, the amount of spare processing capacity 
// per 6-sample set can be determined.  
//
void loop() {
#ifdef WORKLOAD_CHECK
  static int del = 0; // delay, as passed to delayMicroseconds()
  static int res = 0; // result, to be displayed at the next opportunity
  static byte count = 0; // to allow multiple runs per setting
  static byte displayFlag = 0; // to determine when printing may occur
#endif
  
  if (dataReadyForPhase < NO_OF_PHASES) { // flag is set after every pair of ADC conversions
 	  byte phase = dataReadyForPhase;
	  dataReadyForPhase = NO_OF_PHASES; 		// clear dataready flag.
    processRawSamples(phase); // executed once for each pair of V&I samples
    
#ifdef WORKLOAD_CHECK 
    delayMicroseconds(del); // <--- to assess how much spare time there is
    if (dataReadyForPhase < NO_OF_PHASES)       // if data is ready again, delay was too long
    { 
      res = del;             // note the exact value
      del = 1;               // and start again with 1us delay   
      count = 0;
      displayFlag = 0;   
    }
    else
    {
      count++;          // to give several runs with the same value
      if (count > 50)
      {
        count = 0;
        del++;          //  increase delay by 1uS
      } 
    }
#endif
    lastLoopCount = loopCount;
    loopCount = 0;
  } else {  // <-- this closing brace needs to be outside the WORKLOAD_CHECK blocks!

  }
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
  // Executed on every loop.
  //------------------------------------------------------------
  phaseAngleTriacControl();
  loopCount++;
  loopCountOfHalfCycle++;
} // end of loop()
 
#ifdef RF_PRESENT
void send_rf_data()
{
  int i = 0; 
  while (!rf12_canSend() && i<10)
  { 
    rf12_recvDone(); 
    i++;
  }
  rf12_sendStart(0, &tx_data, sizeof tx_data);
}
#endif
