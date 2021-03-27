/*
 * Emonitor_Core.cpp
 *
 * Created: 06/03/2012 10:30:58
 *  Author: pcunha
 */ 

#include "Arduino.h"
#define F_CPU 20000000L
#include <JeeLib.h>  
#include <stdio.h>
#include <avr/eeprom.h>


//------------------------------------------------------------------------------------
//DEFINES
//------------------------------------------------------------------------------------
#define CONFIG_EEPROM_ADDR ((byte*) 0x10)

#define SERIAL 0

#define VERSION 3
#define NODE_TYPE 0x01


#define CALC_INTERVAL 2 //in seconds - time wich arduino will do power calculations.
#define FREQUENCY 60 // IN HZ
//How many CTS for the main breaker (may not be on the same phase)
#define  PHASE_SENSORS  3
// how many voltage sensors do you have (Open energy monitor dafaults to 1)
#define  VOLTAGE_SENSORS 1
//Phase shift angle between phases (defaults to 120 for 3 phase circuits)
#define  PHASE_SHIFT  = 120
//samples per second are F_CPU / ADC prescaler(128) / 13 clock cycles per adc / channel count
#define SAMPLES_PER_SECOND (F_CPU / 128 / 13 / (PHASE_SENSORS + VOLTAGE_SENSORS))

//SAMPLES_PER_SECOND are F_CPU / ADC prescaler(128) / 13 clock cycles per adc / channel count
//the buffer size is samples per second / degrees to go back
//for 3-phase circuits, the degree is 240
//for 2-phase is 90 - i cannot test this...
//for 2-phases of a 3-phase system in theory is 120. 

//MAX_SHIFT_DEGREE = 240
#define ADC_BUFFER_SIZE  33 // ((((SAMPLES_PER_SECOND/FREQUENCY) * MAX_SHIFT_DEGREE) / 360)  +1);
#define PHASE1_OFFSET 16
#define PHASE2_OFFSET 32


/*
Input pins
[index] - Arduino AD
Example: for 3-phase with 1 voltage sensors:
[0] Phase1 voltage
[1] Phase1 current
[2] Phase2 current
[3] Phase3 current

Example: for 3-phase with 3 voltage sensors:
[0] Phase1 voltage
[1] Phase1 current
[2] Phase2 voltage
[3] Phase2 current
[4] Phase3 voltage
[5] Phase3 current

*/


/*
Pooling Order:
3-Phase voltages and 3-CT:
Voltage1, Current1, Voltage2, Current2, V3, C3, V1, C1...
1Phase Voltage - 3 Phase CT:
Voltage1 - Current1, Current2, Current3, Voltage1...
*/

unsigned char input_analog_pins[PHASE_SENSORS + VOLTAGE_SENSORS] = {2,3,0,1};

struct Config {
    byte band;
    byte group;
    byte nodeID;
    byte sendTo;
    byte transmitinterval;
    float Phase_Calibration[3];
    float Current_Calibration[3];
    float Voltage_Calibration[3];
    float Voltage;
    byte valid; // keep this as last byte
} config;

static void ConfigDefaults() {
        config.valid = 253;
        config.band = 4;
        config.group = 210;
        config.nodeID = 10;
        config.sendTo = 1;
        config.transmitinterval = 3;
        config.Phase_Calibration[0] = 1.7;
        config.Phase_Calibration[1] = 1.7;
        config.Phase_Calibration[2] = 1.7;
        config.Current_Calibration[0] = 0.1080;
        config.Current_Calibration[1] = 0.1080;
        config.Current_Calibration[2] = 0.1080;
        config.Voltage_Calibration[0] = 0.4581;
        config.Voltage_Calibration[1] = 0.4581;
        config.Voltage_Calibration[2] = 0.4581;
        config.Voltage = 127.0;
}


//End of User defineable variables
//---------------------------------------------------
// Do not touch the code below this point, unless you know what you are doing
//---------------------------------------------------



static void ConfigDefaults();
static void loadConfig();
static void saveConfig();
void initRF();
void process_rf();

//tx data - 62 bytes in total 
typedef struct { 
  byte nodeId;             //1 Byte
  byte command;            //1 Byte
  unsigned int txCount;    //2 bytes
  float totalP;            //4 bytes
  float cummKw;            //4 bytes
  unsigned char numV;      //1 Byte
  unsigned char numI;      //1 Byte
  float V1;                //4 bytes
  float Irms1;             //4 bytes
  float RP1;               //4 bytes
  float PF1;               //4 bytes
  float V2;                //4 bytes
  float Irms2;             //4 bytes
  float RP2;               //4 bytes
  float PF2;               //4 bytes
  float V3;                //4 bytes
  float Irms3;             //4 bytes
  float RP3;               //4 bytes
  float PF3;               //4 bytes
} PayloadTX;      // create structure - a neat way of packaging data for RF comms

//1 voltage sensor
#if (VOLTAGE_SENSORS == 1)
  #define V0 0
  #if (PHASE_SENSORS > 0)
    #define I0 1
    #if (PHASE_SENSORS > 1)
       #define I1 2
       #if (PHASE_SENSORS > 2)
        #define I2 3
        #endif
    #endif
  #endif
//2 voltage sensors
#elif (VOLTAGE_SENSORS == 2)
  #define V0 0
  #if (PHASE_SENSORS > 0)
    #define I0 1
    #define V1 2
    #if (PHASE_SENSORS > 1)
      #define I1 3
      #if (PHASE_SENSORS > 2)
        #define I2 4
      #endif
    #endif
  #else
    #define V1 1
  #endif
//3 voltage sensors
#elif (VOLTAGE_SENSORS == 3)
  #define V0 0
  #if (PHASE_SENSORS == 1)
    #define I0 1
    #define V1 2
    #define V2 3
  #elif (PHASE_SENSORS == 2)
    #define I0 1
    #define V1 2
    #define I1 3
    #define V2 4
  #elif (PHASE_SENSORS == 3)
    #define I0 1
    #define V1 2
    #define I1 3
    #define V2 4
    #define I2 5
  #else
    #define V1 1
    #define V2 2
  #endif
#else
  //no voltage sensors
  #if (PHASE_SENSORS > 0)
    #define I0 0
    #if (PHASE_SENSORS > 1)
       #define I1 1
       #if (PHASE_SENSORS > 2)
         #define I2 2
       #endif
    #endif
  #endif
#endif


//Temp variables
PayloadTX emontx; 
long adcval;
volatile unsigned char AdcIndex = 1;   //keeps track of sampled channel
volatile unsigned int  SampleCounter, crossCounter, LastSampleCounter;
volatile boolean Cycle_Full; //stores wherever the cycle has been computed
boolean lastVCross, checkVCross;    //Used to measure number of times threshold is crossed.
unsigned char tempidx;

struct        CurrentSampleS {
                float New, Previous;
                float Filtered,PreviousFiltered, Calibrated, Sum, InstPower;
              }  CurrentSample[3]; //for now i will maintain 3 here hardcoded because i dont want to put 300 ifs on the code yet.

struct        VoltageSampleS {
                float New, Previous;
                float Filtered,PreviousFiltered, PhaseShifted, Calibrated, Sum;
              } VoltageSample[3]; //Yes phases here, because we are going to create the voltage data based on what we have //for now i will maintain 3 here hardcoded because i dont want to put 300 ifs on the code yet.
             
struct        LastCurrentS {
                float Sum, InstPower;
              } LastCurrent[3];//for now i will maintain 3 here hardcoded because i dont want to put 300 ifs on the code yet.
struct        LastVoltageS {
                float  Sum;
              } LastVoltage[VOLTAGE_SENSORS];          

//struct  AccDataStructure    { float Sum;
//                              float InstantPower;
//                            } AccData[2], LastAccData[2];

float AccVrms[3];
float AccIrms[3];
float AccRealPower[3] ;
float AccAparentPower[3];
float AccPowerFactor[3];

float cummP;  //cummulative power

//TODO fix if the 3 sensors are on the same phase, puto some code here
#if VOLTAGE_SENSORS == 1 && PHASE_SENSORS > 1
  #define ADC_BUFFER 1
  //declare voltage buffer
  float ADC_V_BUFFER[ADC_BUFFER_SIZE+2];
  unsigned char adc_buffer_index = (ADC_BUFFER_SIZE-1);
#endif

boolean temp = 0;
float tempdbl = 0.0;
unsigned char tmpchar;
volatile unsigned long LastInterruptTime;

unsigned char tcount;
unsigned int counts;



static void loadConfig() {
    for (byte i = 0; i < sizeof config; ++i)
        ((byte*) &config)[i] = eeprom_read_byte(CONFIG_EEPROM_ADDR + i);
    if (config.valid != 253) {
      ConfigDefaults();
    }

} 

static void saveConfig() {
    for (byte i = 0; i < sizeof config; ++i)
        eeprom_write_byte(CONFIG_EEPROM_ADDR + i, ((byte*) &config)[i]);
}

void initRF(){
byte freq = config.band == 4 ? RF12_433MHZ :
                config.band == 8 ? RF12_868MHZ :
                                   RF12_915MHZ;
  #ifdef SERIAL
    Serial.print("Initialiazing RF: Node:");                                  
    Serial.print(config.nodeID);     
    Serial.print(" Freq:");
    Serial.print(freq);     
    Serial.print(" group:");  
    Serial.println(config.group);
  #endif  
  rf12_initialize(config.nodeID, freq, config.group);  
  
}


void setup()
{
  // Check source of reset
  if (MCUSR & 1)               // Power-on Reset
  {
    // put POR handler here, if required
  }
  else if (MCUSR & 2)          // External Reset
  {
    // put external reset handler here, if required
  }
  else if (MCUSR & 4)          // Brown-Out Reset
  {
    // put BOR handler here, if required
  }
  else                          // Watchdog Reset
  {
    // put watchdog reset handler here, if required
  };
  
  #ifdef SERIAL
    Serial.begin(57600);     //begin Serial comm
    Serial.println(""); 
    delay(100);
    Serial.println("emonTX UP and Running"); 
    delay(500);
  #endif
  loadConfig();
  
  initRF();
   
  emontx.numV = VOLTAGE_SENSORS;
  emontx.numI = PHASE_SENSORS;
  
  // ADC initialization:
  // Enable ADC interrupt and use external voltage reference.
  // Set multiplexer input to first channel and do not set
  // ADC to High Speed Mode. Set Auto Trigger Source to Free Running Mode.
  //
  //  fCLK | ADPS | Prescaler | ADCCLK | Sampl.Rate
  // ------+------+-----------+--------+-----------
  // 4.000 |  111 |       128 | 125000 |       9615

  //ADC INIT
  ADMUX = (input_analog_pins[0] & 0x07); // Set ADC reference to external VFREF and first defined port
  ADCSRA |= (1 << ADEN);  // Enable ADC
  ADCSRA |= (1 << ADATE); // Enable auto-triggering
  ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
  
  sei();		     // Enable Global Interrupts

  //put code here to start adc after zero crossing. *TODO*  but not a problem, because we do not pick up first sampled data...
  
  //Start ADC 
  Serial.println("Open Energy Monitor Hack by Pcunha");
  Serial.println("Starting ADC in next cycle");
  ADCSRA=0xEF;   // Enable ADC, start, auto trigger, int enable, presc = 128 
  
}


//------------------------------------------------------------------------------------
//ADC INTERRUPT SERVICE ROUTINE
//------------------------------------------------------------------------------------
ISR(ADC_vect){		//ADC interrupt
//unsigned long t1;
unsigned char IndexAtual = 0;
//t1 = micros();

unsigned char nextindex = AdcIndex;
if (++nextindex >= (PHASE_SENSORS + VOLTAGE_SENSORS))  nextindex=0;
ADMUX=(input_analog_pins[nextindex]);// Select next ADC input

//
if (AdcIndex > 0) 
	IndexAtual = AdcIndex - 1;
else	
	IndexAtual = PHASE_SENSORS + VOLTAGE_SENSORS -1;
  //IndexAtual = (PHASE_SENSORS + VOLTAGE_SENSORS - 1); // adc is always 1 nextindex negative


//Voltage1 - First voltage sensor (Open Energy Monitor only have this voltage sensor) 
#ifdef V0
	if (IndexAtual == V0){ //Very strange behavior when using ifdef here with 0 value
		// Store Last Values for future use
		VoltageSample[0].Previous = VoltageSample[0].New;        //Put last sample on its place
		VoltageSample[0].PreviousFiltered = VoltageSample[0].Filtered;        //Put last sample on its place
		// Read in raw voltage and current samples
		VoltageSample[0].New=ADC;      //Read in raw voltage signal from index sample
		// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
		VoltageSample[0].Filtered = 0.996 * ( VoltageSample[0].PreviousFiltered + VoltageSample[0].New - VoltageSample[0].Previous );
		// Root-mean-square method Index Channel
		// -> sum the square of voltage values
		VoltageSample[0].Sum  += VoltageSample[0].Filtered * VoltageSample[0].Filtered; 
		// Phase calibration
		VoltageSample[0].PhaseShifted = VoltageSample[0].PreviousFiltered + config.Phase_Calibration[0] * (VoltageSample[0].Filtered - VoltageSample[0].PreviousFiltered);  
		//If using ADC BUFFERING
		//unsigned int ADC_V_BUFFER[ADC_BUFFER_SIZE];
		//volatile unsigned char adc_buffer_index;
		#ifdef ADC_BUFFER    
			if (--adc_buffer_index == 255)   adc_buffer_index=(ADC_BUFFER_SIZE-1);
			ADC_V_BUFFER[adc_buffer_index] = VoltageSample[0].PhaseShifted;
		#endif  
	}
#endif

//Current1
#if ((IO == 0) || (IO == 1))//Very strange behavior when using ifdef here with 0 value
	if (IndexAtual == I0){ //First CT
		//A) Store Last Values for future use
		CurrentSample[0].Previous = CurrentSample[0].New;        //Put last sample on its place
		CurrentSample[0].PreviousFiltered = CurrentSample[0].Filtered;        //Put last sample on its place
		// Read in raw voltage and current samples
		CurrentSample[0].New=ADC;      //Read in raw voltage signal from index sample
		// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
		CurrentSample[0].Filtered = 0.996 * ( CurrentSample[0].PreviousFiltered + CurrentSample[0].New - CurrentSample[0].Previous );
		// Root-mean-square method Index Channel
		// -> sum the square of voltage values
		CurrentSample[0].Sum  += CurrentSample[0].Filtered * CurrentSample[0].Filtered; 
		// Instantaneous power calc now that we have I and V
		#if VOLTAGE_SENSORS > 0
			CurrentSample[0].InstPower += VoltageSample[0].PhaseShifted * CurrentSample[0].Filtered; //Instantaneous Power //mudar  pphaseshift
		#else
			CurrentSample[0].InstPower = CurrentSample[0].Filtered;
		#endif
	}
#endif

#ifdef V1
if (IndexAtual == V1){
    //A) Store Last Values for future use
    VoltageSample[1].Previous = VoltageSample[1].New;        //Put last sample on its place
    VoltageSample[1].PreviousFiltered = VoltageSample[1].Filtered;        //Put last sample on its place
    // Read in raw voltage and current samples
    VoltageSample[1].New=ADC;      //Read in raw voltage signal from index sample
    // Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
    VoltageSample[1].Filtered = 0.996 * ( VoltageSample[1].PreviousFiltered + VoltageSample[1].New - VoltageSample[1].Previous );
    // Root-mean-square method Index Channel
    // -> sum the square of voltage values
    VoltageSample[1].Sum  += VoltageSample[1].Filtered * VoltageSample[1].Filtered;  
    // Phase calibration
    VoltageSample[1].PhaseShifted = VoltageSample[1].PreviousFiltered + config.Phase_Calibration[1] * (VoltageSample[1].Filtered - VoltageSample[1].PreviousFiltered);     
  }
#endif 

#ifdef I1
if (IndexAtual == I1){ //First CT
	//A) Store Last Values for future use
	CurrentSample[1].Previous = CurrentSample[1].New;        //Put last sample on its place
	CurrentSample[1].PreviousFiltered = CurrentSample[1].Filtered;        //Put last sample on its place
	// Read in raw voltage and current samples
	CurrentSample[1].New=ADC;      //Read in raw voltage signal from index sample
	// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
	CurrentSample[1].Filtered = 0.996 * ( CurrentSample[1].PreviousFiltered + CurrentSample[1].New - CurrentSample[1].Previous );
	// Root-mean-square method Index Channel
	// -> sum the square of voltage values
	CurrentSample[1].Sum  += CurrentSample[1].Filtered * CurrentSample[1].Filtered; 
	// Instantaneous power calc now that we have I and V
	#if VOLTAGE_SENSORS > 1 
		CurrentSample[1].InstPower += VoltageSample[1].PhaseShifted * CurrentSample[1].Filtered; //Instantaneous Power 
	#else
		#ifdef ADC_BUFFER //if there is adc buffering
			 tmpchar = (adc_buffer_index + PHASE1_OFFSET) % ADC_BUFFER_SIZE;
			 tempdbl = ADC_V_BUFFER[tmpchar];// (adc_buffer_index + PHASE1_OFFSET)% ADC_BUFFER_SIZE);
			//aqui
			CurrentSample[1].InstPower += tempdbl * CurrentSample[1].Filtered;          
		#else //if adc_buffer is inactve
			CurrentSample[1].InstPower = CurrentSample[1].Filtered;
		#endif
	#endif
	}
#endif

#ifdef V2
  if (IndexAtual == V2){
    //A) Store Last Values for future use
    VoltageSample[2].Previous = VoltageSample[2].New;        //Put last sample on its place
    VoltageSample[2].PreviousFiltered = VoltageSample[2].Filtered;        //Put last sample on its place
    // Read in raw voltage and current samples
    VoltageSample[2].New=ADC;      //Read in raw voltage signal from index sample
    // Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
    VoltageSample[2].Filtered = 0.996 * ( VoltageSample[2].PreviousFiltered + VoltageSample[2].New - VoltageSample[2].Previous );
    // Root-mean-square method Index Channel
    // -> sum the square of voltage values
    VoltageSample[2].Sum  += VoltageSample[2].Filtered * VoltageSample[2].Filtered;  
    // Phase calibration
    VoltageSample[2].PhaseShifted = VoltageSample[2].PreviousFiltered + config.Phase_Calibration[2] * (VoltageSample[2].Filtered - VoltageSample[2].PreviousFiltered);     
  }
#endif

  
#ifdef I2
	if (IndexAtual == I2){ //First CT
		//A) Store Last Values for future use
		CurrentSample[2].Previous = CurrentSample[2].New;        //Put last sample on its place
		CurrentSample[2].PreviousFiltered = CurrentSample[2].Filtered;        //Put last sample on its place
		// Read in raw voltage and current samples
		CurrentSample[2].New=ADC;      //Read in raw voltage signal from index sample
		// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
		CurrentSample[2].Filtered = 0.996 * ( CurrentSample[2].PreviousFiltered + CurrentSample[2].New - CurrentSample[2].Previous );
		// Root-mean-square method Index Channel
		// -> sum the square of voltage values
		CurrentSample[2].Sum  += CurrentSample[2].Filtered * CurrentSample[2].Filtered; 
		// Instantaneous power calc now that we have I and V
		#if VOLTAGE_SENSORS > 2
			CurrentSample[2].InstPower += VoltageSample[2].PhaseShifted * CurrentSample[2].Filtered; //Instantaneous Power 
		#else
			#ifdef ADC_BUFFER //if there is adc buffering
				tmpchar = (adc_buffer_index + PHASE2_OFFSET) % ADC_BUFFER_SIZE;
				tempdbl = ADC_V_BUFFER[tmpchar];// (adc_buffer_index + PHASE2_OFFSET)% ADC_BUFFER_SIZE);
				CurrentSample[2].InstPower += tempdbl * CurrentSample[2].Filtered;          
			#else //if adc_buffer is inactve
				CurrentSample[2].InstPower = CurrentSample[2].Filtered;
			#endif
		#endif
	}
#endif

if (AdcIndex == 0) SampleCounter++;
  
  //4807 for 2 channels
  //1602 for 6 channels  
   //1  sec
  if (SampleCounter > (SAMPLES_PER_SECOND * CALC_INTERVAL)){
   LastSampleCounter = SampleCounter;
 
   #if VOLTAGE_SENSORS > 0
     LastVoltage[0].Sum = VoltageSample[0].Sum;
     VoltageSample[0].Sum = 0;
     #if VOLTAGE_SENSORS > 1
       LastVoltage[1].Sum = VoltageSample[1].Sum;
       VoltageSample[1].Sum = 0;
       #if VOLTAGE_SENSORS > 2
         LastVoltage[2].Sum = VoltageSample[2].Sum;
         VoltageSample[2].Sum = 0;
       #endif
     #endif
   #endif
   
   #if PHASE_SENSORS > 0    
      LastCurrent[0].Sum = CurrentSample[0].Sum;
      CurrentSample[0].Sum = 0;
      LastCurrent[0].InstPower = CurrentSample[0].InstPower;
      CurrentSample[0].InstPower = 0;
      #if PHASE_SENSORS > 1    
        LastCurrent[1].Sum = CurrentSample[1].Sum;
        CurrentSample[1].Sum = 0;
        LastCurrent[1].InstPower = CurrentSample[1].InstPower;
        CurrentSample[1].InstPower = 0;
        #if PHASE_SENSORS > 2    
          LastCurrent[2].Sum = CurrentSample[2].Sum;
          CurrentSample[2].Sum = 0;
          LastCurrent[2].InstPower = CurrentSample[2].InstPower;
          CurrentSample[2].InstPower = 0;
         #endif
      #endif
   #endif
    
    SampleCounter = 0;
    Cycle_Full = true;
  }
 
// Maintain multiplexing of input channels
if (++AdcIndex >= (PHASE_SENSORS + VOLTAGE_SENSORS))  AdcIndex=0;
 
//LastInterruptTime = micros() - t1;
}  

void process_rf(){
  
  int node_id = (rf12_hdr & 0x1F);
  byte remoteid = rf12_data[0];
  byte command = rf12_data[1];
  char txbuffer[6] = {0,0,0,0,0,0}; 
  txbuffer[0] = config.nodeID;
  
  switch (command){
  
    case 0x00: // Set all config defaults
       txbuffer[1] = NODE_TYPE;
       txbuffer[2] = VERSION;
       //send the ack with the data
       rf12_sendStart(RF12_ACK_REPLY, &txbuffer, 3);
    break;
    
    case 0x01: // Set all config defaults
      ConfigDefaults();
      saveConfig();
    break;

    case 0x02: // get config value

      switch (rf12_data[1]){
         case 0x01: //radio band
          txbuffer[2] = config.band;
        break;
        case 0x02: // radio group
          txbuffer[2] = config.group;       
        break;
        case 0x03: // radio nodeID
          txbuffer[2] = config.nodeID;   
        break;
        case 0x04: // radio nodeID
          txbuffer[2] = config.sendTo;   
        break;

        case 0x10: // transmit interval
        txbuffer[2] = config.transmitinterval;
        break;
        case 0x11: // Phase Calibration \u2013 Phase 1
         *(float *)(&(txbuffer[2])) = config.Phase_Calibration[0];
        break;      
        case 0x12: // Phase Calibration \u2013 Phase 2
          *(float *)(&(txbuffer[2])) = config.Phase_Calibration[1];
        break;
        case 0x13: // Phase Calibration \u2013 Phase 3
          *(float *)(&(txbuffer[2])) = config.Phase_Calibration[2];
        break;   

        case 0x14: // Current Calibration \u2013 Phase 1
          *(float *)(&(txbuffer[2])) = config.Current_Calibration[0];
        break;  
        case 0x15: // Current Calibration \u2013 Phase 2
          *(float *)(&(txbuffer[2])) = config.Current_Calibration[1];
        break;  
        case 0x16: // Current Calibration \u2013 Phase 3
          *(float *)(&(txbuffer[2])) = config.Current_Calibration[2];
        break;  


        case 0x17: // Voltage Calibration \u2013 Phase 1
          *(float *)(&(txbuffer[2])) = config.Voltage_Calibration[0];        
        break;  
        case 0x18: // Voltage Calibration \u2013 Phase 2
          *(float *)(&(txbuffer[2])) = config.Voltage_Calibration[1]; 
        break;  
        case 0x19: // Voltage Calibration \u2013 Phase 3
          *(float *)(&(txbuffer[2])) = config.Voltage_Calibration[2]; 
        break;  
        
        case 0x1A: // Voltage ( if no voltage sensors are defined)
          *(float *)(&(txbuffer[2])) = config.Voltage; 
        break;  
       }    
   
       txbuffer[1] = 3;
     //send the ack with the data
       rf12_sendStart(RF12_ACK_REPLY, &txbuffer, 6);
   
    break;
    
    case 0x04: // set config value
      switch (rf12_data[2]){
        case 0x01: //radio band
        config.band = (byte) rf12_data[3];
        initRF();
        break;
        case 0x02: // radio group
        config.group = (byte) rf12_data[3];
        initRF();        
        break;
        case 0x03: // radio nodeID
        config.nodeID = (byte) rf12_data[3];
        initRF();      
        break;
        case 0x04: // radio nodeID
        config.sendTo = (byte) rf12_data[3];
        break;

        case 0x10: // transmit interval
        config.transmitinterval = (byte) rf12_data[3];
        break;
        case 0x11: // Phase Calibration \u2013 Phase 1
          
          config.Phase_Calibration[0] = *( (float*)(rf12_data +3) );

        break;       
        case 0x12: // Phase Calibration \u2013 Phase 2
          config.Phase_Calibration[1] = *( (float*)(rf12_data +3) );
        break;
        case 0x13: // Phase Calibration \u2013 Phase 3
          config.Phase_Calibration[2] = *( (float*)(rf12_data +3) );
        break;   

        case 0x14: // Current Calibration \u2013 Phase 1
          config.Current_Calibration[0] = *( (float*)(rf12_data +3) );
        break;  
        case 0x15: // Current Calibration \u2013 Phase 2
          config.Current_Calibration[1] = *( (float*)(rf12_data +3) );
        break;  
        case 0x16: // Current Calibration \u2013 Phase 3
          config.Current_Calibration[2] = *( (float*)(rf12_data +3) );
        break;  


        case 0x17: // Voltage Calibration \u2013 Phase 1
          config.Voltage_Calibration[0] = *( (float*)(rf12_data +3) );
        break;  
        case 0x18: // Voltage Calibration \u2013 Phase 2
          config.Voltage_Calibration[1] = *( (float*)(rf12_data +3) );
        break;  
        case 0x19: // Voltage Calibration \u2013 Phase 3
          config.Voltage_Calibration[2] = *( (float*)(rf12_data +3) );
        break;  
        
        case 0x1A: // Voltage ( if no voltage sensors are defined)
          config.Voltage = *( (float*)(rf12_data +3) );
        break;  
       }
      saveConfig(); 
     //send the ack
     if(RF12_WANTS_ACK){
       rf12_sendStart(RF12_ACK_REPLY, &txbuffer, 1);
     }
    break;
    
  }
 
}


void loop()
{

  if (rf12_recvDone())
      if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)  // and no rf errors
        process_rf(); 
    
    boolean transmit = false;
    float Vrms[3];
    float Irms[3];
    float realPower[3] ;
    float apparentPower[3];
    float powerFactor[3];
    
  //skip first cycle to stabilize readings
  if (tcount == 0 && Cycle_Full == true){
    tcount = 1;
    Cycle_Full = false;
  }
  
  //-------------------------------------------
  if (Cycle_Full == true)
      {Cycle_Full = false;
        //Calculation of the root of the mean of the voltage and current squared (rms)
        //Calibration coeficients applied. 
     #if VOLTAGE_SENSORS > 0
       Vrms[0] = config.Voltage_Calibration[0] * sqrt(LastVoltage[0].Sum / LastSampleCounter); 
       #ifdef ADC_BUFFER
         #if PHASE_SENSORS > 1
          Vrms[1] = Vrms[0]; 
           #if PHASE_SENSORS > 2
            Vrms[2] = Vrms[0]; 
           #endif
         #endif
       #else
         #if VOLTAGE_SENSORS > 1
           Vrms[1] = config.Voltage_Calibration[1] * sqrt(LastVoltage[1].Sum / LastSampleCounter); 
           #if VOLTAGE_SENSORS > 2
             Vrms[2] = config.Voltage_Calibration[2] * sqrt(LastVoltage[2].Sum / LastSampleCounter); 
           #endif
         #endif
       #endif
     #else
       Vrms[0] = config.Voltage;
       Vrms[1] = config.Voltage;
       Vrms[2] = config.Voltage;
     #endif
     
       
     #if PHASE_SENSORS > 0
       //colocar if para 1 ct.
       Irms[0] = config.Current_Calibration[0] * sqrt(LastCurrent[0].Sum / LastSampleCounter); 
         #if VOLTAGE_SENSORS > 0
           realPower[0] = config.Voltage_Calibration[0] * config.Current_Calibration[0] * (LastCurrent[0].InstPower / LastSampleCounter);
         #else
           realPower[0] = Vrms[0] * Irms[0];
         #endif
       apparentPower[0] = Vrms[0] * Irms[0];
       powerFactor[0] = realPower[0] / apparentPower[0];
   //--------------------------
       #if PHASE_SENSORS > 1
         //colocar if para 1 ct.
         Irms[1] = config.Current_Calibration[1] * sqrt(LastCurrent[1].Sum / LastSampleCounter); 
         #if VOLTAGE_SENSORS > 1
           realPower[1] = config.Voltage_Calibration[1] * config.Current_Calibration[1] * (LastCurrent[1].InstPower / LastSampleCounter);
           apparentPower[1] = Vrms[1] * Irms[1];
         #else
           #ifdef ADC_BUFFER
             realPower[1] = config.Voltage_Calibration[0] * config.Current_Calibration[1] * (LastCurrent[1].InstPower / LastSampleCounter);
             apparentPower[1] = Vrms[0] * Irms[1];
           #else
             realPower[1] = Vrms[1] * Irms[1];
             apparentPower[1] = realPower[1];
           #endif
         #endif
         powerFactor[1] = realPower[1] / apparentPower[1];
          //--------------------------
         #if PHASE_SENSORS > 2
           //colocar if para 1 ct.
           Irms[2] = config.Current_Calibration[2] * sqrt(LastCurrent[2].Sum / LastSampleCounter); 
           #if VOLTAGE_SENSORS > 2
             realPower[2] = config.Voltage_Calibration[2] * config.Current_Calibration[2] * (LastCurrent[2].InstPower / LastSampleCounter);
             apparentPower[2] = Vrms[2] * Irms[2];
           #else
             #ifdef ADC_BUFFER
               realPower[2] = config.Voltage_Calibration[0] * config.Current_Calibration[2] * (LastCurrent[2].InstPower / LastSampleCounter);
               apparentPower[2] = Vrms[0] * Irms[2];
             #else
               realPower[2] = Vrms[2] * Irms[2];
               apparentPower[2] = realPower[2];
             #endif
           #endif
           powerFactor[2] = realPower[2] / apparentPower[2]; 
         #endif
       #endif
     #endif
  
  
 
  //calc transmission interval data
  
    #if VOLTAGE_SENSORS > 0
      AccVrms[0] = AccVrms[0] + (Vrms[0] / config.transmitinterval) ;
      #ifdef ADC_BUFFER
      AccVrms[1] = AccVrms[0];
      AccVrms[2] = AccVrms[0];
      #endif
      #if VOLTAGE_SENSORS > 1
        AccVrms[1] = AccVrms[1] + (Vrms[1] / config.transmitinterval) ;
        #if VOLTAGE_SENSORS > 2
          AccVrms[2] = AccVrms[2] + (Vrms[2] / config.transmitintervalL) ;
        #endif
      #endif
    #else
      AccVrms[0] = Voltage;
      AccVrms[1] = Voltage;
      AccVrms[2] = Voltage;
    #endif
    
    #if PHASE_SENSORS > 0
      AccIrms[0] = AccIrms[0] + (Irms[0] / config.transmitinterval) ;
      AccRealPower[0] = AccRealPower[0] + (realPower[0] / config.transmitinterval) ;
      AccAparentPower[0] = AccAparentPower[0] + (apparentPower[0] / config.transmitinterval) ;
      AccPowerFactor[0] = AccPowerFactor[0] + (powerFactor[0] / config.transmitinterval) ;
      #if PHASE_SENSORS > 1
        AccIrms[1] = AccIrms[1] + (Irms[1] / config.transmitinterval) ;
        AccRealPower[1] = AccRealPower[1] + (realPower[1] / config.transmitinterval) ;
        AccAparentPower[1] = AccAparentPower[1] + (apparentPower[1] / config.transmitinterval) ;
        AccPowerFactor[1] = AccPowerFactor[1] + (powerFactor[1] / config.transmitinterval) ;
        #if PHASE_SENSORS > 2
          AccIrms[2] = AccIrms[2] + (Irms[2] / config.transmitinterval) ;
          AccRealPower[2] = AccRealPower[2] + (realPower[2] / config.transmitinterval) ;
          AccAparentPower[2] = AccAparentPower[2] + (apparentPower[2] / config.transmitinterval) ;
          AccPowerFactor[2] = AccPowerFactor[2] + (powerFactor[2] / config.transmitinterval) ;
        #endif
      #endif
    #endif
    tcount++;


    //transmit
    if (tcount > config.transmitinterval) {
      tcount = 1;
      counts++;
       emontx.command = 0x20;
       emontx.nodeId = config.nodeID;
       emontx.txCount = counts;
       emontx.totalP = AccRealPower[0] 
       #if PHASE_SENSORS > 1  
         + AccRealPower[1]
         #if PHASE_SENSORS > 2  
           + AccRealPower[2]
         #endif
       #endif
       ;
       
       //calculate kw increment
       float whInc = emontx.totalP *((CALC_INTERVAL * config.transmitinterval)/3600.0);
       emontx.cummKw += whInc / 1000;
       
       emontx.V1 = AccVrms[0];
       #if (PHASE_SENSORS > 0)
         emontx.Irms1 = AccIrms[0];
         emontx.RP1 = AccRealPower[0];
         emontx.PF1 = AccPowerFactor[0];   
       #endif
       
       #if (VOLTAGE_SENSORS > 1)
         emontx.V1 = AccVrms[1];
       #endif
       #if (PHASE_SENSORS > 1)
         emontx.Irms2 = AccIrms[1];
         emontx.RP2 = AccRealPower[1];
         emontx.PF2 = AccPowerFactor[1];   
       #endif
       #if (VOLTAGE_SENSORS > 2)
         emontx.V1 = AccVrms[2];
       #endif
       #if (PHASE_SENSORS > 2)
         emontx.Irms3 = AccIrms[2];
         emontx.RP3 = AccRealPower[2];
         emontx.PF3 = AccPowerFactor[2];   
       #endif
       
       //send data
       
       int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
       byte outdest = config.sendTo & RF12_HDR_MASK ? RF12_HDR_DST | config.sendTo : 0;
       rf12_sendStart(outdest, &emontx, sizeof emontx);
       //Serial.println("package sent");

       

      //erase data
     #if VOLTAGE_SENSORS > 0
      AccVrms[0] = 0 ;
      #if VOLTAGE_SENSORS > 1
        AccVrms[1] = 0 ;
        #if VOLTAGE_SENSORS > 2
          AccVrms[2] = 0 ;
        #endif
      #endif
    #else
      AccVrms[0] = 0;
      AccVrms[1] = 0;
      AccVrms[2] = 0;
    #endif
    
    #if PHASE_SENSORS > 0
      AccIrms[0] = 0 ;
      AccRealPower[0] = 0 ;
      AccAparentPower[0] = 0 ;
      AccPowerFactor[0] = 0 ;
      #if PHASE_SENSORS > 1
        AccIrms[1] = 0 ;
        AccRealPower[1] = 0 ;
        AccAparentPower[1] = 0 ;
        AccPowerFactor[1] = 0 ;
        #if PHASE_SENSORS > 2
          AccIrms[2] = 0 ;
          AccRealPower[2] = 0 ;
          AccAparentPower[2] = 0 ;
          AccPowerFactor[2] = 0 ;
        #endif
      #endif
    #endif
    }//transmit
  }//cycle full
}








