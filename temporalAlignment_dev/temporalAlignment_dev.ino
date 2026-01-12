// A proof-of-concept build to see if the temporal alignment between V and I 
// can be used to allow current to be measured correctly in the DC sense. 
// 
//            Robin Emley (calypso_rae on Open Energy Monitor Forum)
//                  December 2012

#define TALLYMODE 

// general definitions
#define POSITIVE 1
#define NEGATIVE 0
#define CYCLES_PER_SECOND 50 
#define HALF_CYCLES_PER_SECOND 100

#define CAPACITY_OF_ENERGY_BUCKET 3600 // 0.001 kWh = 3600 Joules
#define SAFETY_MARGIN_IN_WATTS 0  // <<<------ increase for more export
#define START_UP_PERIOD 5 // seconds, to allow HP filters to settle

const byte voltageSensorPin = 2;  // analogue
const byte currentSensorPin = 1;   // analogue
long cycleCount = 0;
int samplesDuringHalfCycle = 0;
float energyInBucket; // mimics the operation of a meter at the grid connection point.                                                
boolean beyondStartUpPhase = false;
float safetyMargin_WattsPerHalfCycle;
float sumP;

// values that need to be stored from one loop to the next
int lastSampleV;      // for HPF on voltage stream       
int lastSampleI;      // for HPF on current stream    
byte polarityOfLastSampleV; // for zero-crossing detection

float phaseCal;  // To phase-shift the voltage waveform w.r.t. the one for current
float powerCal;  // To convert the product of raw V & I samples into Joules.  
float voltageCal; // To convert raw voltage samples into volts.  Used for determining when
                  // the trigger device can be safely armed

float DCoffset_V = 512;       // <<--- for LPF 
float cumVdeltasThisCycle;   // <<--- for LPF 
float lastSampleVminusDC;     // <<--- for phaseCal calculation 
float DCoffset_I = 512;

// items for LED monitoring
float energyInBucket_4led = 0; // also useful for calibration because it has no limits
float energyLevelAtLastLedPulse;                 

#ifdef TALLYMODE
#define NUMBER_OF_TALLIES 100
unsigned int tally_pos[NUMBER_OF_TALLIES + 2]; // For recording the energy distribution in +ve and -ve half-cycles.
unsigned int tally_neg[NUMBER_OF_TALLIES + 2]; // The 2 extra elements are for under-range and over-range
unsigned int tallymode_maxCycleCount;  // the cycleCount value when recording should cease
int tallymode_maxVal; // the maximum power to be recorded (Watts)
int tallymode_minVal; // the minimum power to be recorded (Watts)
float tallymode_stepVal; // the power increment between consecutive tallies (Watts)
boolean tallymode_firstLoop = true; 
unsigned int noOfValuesTallied; // overflows after 10 minutes
unsigned long noOfSamplePairs;
int tallymode_durationOfRecording; // seconds
#endif

void setup()
{  
  Serial.begin(9600);
  Serial.println();
  Serial.println("starting new run");
  Serial.println();
    
  powerCal = 0.06; // Units are Joules per ADC-level squared.  Used for converting the product of 
                    // voltage and current samples into Joules.
                    //    To determine this value, note the rate that the energy bucket's
                    // level increases when a known load is being measured at a convenient
                    // test location (e.g  using a mains extention with the outer cover removed so that 
                    // the current-clamp can fit around just one core.  Adjust POWERCAL so that
                    // 'measured value' = 'expected value' for various loads.  The value of
                    // POWERCAL is not critical as any absolute error will cancel out when 
                    // import and export flows are balanced.  
                    
  voltageCal = (float)679 / 471; // Units are Volts per ADC-level.
                        // This value is used to determine when the voltage level is suitable for 
                        // arming the external trigger device.  To set this value, note the min and max
                        // numbers that are seen when measuring 240Vac via the voltage sensor, which 
                        // is 678.8V p-t-p.  The range on my setup is 471 meaning that I'm under-reading 
                        // voltage by 471/679.  VOLTAGECAL therefore need to be the inverse of this, i.e.
                        // 679/471 or 1.44
                        
  phaseCal = 1.0;  // the default or 'do nothing' value
  energyInBucket = 1500; // for quicker startup in normal conditions
  safetyMargin_WattsPerHalfCycle = (float)SAFETY_MARGIN_IN_WATTS / HALF_CYCLES_PER_SECOND;       
}


void loop() // each loop is for one pair of V & I measurements
{

#ifdef TALLYMODE
if (tallymode_firstLoop) { 
  tallymode_setup(); // user-dialogue for recording energy data
  energyInBucket = 1799; // for instant startup when recording data
  noOfValuesTallied = 0;
  noOfSamplePairs = 1;
  beyondStartUpPhase = false;
  tallymode_firstLoop = false; }
else
if (cycleCount > tallymode_maxCycleCount) {
  tallymode_dispatchData(); // send recorded energy data to the Serial monitor  
  cycleCount = 0;
  tallymode_firstLoop = true;
  pause(); } // so that user can access data from the Serial monitor
else
if (beyondStartUpPhase) {
  noOfSamplePairs++; }
#endif

// Get the next pair of raw samples.  Because the CT generally adds more phase-advance 
// than the voltage sensor, it makes sense to sample current before voltage
  int sampleI = analogRead(currentSensorPin);   
  int sampleV = analogRead(voltageSensorPin);  
  
  // remove the DC offsets from raw samples using their respective calculated values
  float sampleVminusDC = sampleV - DCoffset_V; 
  float sampleIminusDC = sampleI - DCoffset_I; 

  // Check for the start of a new mains cycle 
  byte polarityNow;
  if(sampleVminusDC > 0) {
    polarityNow = POSITIVE; }
  else { 
    polarityNow = NEGATIVE; }
  
  if (polarityNow != polarityOfLastSampleV)
  {    
    // This is the start of a new half cycle (just after the a zero-crossing point)
    if (polarityNow == POSITIVE) 
    {
      cycleCount++; 
      // update the Low Pass Filter for DC-offset removal
      DCoffset_V += 0.01 * cumVdeltasThisCycle; 
      cumVdeltasThisCycle = 0;
    }    
    
    // The DC-offset for the current sensor is updated every half-cycle,
    // immediately after every zero-crossing point:
    //
    // first, locate the moment when the voltage was zero
    float T_fraction = lastSampleVminusDC / (float)(lastSampleVminusDC - sampleVminusDC);
    
    //  then, calculate the raw current value at the moment when the voltage was zero
    float I_zero = lastSampleI + (T_fraction * (sampleI - lastSampleI));
    
    // and finally, update the DC offset value for current
    float oldOffset_I = DCoffset_I;
    DCoffset_I = oldOffset_I + (0.01 * (I_zero - oldOffset_I));    

    //  Calculate the real power during the last half cycle
    float realPower = powerCal * (sumP / samplesDuringHalfCycle);
    float realEnergy = realPower / HALF_CYCLES_PER_SECOND;

     if (beyondStartUpPhase == true)
    {  
      // Providing that the high-pass filters have had sufficient time to settle,    
      // add this power contribution to the energy bucket
      energyInBucket += realEnergy;   
      energyInBucket_4led += realEnergy;   
         
#ifdef TALLYMODE
      tallymode_updateData(realPower, polarityOfLastSampleV); // update the relevant tally 
#endif
      // Reduce the level in the energy bucket by any required safety margin.
      energyInBucket -= safetyMargin_WattsPerHalfCycle;       

      // Apply max and min limits to bucket's level
      if (energyInBucket > CAPACITY_OF_ENERGY_BUCKET) {
        energyInBucket = CAPACITY_OF_ENERGY_BUCKET; } 
      else  
      if (energyInBucket < 0) {
        energyInBucket = 0; }    
    }
    else
    {  
      // check whether the high-pass filters have had time to settle
      if(cycleCount > (START_UP_PERIOD * CYCLES_PER_SECOND))
      {
        beyondStartUpPhase = true;
        Serial.println ("go!"); 
      }
    }   
      
    // clear the per-cycle accumulators for use in this new half cycle that has just started.  
    samplesDuringHalfCycle = 0;
    sumP = 0;
  } // end of processing that is specific to the first Vsample in each new half cycle
    

  
  // Processing for ALL sample pairs
  //--------------------------------
  // 
  // phase-shift the voltage waveform to align with the current
  float  phaseShiftedVminusDC = 
                lastSampleVminusDC + phaseCal * (sampleVminusDC - lastSampleVminusDC);  
//  float instP = phaseShiftedVminusDC * sampleIminusDC; //  power contribution for this pair of samples  
  float instP = sampleVminusDC * sampleIminusDC; //  power contribution for this pair of samples  
  sumP +=instP; // cumulative power contributions
  samplesDuringHalfCycle++;
  cumVdeltasThisCycle += (sampleV - DCoffset_V); // for use with LP filter
    
  // store items that need to be carried over to the next loop
  lastSampleV = sampleV;
  lastSampleI = sampleI;
  lastSampleVminusDC = sampleVminusDC;
//  lastFilteredI = filteredI;  
  polarityOfLastSampleV = polarityNow;
} // end of loop()



#ifdef TALLYMODE
void  tallymode_setup()
{
  char inbuf[10];
  int tempInt;
  byte noOfBytes;
  boolean done;

  Serial.println ("WELCOME TO TALLYMODE ");
  Serial.println ("This mode of operation allows the energy content of individual mains cycles");
  Serial.println ("to be analysed.  For nomal operation, the #define TALLYMODE statement");
  Serial.println ("should be commented out. ");
    
  Serial.print ("Time to run (seconds)? ");
  done = false;
  while (!done) {
    noOfBytes = Serial.available();
    if (noOfBytes > 0) { done = true; } else { delay(100); }}
  for (tempInt = 0; tempInt < 10; tempInt++) { inbuf[tempInt] = 0; }
  Serial.readBytes(inbuf, noOfBytes); 
  tempInt = atoi(inbuf);  Serial.println (tempInt);
  tallymode_maxCycleCount = (tempInt + START_UP_PERIOD) * CYCLES_PER_SECOND;
  tallymode_durationOfRecording = tempInt;
  Serial.print(" tallymode_maxCycleCount = "); Serial.print(tallymode_maxCycleCount);     
  Serial.print(", recording to start at cycleCount ");
  Serial.println(START_UP_PERIOD * CYCLES_PER_SECOND);  
  
  Serial.print ("Min value to be recorded (Watts)? ");
  done = false;
  while (!done) {
    noOfBytes = Serial.available();
    if (noOfBytes > 0) { done = true; } else { delay(100); }}
  for (tempInt = 0; tempInt < 10; tempInt++) { inbuf[tempInt] = 0; }
  Serial.readBytes(inbuf, noOfBytes); 
  tempInt = atoi(inbuf);  Serial.println (tempInt);
  tallymode_minVal = tempInt;
  Serial.print(" tallymode_minVal = "); Serial.println(tallymode_minVal);     
  
  Serial.print ("Max value to be recorded (Watts)? ");
  done = false;
  while (!done) {
    noOfBytes = Serial.available();
    if (noOfBytes > 0) { done = true; } else { delay(100); }}
  for (tempInt = 0; tempInt < 10; tempInt++) { inbuf[tempInt] = 0; }
  Serial.readBytes(inbuf, noOfBytes); 
  tempInt = atoi(inbuf);  Serial.println (tempInt);
  tallymode_maxVal = tempInt;
  Serial.print(" tallymode_maxVal = "); Serial.println(tallymode_maxVal);     
  
  tallymode_stepVal = (float)(tallymode_maxVal - tallymode_minVal) / NUMBER_OF_TALLIES;
  Serial.print(" tallymode_stepVal = "); Serial.println(tallymode_stepVal);   

  for (tempInt = 0; tempInt < NUMBER_OF_TALLIES + 2; tempInt++) {
    tally_pos[tempInt] = 0; 
    tally_neg[tempInt] = 0; }

  Serial.print(" Data recording will start in ");
  Serial.print(START_UP_PERIOD);   
  Serial.println(" seconds ... ");
};

void  tallymode_updateData(float power, byte polarity_now)
{
  int index = (power - tallymode_minVal) / tallymode_stepVal;
  if (index < 0) {
    index = 0; } // tally[0] is for underflow
  else 
  if (index > NUMBER_OF_TALLIES) {
    index = NUMBER_OF_TALLIES + 1; } // tally[N+1] is for overflow

  if (polarity_now == POSITIVE) {
    tally_neg[index]++; } // data from a -ve HC is avalable during the next +ve one
  else {   
    tally_pos[index]++; } // data from a +ve HC is avalable during the next -ve one
  noOfValuesTallied++; 
};

void  tallymode_dispatchData()
{
  Serial.println ();
  Serial.println ("Dispatching tally data ... ");
  Serial.print ("Sorted data runs from tally[1] to tally[");
  Serial.print (NUMBER_OF_TALLIES);
  Serial.println ("].");
  Serial.print ("tally[0] is below range; tally[ ");
  Serial.print (NUMBER_OF_TALLIES + 1);
  Serial.println ("] is above range.");
  Serial.println();
  Serial.print (tallymode_minVal);
  Serial.println (", <- min value of tally range (W)");
  Serial.print (tallymode_maxVal);
  Serial.println (", <- max value of tally range (W)");
  Serial.print (tallymode_stepVal);
  Serial.println (", <- step value between tallies (W)");
  Serial.print (NUMBER_OF_TALLIES);
  Serial.println (", <- number of tallies");
  Serial.print (tallymode_durationOfRecording);
  Serial.println (", <- duration of recording (sec)");
  Serial.print (noOfValuesTallied);
  Serial.println (", <- no of values tallied");
  Serial.print (noOfSamplePairs / 
         (tallymode_durationOfRecording * CYCLES_PER_SECOND));
  Serial.println (", <- samples per mains cycle (average)");
  
//  for (int index = 0; index < NUMBER_OF_TALLIES + 2; index++)
  for (int index = NUMBER_OF_TALLIES + 1; index >= 0; index--)
  {
    Serial.print (index);
    Serial.print (", ");
    Serial.print (tally_pos[index]);
    Serial.print (", ");
    Serial.println (tally_neg[index]);
  }
};
#endif

void pause()
{
  byte done = false;
  byte dummyByte;
   
  while (done != true)
  {
    if (Serial.available() > 0) {
      dummyByte = Serial.read(); // to 'consume' the incoming byte
      if (dummyByte == 'g') done++; }
  }    
}


