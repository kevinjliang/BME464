#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

struct ACdata {
  float frequency; //equal to 1/(2*period between zero-crossings) 
  float amplitude;  
};

//// Declare all variables here as global to avoid re-declaring the same variables in the loop() and running out of memory 
//Timekeeping variables
long currentTime = 0; //tracks how long it's been since the program began running
long lastReadTime = 0; //the last time a reading was taken from the circuit
long lastDisplayTime = 0; //the last time the LCD screen with updated

//Input voltage variables
float readVal; //voltage reading from pin A0, from circuit
float trueVal; //readjusted true value
float V_in; //"final" input value; readjusted + smoothed
  
//Sliding Window Averager variables
float slidingWindow[5] = {0,0,0,0,0}; //queue for the five most recent readings
int nextElement = 0; //rotating index of the array to place the next read val
float avg = 0; //sum of all elements in the queue
int avgDenom = 0; //denominator to calculate average
  
//AC variables
boolean wasPositive = true; //was previous value positive; compare to look for a zero-crossing
boolean ZCrossDetected = false; 
float hystThresh = 0.1; //Hysteresis threshold; must cross before another zero-crossing can be detected
float lastZCrossTime = 0; //to track times between zero-crossings 
float maxVoltage = 0; //Maximum voltage (ie amplitude) detected in an AC signal
ACdata currentHalfWave = {0, 0}; //characteristics of the half-wave detected between zero-crossings  
ACdata benchmark = {0, 0}; //characteristics of the half-wave being compared against
int numConsistencies = 0; //Number of times half-wave characteristics match up;
float ACtolerannce = 0.1; //Tolerance as to how different amplitude and frequency can be to be considered "consistent"

//LCD Display Variables
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
boolean isAC = false;

void setup(){
  // Serial.begin(9600); // For printing to console
  
  // LCD initialization
  lcd.begin(16, 2);
  lcd.setBacklight(0x7);
  
  // Start up message
  lcd.setCursor(0, 0);
  lcd.print("Hey-");
  lcd.setCursor(0, 1);
  lcd.print("O");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.clear();
}


void loop(){
  currentTime = millis();
  
  //read the signal every 20 ms
  if(currentTime > lastReadTime + 20){
    lastReadTime = currentTime; 
    V_in = read_signal(); 
    isAC = zeroCrossingChecker(V_in);
  }  
  
  //update display every second
  if(currentTime > lastDisplayTime + 1000) {
    lastDisplayTime = currentTime;
    displayToLCD();
  }
  
  // TODO: DAC part of the code => output signal to analog pin
}


//Reads the input, corrects it back to the original voltage, then applies a sliding window averager
//Returns averaged (smoothed) voltage value
float read_signal(){
  readVal = analogRead(1)* (5.0 / 1023.0); //analogRead maps 0-5 V to 0-1023
  //Undo the shift and gain circuit
  trueVal = readVal*2 - 5;
    
  slidingWindow[nextElement] = trueVal;
  avg = 0;
  for(int i=0; i<5; i++){
     avg = avg + slidingWindow[i];
  }
    
  //rotate through the five elements of the queue
  if(nextElement<4){
    nextElement++;
  }    
  else{
    nextElement = 0;  
  }  
    
  //adjust denominator of average to the number of elements in queue
  if(avgDenom<5){
    avgDenom++;
  }  
     
  return avg/avgDenom;
}  

void displayToLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  if(!isAC){ //DC case
    lcd.print("Voltage: "); lcd.print(V_in); lcd.print("V");
    lcd.setCursor(0, 1);    
    lcd.print("DC");
  }
  else{ //AC case; TODO: insert whatever we decide to name the amplitude and frequency variables
    lcd.print("Amplitude: "); lcd.print(""); lcd.print("V");
    lcd.setCursor(0, 1);    
    lcd.print("AC freq: "); lcd.print(""); lcd.print("Hz");
  }
}  

// Checks for AC characteristics (amplitude + frequency)
// Returns true if a zero-crossing is detected, at which point a new half-wave has been measured
boolean zeroCrossingChecker(float voltage) {
  if(abs(voltage) > maxVoltage){
    maxVoltage = abs(voltage);
  }
  
  //Check for zero-crossing in the proper direction
  if(wasPositive){
    if(voltage<-hystThresh) {
      currentHalfWave.frequency = 1/(2*(currentTime-lastZCrossTime));
      currentHalfWave.amplitude = maxVoltage;
      maxVoltage = 0;  
      wasPositive = false;
      lastZCrossTime = currentTime;
      return true;
    }
  }
  else {
    if(voltage>hystThresh) {
      currentHalfWave.frequency = 1/(2*(currentTime-lastZCrossTime));  
      currentHalfWave.amplitude = maxVoltage;
      maxVoltage = 0;  
      wasPositive = true;
      lastZCrossTime = currentTime;
      return true;
    }
  }  
  
  return false;
}

//Characterize the AC signal; compare it with previous amplitudes and frequencies to verify
//Returns true if passes checks of consistency
boolean isACsignal(){
   
}
