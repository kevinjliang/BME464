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
long lastACUpdate = 0; //the last time AC characteristics were reset

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
boolean prevZCross = false;
boolean ZCrossInWindow = false;
float hystThresh = 0.1; //Hysteresis threshold; must cross before another zero-crossing can be detected
float lastZCrossTime = 0; //to track times between zero-crossings 
float maxVoltage = 0; //Maximum voltage (ie amplitude) detected in an AC signal
struct ACdata currentHalfWave = {0, 0}; //characteristics of the half-wave detected between zero-crossings  

//LCD Display Variables
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
boolean isAC = false;

//Define Pines
int outputPin = 6; 
int logicPin = 7;

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
  
  //Set pin 6 (PWM) to output
  pinMode(outputPin, OUTPUT);
  
  //Set pin 7 to logic for MUX
  pinMode(logicPin, OUTPUT);
}


void loop(){
  currentTime = millis();
  
  //read the signal every 10 ms
  if(currentTime > lastReadTime + 10){
    lastReadTime = currentTime; 
    V_in = read_signal(); 
    if(zeroCrossing(trueVal)) {
      if(prevZCross) {
        currentHalfWave = characterizeAC();  
        isAC = true;
        ZCrossInWindow = true;
      }
      prevZCross = true;  
    }
  }  
  
  //Output via PWM pin
  if(trueVal<0){ //if voltage is less than 0, then MUX will see LOW
  digitalWrite(logicPin, LOW);
  }
  else{ // if voltage is greater than 0, then MUX will see HIGH
  digitalWrite(logicPin, HIGH);
  }
  output(abs(trueVal)); // TO DO: May need to change frequency of output
  
  //update display every second
  if(currentTime > lastDisplayTime + 1000) {
    lastDisplayTime = currentTime;
    displayToLCD();
    prevZCross = false;
  }
  
  if(currentTime > lastACUpdate + 5000) {
    lastACUpdate = currentTime;
    if(!ZCrossInWindow) {
      isAC = false;
    }
    ZCrossInWindow = false;
  }
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

//Output digital to analog using PWM
void output(float outputVoltage){
  if (outputVoltage > 5){
    outputVoltage = 5;
  }
  else if (outputVoltage < 0){
    outputVoltage = 0;
  }
  else{
  outputVoltage = (outputVoltage/5)*255;
  analogWrite(outputPin, outputVoltage);
  }
}


//Display Information on LCD
void displayToLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  if(!isAC){ //DC case
    lcd.print("V: "); lcd.print(convertToBinary(V_in)); lcd.print("V");
    lcd.setCursor(0, 1);    
    lcd.print("DC");
  }
  else{ //AC case
    lcd.print("Amplitude: "); lcd.print(currentHalfWave.amplitude); lcd.print("V");
    lcd.setCursor(0, 1);    
    lcd.print("AC freq: "); lcd.print(currentHalfWave.frequency); lcd.print("Hz");
  }
}  

String convertToBinary(float decimal){
  String binary= "";
  
  //Add sign of voltage
  if(decimal<0){
    binary = binary + "-";
  }
  else{
    binary = binary + "+";
  }
  // TO DO: Convert this sign to a binary sign instead of the actual character
  
  decimal = abs(decimal);
  
  //Add ones place in BINARY
 int ones = floor(decimal);
 binary = binary + String(ones,BIN) + ".";
 
 //Add decimal places
 float fraction = decimal - ones;
 
 //Alternative Algorithm
 if(fraction - .5 >=0){
   binary = binary + "1";
   fraction = fraction -.5;
 }
 else{
   binary = binary +"0";
 }
 
 if(fraction - .25 >=0){
   binary = binary + "1";
   fraction = fraction -.25;
 }
 else{
   binary = binary + "0";
 }
 
 if(fraction - .125 >=0){
   binary = binary + "1";
   fraction = fraction -.125;
 }
 else{
   binary = binary + "0";
 }

if(fraction - .0625 >=0){
 binary = binary + "1";
 fraction = fraction -.0625; 
}
else{
  binary = binary + "0";
}

if(fraction - .03125 >=0){
  binary = binary + "1";
  fraction = fraction -.03125;
}
else{
  binary = binary + "0";
}

if(fraction - .015625 >=0){
  binary = binary + "1";
  fraction = fraction - .015625;
}
else{
  binary = binary + "0'";
} 

if(fraction - .0078125 >=0){
  binary = binary + "1";
  fraction = fraction - .0078125;
}
else{
  binary = binary +"0";
}

 /*
 // Brute Force Method
 if (fraction <= .03125){
   binary = binary + "0000"
 }
 else if((fraction - .0625)<=.0625){
   binary = binary + "0001";
 } 
 else if((fraction - .125)<=.0625){
   binary = binary + "0010";
 } 
 else if((fraction - .1825)<=.0625){
   binary = binary + "0011";
 } 
  else if((fraction - .25)<=.0625){
   binary = binary + "0100";
 } 
  else if((fraction - .3125)<=.0625){
   binary = binary + "0101";
 } 
  else if((fraction - .375)<=.0625){
   binary = binary + "0110";
 } 
  else if((fraction - .4375)<=.0625){
   binary = binary + "0111";
 } 
  else if((fraction - .5)<=.0625){
   binary = binary + "1000";
 } 
  else if((fraction - .5625)<=.0625){
   binary = binary + "1001";
 } 
  else if((fraction - .625)<=.0625){
   binary = binary + "1010";
 } 
  else if((fraction - .6875)<=.0625){
   binary = binary + "1011";
 } 
  else if((fraction - .75)<=.0625){
   binary = binary + "1100";
 } 
  else if((fraction - .8125)<=.0625){
   binary = binary + "1101";
 } 
  else if((fraction - .875)<=.0625){
   binary = binary + "1110";
 } 
 else{
   binary = binary + "1111";
 }
 */
  return binary;
}


boolean zeroCrossing(float voltage) {
  if(abs(voltage) > maxVoltage){
    maxVoltage = abs(voltage);
  }
  
  //Check for zero-crossing in the proper direction
  if(wasPositive){
    if(voltage<-hystThresh) {
      wasPositive = false;
      return true;
    }
  }
  else {
    if(voltage>hystThresh) { 
      wasPositive = true;
      return true;
    }
  }  
  return false;
}

struct ACdata characterizeAC(){
  //build halfwave object
  ACdata halfWave;
  halfWave.frequency = 1 / (2 * (currentTime - lastZCrossTime)/1000.0);
  halfWave.amplitude = maxVoltage;
  
  //reset variables
  lastZCrossTime = currentTime;
  maxVoltage = 0;
  
  return halfWave;
}


