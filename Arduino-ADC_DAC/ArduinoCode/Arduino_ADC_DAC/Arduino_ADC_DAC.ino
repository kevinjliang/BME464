#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
// avr-libc library includes (for interrupts)
#include <avr/io.h>
#include <avr/interrupt.h>

struct ACdata {
  float frequency; //equal to 1/(2*period between zero-crossings) 
  float amplitude;  
};

//// Declare all variables here as global to avoid re-declaring the same variables in the loop() and running out of memory 
//Timekeeping variables
long currentTime = 0; //tracks how long it's been since the program began running
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
int logicPin = 2;

void setup(){
  Serial.begin(9600); // For printing to console

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

  //Set pin 2 to logic for MUX to control sign of output
  pinMode(logicPin, OUTPUT);
  
  // initialize Timer1 for interrupts
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to 10 ms timer count:
  OCR1A = 156;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

//Interrupt code, runs every 10 ms
ISR(TIMER1_COMPA_vect)
{
  V_in = read_signal(); 
  
  //Check for zero-crossing (ie, is this AC?)
  if(zeroCrossing(trueVal)) {
    if(prevZCross) {
      currentHalfWave = characterizeAC();  
      isAC = true;
      ZCrossInWindow = true;
    }
    prevZCross = true;  
  }

  //Output via PWM pin
  output(abs(trueVal)); // TO DO: May need to change frequency of output
}


void loop(){
  currentTime = millis();

  //update display every second
  if(currentTime > lastDisplayTime + 1000) {
    lastDisplayTime = currentTime;
    displayToLCD();
  }

  if(currentTime > lastACUpdate + 5000) {
    lastACUpdate = currentTime;
    if(!ZCrossInWindow) {
      isAC = false;
      prevZCross = false;
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
  if (trueVal < 0) { //if voltage is less than 0, then MUX will see HIGH
    digitalWrite(logicPin, HIGH);
  }
  else { // if voltage is greater than 0, then MUX will see LOW
    digitalWrite(logicPin, LOW);
  }

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
    lcd.print("V:"); 
    lcd.print(convertToBinary(V_in)); 
    lcd.print("V");
    lcd.setCursor(0, 1);    
    lcd.print("DC");
  }
  else{ //AC case
    lcd.print("Amplitude: "); 
    lcd.print(currentHalfWave.amplitude); 
    lcd.print("V");
    lcd.setCursor(0, 1);    
    lcd.print("AC freq: "); 
    lcd.print(currentHalfWave.frequency); 
    lcd.print("Hz");
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

  decimal = abs(decimal);

  int ones = floor(decimal);
  int hold = ones;

  if(ones - 4 >= 0){
    binary = binary + "1";
    ones = ones - 4;
  }
  else{
    binary = binary + "0";
  }
  
  if(ones - 2 >= 0){
    binary = binary + "1";
    ones = ones - 2;
  }
  else{
    binary = binary + "0";
  }
  
  if(ones - 1 >= 0){
    binary = binary + "1.";
    ones = ones -1;
  }
  else{
    binary = binary + "0.";
  }

  //Add decimal places
  float fraction = decimal - hold;

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
    binary = binary + "0";
  } 

  if(fraction - .0078125 >=0){
    binary = binary + "1";
    fraction = fraction - .0078125;
  }
  else{
    binary = binary +"0";
  }

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



