#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();


//// Declare all variables here as global to avoid re-declaring the same variables in the loop() and running out of memory 
//Timekeeping variables
long currentTime = 0; //tracks how long it's been since the program began running
long lastReadTime = 0; //the last time a reading was taken from the circuit

//Input voltage variables
float readVal; //voltage reading from pin A0, from circuit
float trueVal; //readjusted true value
  
//Sliding Window Averager variables
float slidingWindow[5] = {0,0,0,0,0}; //queue for the five most recent readings
int nextElement = 0; //rotating index of the array to place the next read val
float avg = 0; //sum of all elements in the queue
int avgDenom = 0; //denominator to calculate average
  
//Zero-crossings detector variables
boolean wasPositive = true; //was previous value positive; compare to look for a zero-crossing
float hystThresh = 0.1; //Hysteresis threshold; must cross before another zero-crossing can be detected
float lastZCrossTime = 0; //to track times between zero-crossings
float frequency = 0; //equal to 1/(2*period between zero-crossings)  


void setup(){
  Serial.begin(9600);
  
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
  
  //read the signal every 200 ms
  if(currentTime > lastReadTime + 200){
    lastReadTime = currentTime; 
    displayToLCD(read_signal());
  }  
}



float read_signal(){
  readVal = analogRead(1)* (5.0 / 1023.0); //analogRead maps 0-5 V to 0-1023
  Serial.println(readVal);
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

void displayToLCD(float outVolt) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage: "); lcd.print(outVolt);
}  

int checkForZeroCrossing() {
  if(wasPositive){
    if(trueVal<-hystThresh) {
      frequency = 1/(2*(currentTime-lastZCrossTime));  
      wasPositive = false;
      lastZCrossTime = currentTime;
    }
  }
  else {
    if(trueVal>hystThresh) {
      frequency = 1/(2*(currentTime-lastZCrossTime));  
      wasPositive = true;
      lastZCrossTime = currentTime;
    }
  }  
  return frequency;
}
