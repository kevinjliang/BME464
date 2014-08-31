#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup(){
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
  
  //Timekeeping variables
  long currentTime = 0; //tracks how long it's been since the program began running
  long lastReadTime = 0; //the last time a reading was taken from the circuit

  //Input voltage variables
  float readVal; //voltage reading from pin A0, from circuit
  float trueVal; //readjusted true value
  
  //Sliding Window Averager variables
  float slidingWindow[10] = {0,0,0,0,0,0,0,0,0,0}; //queue for the ten most recent readings
  int nextElement = 0; //rotating index of the array to place the next read val
  float avg = 0; //sum of all elements in the queue
  int avgDenom = 0; //denominator to calculate average
  
  //Zero-crossings detector variables
  boolean wasPositive = false;
  float hystThresh = 0.1;
  float lastZCrossTime = 0;
  

}


void loop(){
  
}
