// Blynk setup
// PASTE CODE HERE FROM STEP 9
#define BLYNK_TEMPLATE_ID "TMPL2z73z5VrR"
#define BLYNK_TEMPLATE_NAME "Wiifit Wifi Test"
#define BLYNK_AUTH_TOKEN "1Sm7H4tBMp8ETi_d4ViLSGwZWtfaw8j6"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Arduino"; //WIFI NAME
char pass[] = "fitnesstracker"; //WIFI PWD


// Accelerometer setup
#include <DFRobot_LIS2DH12.h>
/*!
 * @brief Constructor 
 * @param pWire I2c controller
 * @param addr  I2C address(0x18/0x19)
 */
DFRobot_LIS2DH12 acce(&Wire,0x18);

// Barometer Setup
#include <DFRobot_BMP3XX.h>
DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// Setup OLED
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;

// Define gloabal variables

// Step count variables
int steps = 0;
int stepsGoal = 10;
float steadyStateMin;
float steadyStateMax;
float previousAccX;
float currentAccX;

// Barometer Variables
float currentAlt;
float currentLanding;
int flightsClimbed = 0;
float landingHeight = 2.185;
int flightsGoal = 1;

// Piezo buzzer variables
int buzzerPin = D6;
int turnOnNumNotes = 7;
int turnOnNotes[] = {523,587,698,659,587,523,784};
int turnOnDelays[] = {250,250,500,500,250,500,1000};
int achievedGoalNumNotes = 12;
int achievedGoalNotes[] = {659,698,784,1046,988,1046,784,659,523,587,659,587};
int achievedGoalDelays[] = {1000,1000,400,400,250,400,400,250,400,400,250,400};

// Button variables
int buttonPin = D7;
int buttonHeldLoops = 0;

// OLED variables
int pageNum = 0;
bool screenOn = true;
long screenTime = 10000; // in milliseconds
long startMillis;

// Blynk App Variables
int stepsPin = V0;
int flightsPin = V1;

void setupBlynk()
{
  //Initialize blynk wifi connection
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
}

void setupOLED()
{
  Wire.begin();
  Wire.setClock(400000L);

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  startMillis = millis();
}

void achievedStepsGoal()
{
  if (screenOn == false)
  {
    toggleScreenOnOff();
  }

  // Play sound and print OLED screen
  int temp = pageNum;
  printOLEDScreen(2);
  playBuzzer(achievedGoalNumNotes, achievedGoalNotes, achievedGoalDelays);
  printOLEDScreen(temp);
}

void achievedFlightsGoal()
{
  if (screenOn == false)
  {
    toggleScreenOnOff();
  }

  // Play sound and print OLED screen
  int temp = pageNum;
  printOLEDScreen(3);
  playBuzzer(achievedGoalNumNotes, achievedGoalNotes, achievedGoalDelays);
  printOLEDScreen(temp);
}

void printOLEDScreen(int pageToPrint)
{
  if (pageToPrint == 0)
  {
    // Print page 0 to OLED
    oled.clear();
    oled.print("Steps: ");
    oled.set2X();
    oled.println(steps);
    oled.set1X();
    oled.print("Goal: ");
    oled.set2X();
    oled.println(stepsGoal);
    oled.set1X();
    oled.print("Calories: ");
    oled.set2X();
    oled.println(0.04 * steps);
    oled.set1X();
  }
  else if (pageToPrint == 1)
  {
    // Print page 1 to OLED
    oled.clear();
    oled.print("Flights Climbed: ");
    oled.set2X();
    oled.println(flightsClimbed);
    oled.set1X();
    oled.print("Goal: ");
    oled.set2X();
    oled.println(flightsGoal);
    oled.set1X();
  }
  else if (pageToPrint == 2)
  {
    // Print page 2 to OLED
    oled.clear();
    oled.set2X();
    oled.println("Congrats!");
    oled.set1X();
    oled.println("You reached your");
    oled.println("goal of:");
    oled.set2X();
    oled.print(stepsGoal);
    oled.println(" steps!");
    oled.set1X();
  }
  else if (pageToPrint == 3)
  {
    // Print page 2 to OLED
    oled.clear();
    oled.set2X();
    oled.println("Congrats!");
    oled.set1X();
    oled.println("You reached your");
    oled.println("goal of:");
    oled.set2X();
    oled.print(flightsGoal);
    oled.println(" flights!");
    oled.set1X();
  }
  else if (pageToPrint == 4)
  {
    // Print page 2 to OLED
    oled.clear();
    oled.set2X();
    oled.println("Please");
    oled.println("wait...");
    oled.set1X();
  }
}

void toggleScreenOnOff()
{
  if (screenOn)
  {
    oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
    screenOn = false;
  }
  else
  {
    oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
    screenOn = true;
  }
}

void OLEDLogic()
{
  // Variable for milliseconds
  unsigned long currentMillis = millis();
  
  if (digitalRead(buttonPin) == LOW)
  {
    // Button is pressed

    // restart timer until turn off
    startMillis = currentMillis;

    // Determine what to do with the input
    if (screenOn)
    {
      // Toggle page number
      if (pageNum == 0)
      {
        pageNum = 1;
      }
      else if (pageNum == 1)
      {
        pageNum = 0;
      }

      // Print page
      printOLEDScreen(pageNum);
    }
    else
    {
      toggleScreenOnOff();
    }

    // Check if button has been held for 4 cycles
    buttonHeldLoops++;
    if (buttonHeldLoops >= 4){
      buttonHeld = 0;
      
      // Reset the data
      resetData();
    }

    // Delay so that button is not considered to be pressed twice if held for a while
    delay(300);
  }
  else{
    buttonHeldLoop = 0;
  }

  // if no button press for a certain time, turn off screen
  if ((currentMillis - startMillis >= screenTime) && screenOn)
  {
    toggleScreenOnOff();
  }
}

void getAccelerometerData()
{
  //Assign the current acceleration to the previous
  previousAccX = currentAccX;
  
  //The measurement range can be ±100g or ±200g set by the setRange() function
  //Get new current acceleration from accelerometer
  currentAccX = acce.readAccY();//Get the acceleration in the x direction (actually getting y, but all the variables are x)
}

void getBarometerData()
{
  currentAlt = sensor.readAltitudeM();

  float altitudeDif = currentAlt - currentLanding;
  if (altitudeDif >= 1.9)
  {
    flightsClimbed += 1;
    currentLanding += landingHeight;

    Serial.print("Flights climbed : ");
    Serial.println(flightsClimbed);
    Serial.println();

    // Tell Blynk App new number of flights
    Blynk.virtualWrite(flightsPin, flightsClimbed);
    
    if (flightsClimbed == flightsGoal)
    {
      achievedFlightsGoal();
    }
    else
    {
      // Print OLED
      printOLEDScreen(pageNum);
    }
  }
  else if (altitudeDif <= -1.9)
  {
    currentLanding -= landingHeight;
  }
}

void incrementSteps()
{
  //Check if acceleration changed from above steady state max to under
  if (previousAccX > steadyStateMax && currentAccX <= steadyStateMax)
  {
    steps++;
    Serial.print("Steps: ");
    Serial.println(steps);

    // Tell Blynk App new number of steps
    Blynk.virtualWrite(stepsPin, steps);

    if (steps == stepsGoal)
    {
      achievedStepsGoal();
    }
    else
    {
      // Print OLED
      printOLEDScreen(pageNum);
    }
  }

  //Check if acceleration changed from below steady state min to above
  if (previousAccX < steadyStateMin && currentAccX >= steadyStateMin)
  {
    steps++;

    if (steps == stepsGoal)
    {
      achievedStepsGoal();
    }
    else
    {
      // Print OLED
      printOLEDScreen(pageNum);
    }
  }
}

void playBuzzer(int numNotes, int notes[], int delays[])
{
  // Loop through each note and play it for the assigned duration
  for(int i = 0; i < numNotes; i++)
  {
    tone(buzzerPin, notes[i], delays[i]);
  	delay(delays[i]);
  }
}

void setupAccelerometer()
{
  //Chip initialization
  while(!acce.begin()){
     Serial.println("Initialization failed, please check the connection and I2C address settings");
     delay(1000);
  }
  //Get chip id
  Serial.print("chip id : ");
  Serial.println(acce.getID(),HEX);
  /**
    set range:Range(g)
              eLIS2DH12_2g,/< ±2g>/
              eLIS2DH12_4g,/< ±4g>/
              eLIS2DH12_8g,/< ±8g>/
              eLIS2DH12_16g,/< ±16g>/
  */
  acce.setRange(/*Range = */DFRobot_LIS2DH12::eLIS2DH12_16g);
  /**
    Set data measurement rate：
      ePowerDown_0Hz 
      eLowPower_1Hz 
      eLowPower_10Hz 
      eLowPower_25Hz 
      eLowPower_50Hz 
      eLowPower_100Hz
      eLowPower_200Hz
      eLowPower_400Hz
  */
  acce.setAcquireRate(/*Rate = */DFRobot_LIS2DH12::eLowPower_10Hz);
  delay(1000);
}

void setupBarometer()
{
  int rslt;
  while( ERR_OK != (rslt = sensor.begin()) )
  {
    if(ERR_DATA_BUS == rslt)
    {
      Serial.println("Data bus error!!!");
    }else if(ERR_IC_VERSION == rslt)
    {
      Serial.println("Chip versions do not match!!!");
    }
    delay(3000);
  }
  Serial.println("Begin ok!");

  while( !sensor.setSamplingMode(sensor.eUltraPrecision) )
  {
    Serial.println("Set samping mode fail, retrying....");
    delay(3000);
  }

  delay(100);
  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  if( sensor.calibratedAbsoluteDifference(540.0) )
  {
    Serial.println("Absolute difference base value set successfully!");
  }
  #endif

  float sampingPeriodus = sensor.getSamplingPeriodUS();
  Serial.print("samping period : ");
  Serial.print(sampingPeriodus);
  Serial.println(" us");

  float sampingFrequencyHz = 1000000 / sampingPeriodus;
  Serial.print("samping frequency : ");
  Serial.print(sampingFrequencyHz);
  Serial.println(" Hz");

  Serial.println();
}

void getSteadyState(void)
{
  Serial.println("Calculating Steady State Values");

  // Get first acceleration
  float x = acce.readAccX();

  // Set original min and max to the first x value read
  steadyStateMin = x;
  steadyStateMax = x;

  // Reapeat 10000 times
  for(int i = 0; i < 1000; i++){

    // Get acceleration
    x = acce.readAccX();

    if (x < steadyStateMin){
      // If x is less than the current min, set the min to x
      steadyStateMin = x;
    }
    if (x > steadyStateMax){
      // If x is more than the current max, set the max to x
      steadyStateMax = x;
    }

    // Delay
    delay(10);
  }

  // Print min and max
  Serial.print("Min = ");
  Serial.println(steadyStateMin);
  Serial.print("Max = ");
  Serial.println(steadyStateMax);

  startMillis = millis();
}

void resetData(){
  // Run this function at the start and whenever resetting the program

  // Reset steps and flights
  steps = 0;
  flightsClimbed = 0;

  // Tell Blynk App new number of steps and flights
  Blynk.virtualWrite(stepsPin, steps);
  Blynk.virtualWrite(flightsPin, flights);

  // Get original landing altitude
  currentAlt = sensor.readAltitudeM();
  currentLanding = currentAlt;

  // Get original steps
  currentAccX = acce.readAccY();

  // Set original OLED Screen
  setupOLED();

  // Print please wait message to oled
  printOLEDScreen(4);

  // Play sound for turn on
  pinMode(buzzerPin, OUTPUT);
  playBuzzer(turnOnNumNotes, turnOnNotes, turnOnDelays);

  // Calculate the steady state of the accelerometer
  getSteadyState();

  // Setup Button
  pinMode(buttonPin, INPUT_PULLUP);

  printOLEDScreen(pageNum);
}

BLYNK_WRITE(V2) // Executes when the value of virtual pin 1 changes
{
  stepsGoal = param.asInt();
  printOLEDScreen(pageNum);
  Serial.print("Steps Goal: ");
  Serial.println(stepsGoal);
}

BLYNK_WRITE(V3) // Executes when the value of virtual pin 1 changes
{
  flightsGoal = param.asInt();
  printOLEDScreen(pageNum);
  Serial.print("Flights Climbed Goal: ");
  Serial.println(flightsGoal);
}

void setup(void)
{
  // Begin serial communication
  Serial.begin(9600);

  // Setup devices
  setupBlynk();
  setupAccelerometer();
  setupBarometer();

  // Reset steps, flights, etc. and calc steady state
  resetData();

  // Tell Blynk App starting stats
  Blynk.virtualWrite(stepsPin, steps);
  Blynk.virtualWrite(flightsPin, flightsClimbed);
}

void loop(void)
{
  // Blynk
  Blynk.run();
  Blynk.virtualWrite(stepsPin, steps);
  Blynk.virtualWrite(flightsPin, flightsClimbed);

  // Get sensor data
  getAccelerometerData();
  getBarometerData();

  // Logic involving oled display (button input and turning on/ off)
  OLEDLogic();

  // Increment steps if satisfying conditions
  incrementSteps();

  delay(100);
}
