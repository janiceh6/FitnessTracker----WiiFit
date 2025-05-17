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
float steadyStateMin = -400;
float steadyStateMax = 400;
float previousAccX;
float currentAccX;

// Barometer Variables
float currentAlt;
float currentLanding;
int flightsClimbed = 0;
float landingHeight = 2;
int flightsGoal = 1;

// Piezo buzzer variables
int buzzerPin = 11;
int turnOnNumNotes = 7;
int turnOnNotes[] = {523,587,698,659,587,523,784};
int turnOnDelays[] = {250,250,500,500,250,500,1000};
int achievedGoalNumNotes = 12;
int achievedGoalNotes[] = {659,698,784,1046,988,1046,784,659,523,587,659,587};
int achievedGoalDelays[] = {1000,1000,400,400,250,400,400,250,400,400,250,400};

// Button variables
int buttonPin = 10;

// OLED variables
int pageNum = 0;
bool screenOn = true;
long screenTime = 10000; // in milliseconds
long startMillis;

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

void printOLEDScreen(int pageToPrint)
{
  if (pageToPrint == 0)
  {
    // Print page 0 to OLED
    oled.clear();
    oled.set2X();
    oled.print("Steps: ");
    oled.println(steps);
    oled.print("Goal: ");
    oled.println(stepsGoal);
    oled.print("Est. Calories: ");
    oled.println(0.04 * steps);
  }
  else if (pageToPrint == 1)
  {
    // Print page 1 to OLED
    oled.clear();
    oled.set2X();
    oled.print("Flights Climbed: ");
    oled.println(flightsClimbed);
    oled.print("Goal: ");
    oled.println(flightsGoal);
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
  
  if (digitalRead(buttonPin) == HIGH)
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
  currentAccX = acce.readAccX();//Get the acceleration in the x direction
}

void getBarometerData()
{
  currentAlt = sensor.readAltitudeM();

  float altitudeDif = currentAlt - currentLanding;
  if (altitudeDif >= 1.9)
  {
    flightsClimbed += 1;
    currentLanding += landingHeight;
    
    // Print OLED
    printOLEDScreen(pageNum);
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

    // Print OLED
    printOLEDScreen(pageNum);
  }

  //Check if acceleration changed from below steady state min to above
  if (previousAccX < steadyStateMin && currentAccX >= steadyStateMin)
  {
    steps++;

    // Print OLED
    printOLEDScreen(pageNum);
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

void setup(void)
{

  // Begin serial communication
  Serial.begin(9600);
  
  setupAccelerometer();

  setupBarometer();

  // Get original landing altitude
  currentAlt = sensor.readAltitudeM();
  currentLanding = currentAlt;

  // Play sound for turn on
  pinMode(buzzerPin, OUTPUT);
  playBuzzer(turnOnNumNotes, turnOnNotes, turnOnDelays);

  // Setup Button
  pinMode(buttonPin, INPUT);

  // Set original OLED Screen
  printOLEDScreen(pageNum);
}

void loop(void)
{

  getAccelerometerData();

  getBarometerData();

  OLEDLogic();

  // Print Acceleration and Steps
  /*Serial.print("Acceleration x: ");
  Serial.println(currentAccX);*/

  Serial.print("Steps: ");
  Serial.println(steps);
  incrementSteps();

  // Print barometer data and flights climbed
  /*Serial.print("Altitude : ");
  Serial.print(currentAlt);
  Serial.println(" m");

  Serial.print("Current Landing : ");
  Serial.print(currentLanding);
  Serial.println(" m");*/

  Serial.print("Flights climbed : ");
  Serial.println(flightsClimbed);
  Serial.println();

  delay(300);

}
