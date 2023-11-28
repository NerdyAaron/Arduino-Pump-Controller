#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>
#include <LowPower.h>

// Define the I2C address of the LCD
#define I2C_ADDRESS 0x27

// Create an instance of the LiquidCrystal_I2C class
LiquidCrystal_I2C lcd(I2C_ADDRESS, 20, 4); //set the LCD address to 0x27 for a 20 chars and 4 line display

float pumpCycleInterval = 900000; // milliseconds (15 minutes)
float pumpOnTime = 30000; // milliseconds (30 seconds)
float safetyDelay = 3000; // milliseconds to turn off pump if no water flow 
const int flowSensorPin = 2;
const int relayPin = 9;
const int batteryVoltagePin = A0;
const int userInteractionPin = 13; // Pin for momentary switch
const int pumpSwitchPin = 12; // Pin for momentary switch

// Define DS1302 clock pins
const int CLK = 5;  // RTC Clock
const int DAT = 4;  // RTC Data
const int RST = 3;  // RTC Reset

int userSwitchState = LOW; // Variable to store the state of the backlight switch (not used)
int pumpSwitchState = LOW; // Variable to store the state of the manual pump switch

// voltage to check for before running the pump
const float batteryVoltageThreshold = 12; 

// Declare a variable to store the total run time.
unsigned long totalRunTime = 0;

float currentCycleVolume = 0.0;
float totalVolumePumped = 0.0;

// Define a flag to track flow detection
bool flowDetected = false;

volatile unsigned long pulseCount = 0;

float batteryVoltage;
float voltage;

int pumpState = LOW;
unsigned long previousPumpCycleTime = 0;
unsigned long currentTimerState = 0;
unsigned long timeRemaining = 0;

unsigned long cycleTime = 0;
unsigned long totalWaterDuringCycle = 0;
unsigned long lastCycleTime = 0;
unsigned long endPulseCount = 0;
unsigned long lastBatteryReadingTime = 0;

unsigned long backlightTimeout = 180000; // Timeout in milliseconds for LCD backlight
unsigned long lastInteractionTime = 0; // Time of last user interaction
bool backlightOn = false; // Flag to indicate backlight state
unsigned long manualStartTime = 0; // Initialize manualStartTime to 0 used to store the pump start time when manual button is used

//functions

ThreeWire myWire(DAT, CLK, RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);


// Function to read the battery voltage
float readBatteryVoltage() {
  int adcReading = analogRead(A0);
  float voltage = adcReading * 4.9 / 1023;
  return voltage * 2.85;
}

// Function to calculate the current cycle volume
float calculateCurrentCycleVolume() {
  return ((pulseCount - endPulseCount) / 330) * 0.264172;
}

// Function to calculate total volume pumped
float calculateTotalCycleVolume() {
  return (pulseCount / 330) * 0.264172;
}


// Function to update the LCD display
void updateLCD() {
  if (backlightOn) {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print(timeRemaining / 60000);
    lcd.print(":");
    lcd.print((timeRemaining / 1000) % 60);
    
    
    // Read and display the current time from the RTC
    RtcDateTime currentTime = Rtc.GetDateTime();
    
    lcd.setCursor(15, 0); // Display time in the top right corner
    lcd.print(currentTime.Hour());
    lcd.print(":");
    lcd.print(currentTime.Minute());

    lcd.setCursor(0, 1);
    lcd.print(" Battery:");
    lcd.print(batteryVoltage);

    lcd.setCursor(0, 2);
    lcd.print(" Run Time: ");
    lcd.print(totalRunTime / 60000);
    lcd.print(" m ");
    lcd.print((totalRunTime / 1000) % 60);
    lcd.print(" s ");

    lcd.setCursor(0, 3);
    lcd.print(" Gallons: ");
    lcd.print(totalVolumePumped);
    } else {
        lcd.noBacklight(); // Turn off backlight
        }
  }

//Function to turn the back light on when momentary switch is pressed - may look for more power saving in the future
void userInteractionDetected() {
  lastInteractionTime = millis();
  backlightOn = true;
  updateLCD(); 
}


void setup() {
  // Set the relay pin as an output.
  pinMode(relayPin, OUTPUT);

  // Set the battery voltage pin as an input.
  pinMode(batteryVoltagePin, INPUT);

  // Set the flow sensor pin as an input.
  pinMode(flowSensorPin, INPUT);

  // Attach an interrupt to the flow sensor pin.
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowPulse, RISING); // Trigger interrupt on rising edge

  // Set the user interaction pin as an input.
  pinMode(userInteractionPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(userInteractionPin), userInteractionDetected, RISING); // Trigger interrupt on rising edge

  // Initialize the serial monitor.
  Serial.begin(9600);

  // Initialize the LCD 
  lcd.init();
  
  // Initialize RTC
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid()) {
    Rtc.SetDateTime(compiled);
  }
}

// Define the flowPulse function here
void flowPulse() {
  pulseCount++; // Increment the pulse count whenever an interrupt is triggered
}

void loop() {
  RtcDateTime currentTime = Rtc.GetDateTime(); // Fetch current time
  int currentHour = currentTime.Hour(); // Get current hour

  if ((currentHour >= 18) || (currentHour < 8)) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  } else {

  // Read the battery voltage every minute
  if (lastBatteryReadingTime == 0 || millis() - lastBatteryReadingTime >= 60000) {
    batteryVoltage = readBatteryVoltage();
    lastBatteryReadingTime = millis();
  }

  // Turn the backlight off if there has been no user interaction within backlightTimeout timer
  if (millis() - lastInteractionTime >= backlightTimeout && backlightOn) {
    backlightOn = false;
  }

// Check if the pump manual switch is pressed
pumpSwitchState = digitalRead(pumpSwitchPin);
if (pumpSwitchState == HIGH) {
  userInteractionDetected(); //Turn on backlight - may call updateLCD too frequently -keeps bl on while button pressed
  if (manualStartTime == 0) {
    manualStartTime = millis();
    
  }
  // Turn on the pump only if it's not already ON
  if (pumpState != HIGH) {
    pumpState = HIGH;
    digitalWrite(relayPin, pumpState);
    // Delay for 2 seconds to avoid someone chattering the pump.
    delay(2000);
  }
} else {
  // Turn off the pump only if it's not already OFF
  if (pumpState != LOW) {
    pumpState = LOW;
    digitalWrite(relayPin, pumpState);

    // Store last cycles ending pulseCount
    endPulseCount = pulseCount;

    //Update the total volume pumped
    totalVolumePumped = calculateTotalCycleVolume();
    
    // If manual start time was set, update totalRunTime
    if (manualStartTime != 0) {
      totalRunTime += millis() - manualStartTime;
      manualStartTime = 0;
    }

    // Update the previous pump cycle time to reset timer upon manual use
      previousPumpCycleTime = millis();

  }
}


  // Check if it is time to turn on the pump.
  currentTimerState = millis();
    if(currentTimerState >= previousPumpCycleTime + pumpCycleInterval) {
    
    // Check if the battery voltage is above the threshold. Otherwise skip this round.
    if (batteryVoltage >= batteryVoltageThreshold) {
      // Turn on the pump.
      pumpState = HIGH;
      digitalWrite(relayPin, pumpState);

      // Start the timer.
      unsigned long startTime = millis();
      
      while (millis() - startTime < pumpOnTime) {
        // Check for pulses from the flow sensor.
        if (pulseCount > endPulseCount) {
          
          
          // Calculate the current amount of water that has passed through the flow meter this cycle
          currentCycleVolume = calculateCurrentCycleVolume();

          //Update the total volume pumped
          totalVolumePumped = calculateTotalCycleVolume();
          
          //Safety Check to ensure pump is not running dry
          endPulseCount = 0; //reset end count each loop and it will trigger the else safety check if there are no additional pulses after the 1 second delay

          // Update the LCD display
          updateLCD();

          // Delay for 1 second.
          delay(1000);        
      } else {
      //check if pump has been on for safetDelay without water flow 
      if (millis() - startTime > safetyDelay){
      // Turn off the pump.
      pumpState = LOW;
      digitalWrite(relayPin, pumpState);

      // Stop the timer and calculate the total run time.
      unsigned long endTime = millis();
      totalRunTime += endTime - startTime;

      // Update the previous pump cycle time.
      previousPumpCycleTime = currentTimerState;

      // Store last cycles ending pulseCount
      endPulseCount = pulseCount;
      
    // Display error message
    lcd.clear();
    backlightOn = true;
    lcd.setCursor(0, 0);
    lcd.print("No Flow Detected.");
    lcd.setCursor(0, 1);
    lcd.print("Press Button to Restart.");
    
    //Add ability to have flashing error notification LED

    // Wait for user interaction
    while (digitalRead(userInteractionPin) == LOW) {
      delay(100);
    }

    // Reset flow flag
    flowDetected = false;

    // Clear error message
    lcd.clear();
          }
              }
      }
      // Turn off the pump.
      pumpState = LOW;
      digitalWrite(relayPin, pumpState);

      // Stop the timer and calculate the total run time.
      unsigned long endTime = millis();
      totalRunTime += endTime - startTime;

      // Update the previous pump cycle time.
      previousPumpCycleTime = currentTimerState;

      // Store last cycles ending pulseCount
      endPulseCount = pulseCount;

    
    }
  }

  // Calculate the time remaining until the next pump cycle.
  timeRemaining = pumpCycleInterval - (currentTimerState - previousPumpCycleTime);



  // Delay for 500 milli seconds.
  delay(500);
}
