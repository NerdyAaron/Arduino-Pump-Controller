#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the I2C address of the LCD
#define I2C_ADDRESS 0x27

// Create an instance of the LiquidCrystal_I2C class
LiquidCrystal_I2C lcd(I2C_ADDRESS, 20, 4); //set the LCD address to 0x27 for a 20 chars and 4 line display

const int flowSensorPin = 2;
const int relayPin = 9;
const int batteryVoltagePin = A0;
const int userInteractionPin = 13; // Pin for momentary switch
int switchState = LOW; // Variable to store the state of the momentary switch

// voltage to check for before running the pump
const float batteryVoltageThreshold = 11.5; 

// Declare a variable to store the total run time.
unsigned long totalRunTime = 0;

float currentCycleVolume = 0.0;
float totalVolumePumped = 0.0;



volatile unsigned long pulseCount = 0;

float pumpCycleInterval = 100000; // milliseconds (10 minutes)
float pumpOnTime = 10000; // milliseconds (30 seconds)

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

unsigned long backlightTimeout = 60000; // Timeout in milliseconds for LCD backlight
unsigned long lastInteractionTime = 0; // Time of last user interaction
bool backlightOn = false; // Flag to indicate backlight state
unsigned long manualStartTime = 0; // Initialize manualStartTime to 0 used to store the pump start time when manual button is used

//functions

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
    lcd.print("Next Pump:");
    lcd.print(timeRemaining / 60000);
    lcd.print(" m ");
    lcd.print((timeRemaining / 1000) % 60);
    lcd.print(" s");

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
  }
}

//Function to turn the back light on when momentary switch is pressed - may look for more power saving in the future
void userInteractionDetected() {
  lastInteractionTime = millis();
  backlightOn = true;
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
}

// Define the flowPulse function here
void flowPulse() {
  pulseCount++; // Increment the pulse count whenever an interrupt is triggered
}

void loop() {
  // Read the battery voltage every minute
  if (lastBatteryReadingTime == 0 || millis() - lastBatteryReadingTime >= 60000) {
    batteryVoltage = readBatteryVoltage();
    lastBatteryReadingTime = millis();
  }

  // Turn the backlight off if there has been no user interaction within backlightTimeout timer
  if (millis() - lastInteractionTime >= backlightTimeout && backlightOn) {
    backlightOn = false;
  }


  updateLCD();

// Check if the momentary switch is pressed
switchState = digitalRead(userInteractionPin);
if (switchState == HIGH) {
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



  }
}


  // Check if it is time to turn on the pump.
  currentTimerState = millis();
  if (currentTimerState >= previousPumpCycleTime + pumpCycleInterval) {
    
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

          // Update the LCD display
          updateLCD();

          // Delay for 1 second.
          delay(1000);

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



  // Delay for 1 second.
  delay(1000);
}
