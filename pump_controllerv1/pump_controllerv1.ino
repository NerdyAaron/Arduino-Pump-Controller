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

//Debugging
float morningTime = 8; //Hour to exit sleep mode
float nightTime = 16; //Military time hour to enter sleep mode
float pumpCycleInterval = 60000; // milliseconds (15 minutes)
float pumpOnTime = 40000; // milliseconds (40 seconds)
float safetyDelay = 3000; // milliseconds to turn off pump if no water flow 
const int flowSensorPin = 2;
const int relayPin = 9;
const int batteryVoltagePin = A0;
const int userInteractionPin = 3; // Pin for momentary switch
const int pumpSwitchPin = 12; // Pin for momentary switch

// Define DS1302 clock pins
const int CLK = 5;  // RTC Clock
const int DAT = 4;  // RTC Data
const int RST = 6;  // RTC Reset

int userSwitchState = LOW; // Variable to store the state of the backlight switch (not used)
int pumpSwitchState = LOW; // Variable to store the state of the manual pump switch

// voltage to check for before running the pump
const float batteryVoltageThreshold = 12.5; 

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

unsigned long backlightTimeout = 600000; // Timeout in milliseconds for LCD backlight
unsigned long lastInteractionTime = 0; // Time of last user interaction
bool backlightOn = false; // Flag to indicate backlight state
unsigned long manualStartTime = 0; // Initialize manualStartTime to 0 used to store the pump start time when manual button is used

//functions

ThreeWire myWire(DAT, CLK, RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt) {
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial.print(datestring);
}


// Function to read the battery voltage
float readBatteryVoltage() {
  analogReference(DEFAULT); // Set the analog reference to the default (usually 5V)
  int adcReading = analogRead(A0);
  float voltage = adcReading * 4.9 / 1023;
  Serial.print("ADC: ");
  Serial.println(adcReading);
  Serial.print("Voltage: ");
  Serial.println(voltage);
  voltage = voltage *3.1; //Bridge multiplier
   Serial.print("Final: ");
  Serial.println(voltage);
  return voltage;
 
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

    lcd.backlight();

//Display a count down until next pump run
// Calculate minutes and seconds from the remaining time
unsigned long minutes = timeRemaining / 60000; // Convert milliseconds to minutes
unsigned long seconds = (timeRemaining / 1000) % 60; //Convert milliseconds to seconds

String formattedMinutes = (minutes < 10) ? "0" + String(minutes) : String(minutes);
String formattedSeconds = (seconds < 10) ? "0" + String(seconds) : String(seconds);


// Display the formatted time on the LCD
lcd.setCursor(0, 0);
lcd.print(formattedMinutes + ":" + formattedSeconds);
    
    
    // Read and display the current time from the RTC
    // Get current time from RTC module
    RtcDateTime currentTime = Rtc.GetDateTime();

    // Convert hours to a two-digit format
    String formattedHours = currentTime.Hour() < 10 ? "0" + String(currentTime.Hour()) : String(currentTime.Hour());

    // Format minutes to two-digit format
    formattedMinutes = currentTime.Minute() < 10 ? "0" + String(currentTime.Minute()) : String(currentTime.Minute());

    // Display time in military format on the LCD
    lcd.setCursor(15, 0);
    lcd.print(formattedHours);
    lcd.print(":");
    lcd.print(formattedMinutes);;

    // Calculate minutes and seconds from the remaining time
    unsigned long runTimeHours = totalRunTime / 3600000; //Convert milliseconds to hours
    unsigned long runTimeMinutes = totalRunTime / 60000; // Convert milliseconds to minutes
    unsigned long runTimeSeconds = (totalRunTime / 1000) % 60; //Convert milliseconds to seconds

    // Format the run time with leading zeros
    String formattedrunTimeHours = runTimeHours < 10 ? "0" + String(runTimeHours) : String(runTimeHours);
    String formattedrunTimeMinutes = runTimeMinutes < 10 ? "0" + String(runTimeMinutes) : String(runTimeMinutes);
    String formattedrunTimeSeconds = runTimeSeconds < 10 ? "0" + String(runTimeSeconds) : String(runTimeSeconds);

    // Display the formatted run time on the LCD
    lcd.setCursor(0, 1);
    lcd.print("Run Time: ");
    lcd.print(formattedrunTimeHours);
    lcd.print(":");
    lcd.print(formattedrunTimeMinutes);
    lcd.print(":");
    lcd.print(formattedrunTimeSeconds);

    //Debugging
    Serial.print("Run Time: ");
    Serial.print(formattedHours);
    Serial.print(":");
    Serial.print(formattedMinutes);
    Serial.print(":");
    Serial.print(formattedSeconds);
  
    lcd.setCursor(0, 2);
    lcd.print("Gallons: ");
    lcd.print(totalVolumePumped);

    lcd.setCursor(0, 3);
    lcd.print("Battery:");
    lcd.print(batteryVoltage);
    
  }

//Function to turn the back light on when momentary switch is pressed - may look for more power saving in the future
void userInteractionDetected() {
  // Clear the interrupt flag to prevent continuous triggering
  detachInterrupt(digitalPinToInterrupt(userInteractionPin));
  lastInteractionTime = millis();
  delayMicroseconds(10000); // Delay for 10 milliseconds
  backlightOn = true;
  //Serial.println(" User Interaction Detected ");
  
  // Check if enough time has passed since the last interaction
  if (millis() - lastInteractionTime > 1000) { // Adjust the debounce time as needed
    userSwitchState = digitalRead(userInteractionPin);
    if (userSwitchState == LOW) { // Check for LOW state when the switch is pressed
      Serial.println(" User Switch Pressed ");
    }
  }
  
  attachInterrupt(digitalPinToInterrupt(userInteractionPin), userInteractionDetected, FALLING); // Re-attach interrupt
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
  pinMode(userInteractionPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(userInteractionPin), userInteractionDetected, FALLING); // Trigger interrupt on rising edge

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
  

  if (!Rtc.IsDateTimeValid()) {
    // Common Causes:
    //    1) first time you ran and the device wasn't running yet
    //    2) the battery on the device is low or even missing

    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

 if (Rtc.GetIsWriteProtected()) {
    Serial.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  } else if (now > compiled) {
    Serial.println("RTC is newer than compile time. (this is expected)");
  } else if (now == compiled) {
    Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }
  Serial.println(" End of setup funtion ");
}

// Define the flowPulse function here
void flowPulse() {
  pulseCount++; // Increment the pulse count whenever an interrupt is triggered
}

void loop() {
  
  RtcDateTime currentTime = Rtc.GetDateTime(); // Fetch current time
  //printDateTime(currentTime);
  //Serial.println();
  //Serial.println(" Check Point 1 ");
  
  if (!currentTime.IsValid()) {
    // Common Causes:
    //    1) the battery on the device is low or even missing and the power line was disconnected
    Serial.println("RTC lost confidence in the DateTime!");
  }


  int currentHour = currentTime.Hour(); // Get current hour

  if ((currentHour >= nightTime) || (currentHour < morningTime)) {
    Serial.println(" Low Power Mode ");
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  } else {

    updateLCD(); //LCD updates if we are not in low power mode

  // Read the battery voltage every minute
  if (lastBatteryReadingTime == 0 || millis() - lastBatteryReadingTime >= 15000) {
    batteryVoltage = readBatteryVoltage();
    lastBatteryReadingTime = millis();
  }

  // Turn the backlight off if there has been no user interaction OR PUMP within backlightTimeout timer
  if (millis() - lastInteractionTime >= backlightTimeout && backlightOn) {
    backlightOn = false; //should be false when we want to use. keeping it on for dev
    Serial.println(" Back Light Turned Off ");
  }
//Serial.println(" Pump Switch Check ");
// Check if the pump manual switch is pressed
pumpSwitchState = digitalRead(pumpSwitchPin);
if (pumpSwitchState == HIGH) {
  Serial.println(" Pump Switch Pressed ");

  if (manualStartTime == 0) {
    manualStartTime = millis();
    //Delay for half a second
    delay(500);
  }
  // Turn on the pump only if it's not already ON
  if (pumpState != HIGH) {
    pumpState = HIGH;
    Serial.println(" Turning Pump On ");
    digitalWrite(relayPin, pumpState);
    backlightOn = true;
    updateLCD();
    // Delay for 2 seconds to avoid someone chattering the pump. Move this to a check before turing the pump off - same safety check but starts the timer 
    delay(2000);
    
  }

if (manualStartTime != 0) {
      totalRunTime += millis() - manualStartTime;
      manualStartTime = 0;
      
    }
    Serial.print( "Total Run Time ");
    Serial.println(totalRunTime);
    lastInteractionTime = millis(); 
    backlightOn = true;

    updateLCD();

} else {
  // Turn off the pump only if it's not already OFF
  if (pumpState != LOW) {
    pumpState = LOW;
    digitalWrite(relayPin, pumpState);
    Serial.println( "Turn Pump Off ");

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
    Serial.println(" No Flow Detected Error ");
    
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
   }

  // Calculate the time remaining until the next pump cycle.
  timeRemaining = pumpCycleInterval - (currentTimerState - previousPumpCycleTime);



  // Delay for 500 milli seconds.
  delay(500);
}
