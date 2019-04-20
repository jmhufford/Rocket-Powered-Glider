// --------------------------------------------------------------------------
// File: AutopilotV4.ino
// Author: Jerome Hufford
// --------------------------------------------------------------------------
// Function:
// A flight controller with a manual override built for a rocket propelled 
// glider utilizing a curie nano. 
// --------------------------------------------------------------------------
#include <CurieIMU.h>
#include <Servo.h>
//#include <stdio.h>
#include "Globals.h"
#include "JHFilter.h"
#include "JHGuidance.h"
#include "JHController.h"

int loopCounter = 0;
int aix, aiy, aiz;
int gip, giq, gir;

JHFilter filter;
JHGuidance guidance;
JHController controller;
Servo right;
Servo left;
Servo rudder;

float hz = g_ReadRateHz;

const int ledPin =  LED_BUILTIN;
int ledState = LOW;     
unsigned long previousMillis = 0;    
const long interval = 500;       
unsigned long microsPerReading, microsPrevious, microsNow;

// Set these pin numbers to any of the non-PWM digital pins (Note: 4 & 7 are valid)
int orPinNum = 2; // Digital pin number for the override switch (from tx/rx)
int elPinNum = 4; // Digital pin number for the elevator command (from tx/rx)

// Global override flag
bool manualOverride = false;

// Thresholds for what is considered "on" for the override switch
static int orPwmThresh = 1500; // Units: microseconds

// Initialization of global variables needed for manual PWM capture (i.e. the failsafe feature)
volatile int orPwmValue = 0;
volatile int elPwmValue = 0;
volatile int orStartTime = 0;
volatile int elStartTime = 0;

void setup() {
  // ------------------------------------------------------------------------
  // SETUP - This function only runs once at power on / reset
  // ------------------------------------------------------------------------

  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);

  setupCurie();
  
  // Initialize state machine, nav (state est.), guidance, and control loops
  filter.begin(g_ReadRateHz, g_RCV, g_QCV); 
  filter.zeroAngles(aix,aiy,aiz);
  //attach pins to servos
  right.attach(6);
  left.attach(7);
  rudder.attach(5);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / hz;
  microsPrevious = micros();

  // Initialize the interrupt to look for a rising edge on the override pin
  attachInterrupt(digitalPinToInterrupt(orPinNum), orRising, RISING);
}

void loop() {
    
    microsNow = micros();
    
    if (microsNow - microsPrevious >= microsPerReading) {
        microsPrevious = microsNow;
        
        //loopCounter += 1;
        //Serial.println(loopCounter);

        if (manualOverride) {
           //Serial.println("Override");
           manualMode();
         }
         else {
            autoMode();  
         }
         //find time one run takes
         //printStatus();
         //Serial.println(micros() - microsNow);
    }
    
    blinkLED();
}//end main loop


// ------------------------------------------------------------------------
// Extra Functions Here
// ------------------------------------------------------------------------

void setupCurie() {
  noInterrupts(); // Turn off all interrupts

  CurieIMU.begin();
  CurieIMU.autoCalibrateGyroOffset();

  CurieIMU.setGyroRate(g_ReadRateHz);
  CurieIMU.setAccelerometerRate(g_ReadRateHz);
 
  // Set the accelerometer range to 4G
  CurieIMU.setAccelerometerRange(4);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(500);

  //remove this later?
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  CurieIMU.readMotionSensor(aix, aiy, aiz, gip, giq, gir);
  interrupts(); // Turn back on all interrupts
}

void autoMode() {
  
  noInterrupts(); // Turn off all interrupts
  CurieIMU.readMotionSensor(aix, aiy, aiz, gip, giq, gir);
  interrupts(); // Turn back on all interrupts
 
  filter.updateIMU(aix, aiy, aiz, gip, giq, gir);
  guidance.inputData(filter);
  float headingCommand = guidance.getHeadingCommand();
  float rollCommand = guidance.getRollCommand();
  float pitchCommand = guidance.getPitchCommand();
  //controller.headingController(headingCommand, highpassP.output());
  controller.rollController(rollCommand, filter.getRoll(), filter.getP());
  controller.pitchController(pitchCommand, filter.getPitch(), filter.getQ());
  controller.calculateDelta(.9);

  left.write(controller.leftElevator());
  right.write(controller.rightElevator());
  rudder.write(controller.rudder());
}

void manualMode() {
  float val = ((elPwmValue * 15)/77) - (15930/77);
  left.write(180-val);
  right.write(val);
  rudder.write(90);
}

void printStatus() {
        
     Serial.print(" Heading = ");
  //Serial.print(heading);
  Serial.print("          roll ");
  Serial.print(filter.getRoll());
  Serial.print(" pitch ");
  Serial.print(filter.getPitch());
  Serial.print(" left ");
  Serial.print(controller.leftElevator());
   Serial.print(" right  ");
  Serial.print(controller.rightElevator());
   Serial.print(" rudder ");
  Serial.print(controller.rudder());
  Serial.print(" override ");
  Serial.print(orPwmValue);
  Serial.println();
  
}

void blinkLED() {
  unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) 
    {
        previousMillis = currentMillis;
        if (ledState == LOW) {
            ledState = HIGH;
         } else {
              ledState = LOW;
         }
    digitalWrite(ledPin, ledState);
  
    }
}


// This function runs when the rising edge of the override pin occurs
void orRising() {

  // Grab current time when signal went high
  orStartTime = micros(); // Units: microseconds
  // Start the interrupt that fires when the falling edge (signal low) is detected
  attachInterrupt(digitalPinToInterrupt(orPinNum), orFalling, FALLING);
}

// This function runs when the falling edge of the override pin occurs
void orFalling() {

  // Grab difference between current time (when falling edge occured) and  when
  // the rising edge occured - the delta is the pulse width
  orPwmValue = micros() - orStartTime; // Units: microseconds

  // Check if the threshold was met (i.e. the override switch was flipped on the Tx)
  if (orPwmValue > orPwmThresh) {
    // If so, set manual override flag true and start elevator cmd interrupt
    if (!manualOverride){
      manualOverride = true;
      attachInterrupt(digitalPinToInterrupt(elPinNum), elRising, RISING);
    }

  }
  else {
    
    // If not, set manual override flag false and detach elevator cmd interrupt
    manualOverride = false;
    detachInterrupt(digitalPinToInterrupt(elPinNum));
  }
  
  // Restart the interrupt that fires when the rising edge (signal high) is detected
  attachInterrupt(digitalPinToInterrupt(orPinNum), orRising, RISING);
}

// This function runs when the rising edge of the elevator PWM command occurs
void elRising() {
  // Grab current time when signal went high
  elStartTime = micros(); // Units: microseconds

  // Start the interrupt that fires when the falling edge (signal low) is detected
  attachInterrupt(digitalPinToInterrupt(elPinNum), elFalling, FALLING);
}

// This function runs when the falling edge of the elevator PWM command occurs
void elFalling() {
  // Grab difference between current time (when falling edge occured) and  when
  // the rising edge occured - the delta is the pulse width
  elPwmValue = micros() - elStartTime; // Units: microseconds

  // Start the interrupt that fires when the falling edge (signal low) is detected
  attachInterrupt(digitalPinToInterrupt(elPinNum), elRising, RISING);
}
