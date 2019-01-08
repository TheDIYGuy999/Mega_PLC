//
// =======================================================================================================
// PROJECT DESCRIPTION
// =======================================================================================================
//
// Screw tightening robot with 16x2 I2C display, automatic scew feeder

// Arduino Board:         Due
// Mainmoard:             Mega PLC Mainboard V1.0
// Digital input board:   Mega PLC Digital Input Board V1.0
// Digital output board:  Mega PLC Digital Output Board V1.0
// Stepper drivers:       Watterott Silent Step Stick with TMC2100
// Stepper motors:        NEMA 17 "High Torque" from fredee.ch 0.65Nm @ 1.5A, 4V

// Mega PLC supplier:     TheDIYGuy999: https://github.com/TheDIYGuy999/Mega_PLC

//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

// STEPPER
#include <AccelStepper.h> // http://www.airspayce.com/mikem/arduino/AccelStepper/

// DISPLAY
#include <Wire.h> // I2C library
#include <LiquidCrystal_I2C.h> // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

// KEYPAD
#include <Keypad.h> // http://playground.arduino.cc/Code/Keypad

// SPI BUS FOR SD CARD
#include <SPI.h>
#include <SD.h>

// TIMER
#include <SimpleTimer.h> // https://github.com/jfturcot/SimpleTimer

// Cylinder monitoring
#include "cylinderPositionMonitoring.h"

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

//
// Digital 0, 1, Shared as serial pins (Serial1)
// Digital 2 - 13, General Purpose
// Digital 3, 5, 6, 9, 10, 11, and 13 - PWM
// Digital 11 - 13 Excluded on Pro Micro
// Digital 13 has (not Leo) an LED and resistor attached to it
// Digital 14 - 16, Shared with an SPI bus
// Digital 17 - Doubles and RX LED
// Digital Pins can sink 40ma, recommended 470 ohm resistor
// Analog Inputs: A0-A3, Pro Micro excludes A4, A5
// Analog Inputs: A6-A11 (on digital pins 4, 6, 8, 9, 10, and 12)
// Analog Pins can be used as digital ( refer to them as A0, A1, ...)
//

byte startButtonState;

// macro for detection of rising edge and debouncing
/*the state argument (which must be a variable) records the current and the last 3 reads
  by shifting one bit to the left at each read and bitwise anding with 15 (=0b1111).
  If the value is 7(=0b0111) we have one raising edge followed by 3 consecutive 1's.
  That would qualify as a debounced raising edge*/
#define DRE(signal, state) (state=(state<<1)|(signal&1)&15)==7

// LCD Display-------------------------------------------------------------------------

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad------------------------------------------------------------------------------

const byte ROWS = 4; //four rows (4 Zeilen)
const byte COLS = 4; //four columns (4 Spalten)
//define the cymbols on the buttons of the keypads
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {A11, A10, A9, A8}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {A7, A6, A5, A4}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);
int value = 0;

// Chip select pin for SD card reader-----------------------------------------------------
const int SD_CS = DAC1; // A13 on the mainboard

// General purpose 24V DC Digital Inputs (on the "Digital Input Board")--------------------
const int DI_38_X_REF_SWITCH = 38; // X, Y, Z Reference switches (in parallel)
const int DI_39_Y_REF_SWITCH = 39;
const int DI_40_Z_REF_SWITCH = 40;
const int DI_41_SPINDLE_ROTATION = 41;
const int DI_42_SCREW_READY = 42;  // Screw present in pawl
const int DI_43_SCREW_IN_SEPARATOR = 43;  // Screw present in separator (rosa)
const int DI_44_SEPARATOR_IN_STARTING_POSITION = 44;  // Separator in incoming position (rot)
const int DI_45_SEPARATOR_IN_WORKING_POSITION = 45;  // Separator in output position (grau-rosa)
const int DI_46_PIN = 46;
const int DI_47_PIN = 47;
const int DI_48_PIN = 48;
const int DI_49_PIN = 49;
const int DI_50_PIN = 50;
const int DI_51_PIN = 51;
const int DI_52_EMERGENCY_BUTTON = 52;  // Emergency Stop Button
const int DI_53_START_BUTTON = 53; // Start Button

// General purpose 24V DC Digital Outputs (on the "Digital Output Board")--------------------
const int DO_22_SCREW_SEPARATOR = 22;  // Separator Cylinder
const int DO_23_SCREW_INJECTOR = 23;  // Srewc injector air
const int DO_24_BLOW_OFF_NOZZLE = 24;  // Blow off nozzle in screw feeder
const int DO_25_PIN = 25;  // Res. Output
const int DO_26_SCREW_FEEDER = 26;  // Screw feeder
const int DO_27_VACUUM = 27;  // Vacuum for Screw
const int DO_28_RELEASE = 28; // Screw releasing air
const int DO_29_PIN = 29;
const int DO_30_PIN = 30;
const int DO_31_PIN = 31;
const int DO_32_PIN = 32;
const int DO_33_PIN = 33;
const int DO_34_PIN = 34;
const int DO_35_PIN = 35;
const int DO_36_PIN = 36;
const int DO_37_PIN = 37;

// Stepper Pins (on the "Stepper Board")---------------------------------------------------
// The X stepper pins
const int STEPPER_X_STEP_PIN = 13;
const int STEPPER_X_DIR_PIN = 14;
const int STEPPER_X_CURRENT_PIN = 5;

// The Y stepper pins
const int STEPPER_Y_STEP_PIN = 12;
const int STEPPER_Y_DIR_PIN = 15;
const int STEPPER_Y_CURRENT_PIN = 4;

// The Z stepper pins
const int STEPPER_Z_STEP_PIN = 11;
const int STEPPER_Z_DIR_PIN = 16;
const int STEPPER_Z_CURRENT_PIN = 3;

// The A stepper pins
const int STEPPER_A_STEP_PIN = 10;
const int STEPPER_A_DIR_PIN = 17;
const int STEPPER_A_CURRENT_PIN = 2;

// Define some steppers and the pins they will use
AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STEP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STEP_PIN, STEPPER_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, STEPPER_Z_STEP_PIN, STEPPER_Z_DIR_PIN);
AccelStepper stepperA(AccelStepper::DRIVER, STEPPER_A_STEP_PIN, STEPPER_A_DIR_PIN);

// Axis calibration (stepper motor "steps" per mm)
float stepsPerMmX = 25; // 12.5 for GT2, Z16 (10.2mm pitch diameter) and 1/2 step (400 steps/Rev.)
float stepsPerMmY = 25; // 25 for 1/4 step (800 steps/Rev.)
float stepsPerMmZ = 25;
float stepsPerRoundA = 800;

// Accelerations for normal operation
float accelX = 40000.0; //40000
float accelY = 40000.0; //40000
float accelZ = 40000.0; // Z & A axis acceleration MUST BE IDENTICAL!
float accelA = 40000.0; // about 40000 for good screwing results

// X, Y, Z axis current
int currentX = 100;
int currentY = 100;
int currentZ = 150;

// A axis torque (current) calibration
// PWM signal, 255 = always on, 127 = 50%, filtered with RC-filter for analog voltage signal to TMC2100 driver
int currentA = 150; // 150 = 30Ncm @ 400 RPM (with TMC2100)

// Define Z Axis variables
float zError;
float zRetract;
float zSuck;
float zScrewBegin;
float zScrewingDepth; // how deep is the screw screwed in (+ about 3mm reserve)?

// Define screw pitch
float screwPitch;

// Define screw RPM
int screwRpm;
boolean spindleWasAtFullRpm = false;
boolean spindleStalled = false;
boolean stallDetectionActive = false;

// Define screw feeder variables
boolean screwRequest = false;
boolean screwIsFeeding = false;

// Define max. stepper speeds
float homingSpeed = 1500;
float homingRetractSpeed = 200;
float maxSpeed = 7000;  // 7000
float maxSpeedSucked = 5000;

// The currerntly processed screw in the pattern
unsigned int screwNo = 0;

// How many screws are in the selected pattern?
unsigned int numberOfScrews;

// The currently active program
unsigned int activeProgram = 1;

// The max. number of programs
const unsigned int numberOfPrograms = 20;

// Has the operation mode (Run, Stop etc.) changed?
boolean modeDidChange = false;

// The measured spindle rotation count
volatile long spindleDegreesRaw = 0; // Sensor state changes (impulses)
volatile long spindleRpmRaw = 0;  // "volatile" is required because of the interrupt function!
float spindleDegrees = 0; // in degrees

// Tolerances for screwing angle monitoring
int spindleDegreesMin = 0; // in degrees
int spindleDegreesMax = 0; // in degrees
boolean error = false;
unsigned long rpm = 0; // the measured live spindle RPM

unsigned int operationMode = 1; // operationMode: 0 = Run, 1 = Stop, 2 = Reset, 3 = Program Select 4 = Axis homing

boolean ProgNoInput = false;

// Was the emegrency stop button pressed?
boolean emergencyWasPressed = false;

// Create cylinder position monitoring objects (one for each cylinder)
cylinderPosition separatorMonitoring;

// the currently displayed error message number
int currentMessage = 999;

boolean repeated = false; // Was the screw injection pulse already repeated?

// Timer
SimpleTimer timer;

//
// =======================================================================================================
// TABS FOR SCREW PATTERN SPECIFICATION (Edit them for implementing new types)
// =======================================================================================================
//

#include "patterns.h" // "Screw patterns"
#include "typeVariables.h" // Z axis heihgts, spindle RPM, screw pitch etc.
#include "offsets.h" // Axis offsets for each screw pattern

//
// =======================================================================================================
// READ GENERAL PURPOSE INPUTS
// =======================================================================================================
//

boolean refSwitchX() {
  return digitalRead(DI_38_X_REF_SWITCH);
}

boolean refSwitchY() {
  return digitalRead(DI_39_Y_REF_SWITCH);
}

boolean refSwitchZ() {
  return digitalRead(DI_40_Z_REF_SWITCH);
}

// Pin 41 is an interrupt-Pin

boolean screwReady() {
  return digitalRead(DI_42_SCREW_READY);
}

boolean screwInSeparator() {
  return digitalRead(DI_43_SCREW_IN_SEPARATOR);
}

boolean separatorInStartingPosition() {
  return digitalRead(DI_44_SEPARATOR_IN_STARTING_POSITION);
}

boolean separatorInWorkingPosition() {
  return digitalRead(DI_45_SEPARATOR_IN_WORKING_POSITION);
}

boolean emergencyStop() {
  return digitalRead(DI_52_EMERGENCY_BUTTON);
}


//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup()
{

  // I2C initialization-------------------------------------
  Wire.setClock(100000); // Set 100kHz I2C clock for the PCF8574

  // Serial initialization----------------------------------
  //#ifdef DEBUG
  Serial.begin(115200);
  //#endif

  // Keypad--------------------------------------------------
  customKeypad.setHoldTime(250);       // Default is 1000mS
  customKeypad.setDebounceTime(50);    // Default is 50mS

  // SD card initialization-----------------------------------
  pinMode(SD_CS, OUTPUT);

  Serial.print("Initializing SD card... ");

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
    Serial.println("Card failed or not present");
    // don't do anything more:
  } else {
    Serial.println("Card initialized.");
  }

  // Stepper Pins--------------------------------------------
  pinMode(STEPPER_X_STEP_PIN, OUTPUT);
  pinMode(STEPPER_X_DIR_PIN, OUTPUT);
  pinMode(STEPPER_X_CURRENT_PIN, OUTPUT);
  pinMode(STEPPER_Y_STEP_PIN, OUTPUT);
  pinMode(STEPPER_Y_DIR_PIN, OUTPUT);
  pinMode(STEPPER_Y_CURRENT_PIN, OUTPUT);
  pinMode(STEPPER_Z_STEP_PIN, OUTPUT);
  pinMode(STEPPER_Z_DIR_PIN, OUTPUT);
  pinMode(STEPPER_Z_CURRENT_PIN, OUTPUT);
  pinMode(STEPPER_A_STEP_PIN, OUTPUT);
  pinMode(STEPPER_A_DIR_PIN, OUTPUT);
  pinMode(STEPPER_A_CURRENT_PIN, OUTPUT);

  // General Purpose 24V DC Inputs---------------------------
  pinMode(DI_38_X_REF_SWITCH, INPUT);
  pinMode(DI_39_Y_REF_SWITCH, INPUT);
  pinMode(DI_40_Z_REF_SWITCH, INPUT);
  pinMode(DI_41_SPINDLE_ROTATION, INPUT);
  attachInterrupt(digitalPinToInterrupt(DI_41_SPINDLE_ROTATION), rotationInterrupt, CHANGE);
  pinMode(DI_42_SCREW_READY, INPUT);
  pinMode(DI_43_SCREW_IN_SEPARATOR, INPUT);
  pinMode(DI_44_SEPARATOR_IN_STARTING_POSITION, INPUT);
  pinMode(DI_45_SEPARATOR_IN_WORKING_POSITION, INPUT);
  pinMode(DI_46_PIN, INPUT);
  pinMode(DI_47_PIN, INPUT);
  pinMode(DI_48_PIN, INPUT);
  pinMode(DI_49_PIN, INPUT);
  pinMode(DI_50_PIN, INPUT);
  pinMode(DI_51_PIN, INPUT);
  pinMode(DI_52_EMERGENCY_BUTTON, INPUT);
  pinMode(DI_53_START_BUTTON, INPUT);

  // General Purpose 24V DC Outputs---------------------------
  pinMode(DO_22_SCREW_SEPARATOR, OUTPUT);
  pinMode(DO_23_SCREW_INJECTOR, OUTPUT);
  pinMode(DO_24_BLOW_OFF_NOZZLE, OUTPUT);
  pinMode(DO_25_PIN, OUTPUT);
  pinMode(DO_26_SCREW_FEEDER, OUTPUT);
  pinMode(DO_27_VACUUM, OUTPUT);
  pinMode(DO_28_RELEASE, OUTPUT);
  pinMode(DO_29_PIN, OUTPUT);
  pinMode(DO_30_PIN, OUTPUT);
  pinMode(DO_31_PIN, OUTPUT);
  pinMode(DO_32_PIN, OUTPUT);
  pinMode(DO_33_PIN, OUTPUT);
  pinMode(DO_34_PIN, OUTPUT);
  pinMode(DO_35_PIN, OUTPUT);
  pinMode(DO_36_PIN, OUTPUT);
  pinMode(DO_37_PIN, OUTPUT);

  // Initialize outputs
  digitalWrite(DO_22_SCREW_SEPARATOR, LOW);
  digitalWrite(DO_23_SCREW_INJECTOR, LOW);
  digitalWrite(DO_24_BLOW_OFF_NOZZLE, LOW);
  digitalWrite(DO_25_PIN, LOW);
  digitalWrite(DO_26_SCREW_FEEDER, LOW);
  digitalWrite(DO_27_VACUUM, LOW);
  digitalWrite(DO_28_RELEASE, LOW);


  // Adjust the current for each axis
  analogWrite(STEPPER_X_CURRENT_PIN, currentX);  //255 = 1/2 of 3.3V (Arduino DUE) = 1.65A
  analogWrite(STEPPER_Y_CURRENT_PIN, currentY);
  analogWrite(STEPPER_Z_CURRENT_PIN, currentZ);
  analogWrite(STEPPER_A_CURRENT_PIN, 5); // screwing torque is written in robot FSM!


  // NOTE: the max. stepping frequency (in total for all motors) is 4000 steps/s!!(on a Due 14000)--------------------------

  // Drive Z axis to zero
  stepperZ.setMaxSpeed(homingSpeed); // @4000: 400 Steps per Rev. (1/2 Step Mode) = 10 Rev./s = 600 RPM (571 measured)
  stepperZ.setAcceleration(65000.0);  // fast accelerations are needed for homing!
  stepperZ.moveTo(0);

  // Drive X & Y axis to zero
  stepperX.setMaxSpeed(homingSpeed);
  stepperX.setAcceleration(65000.0);  // 50000 max. for 1/2 steps
  stepperX.moveTo(0);

  stepperY.setMaxSpeed(homingSpeed);
  stepperY.setAcceleration(65000.0);
  stepperY.moveTo(0);

  stepperA.setAcceleration(65000.0);

  // LCD initialization
  delay(1000);
  lcd.init(); // Windows = .begin, OS X = .init !!
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Welcome To");
  lcd.setCursor(1, 1);
  lcd.print("Screw-O-Matic");
  delay(1500);

  homingCycle();  // execute automatic axis homing cycle

  // adjust axis accelerations for normal operation
  stepperX.setAcceleration(accelX);
  stepperY.setAcceleration(accelY);
  stepperZ.setAcceleration(accelZ);
  stepperA.setAcceleration(accelA);

  // Timer, which triggers the spindle RPM calculation every 60ms
  timer.setInterval(60, getSpindleRPM);

  // Start button reading timer
  timer.setInterval(20, readButtonFunction);
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop()
{

  // Buttons
  //readButtonFunction();

  // Call Robot FSM
  fsmRobot();

  // Call screw feeder FSM
  fsmFeeder();

  // Call Feeder Relay ON / OFF function
  feederOnOff();

  // Screw ready for separating?
  screwReadyDetection();

  // Screw injection time limit
  screwInjectionTime();

  // Call Steppers
  stepperX.run();
  stepperY.run();
  stepperZ.run();
  stepperA.run();

  // refresh operation mode on display
  refreshMode();

  // Read Keypad
  readKeypad();

  // Error detection
  errorDetection();

  // Timer
  timer.run();
}

//
// =======================================================================================================
// HOMING CYCLE
// =======================================================================================================
//

int phase;

void homingCycle() {
  if (!emergencyStop()) { // if not emergency stop button pressed

    displayHoming(0); // Show Homing on Display

    // Z axis----------------------------------------------------------
    stepperZ.setMaxSpeed(homingSpeed);
    stepperZ.moveTo(1000 * stepsPerMmZ);  // max. seek distance 1000 mm
    while (!refSwitchZ()) stepperZ.run();   // until switch is ON
    stepperZ.stop(); // Stop as fast as possible: sets new target
    stepperZ.runToPosition();
    displayHoming(1);

    stepperZ.setMaxSpeed(homingRetractSpeed);
    stepperZ.move(-50 * stepsPerMmZ);    // go max. 50mm back
    while (refSwitchZ()) stepperZ.run();// until switch is released
    stepperZ.stop(); // Stop as fast as possible: sets new target
    stepperZ.runToPosition();

    stepperZ.setCurrentPosition(0); // Reset position to zero
    stepperZ.setMaxSpeed(maxSpeed); // switch back to full speed
    displayHoming(2);

    // Y axis----------------------------------------------------------
    stepperY.setMaxSpeed(homingSpeed);
    stepperY.moveTo(-1000 * stepsPerMmY);  // max. seek distance 1000 mm
    while (!refSwitchY()) stepperY.run();   // until switch is ON
    stepperY.stop(); // Stop as fast as possible: sets new target
    stepperY.runToPosition();
    displayHoming(3);

    stepperY.setMaxSpeed(homingRetractSpeed);
    stepperY.move(50 * stepsPerMmY);    // go max. 50mm back
    while (refSwitchY()) stepperY.run();// until switch is released
    stepperY.stop(); // Stop as fast as possible: sets new target
    stepperY.runToPosition();

    stepperY.setCurrentPosition(0); // Reset position to zero
    stepperY.setMaxSpeed(maxSpeed); // switch back to full speed
    displayHoming(4);

    // X axis----------------------------------------------------------
    stepperX.setMaxSpeed(homingSpeed);
    stepperX.moveTo(-1000 * stepsPerMmX);  // max. seek distance 1000 mm
    while (!refSwitchX()) stepperX.run();   // until switch is ON
    stepperX.stop(); // Stop as fast as possible: sets new target
    stepperX.runToPosition();
    displayHoming(5);

    stepperX.setMaxSpeed(homingRetractSpeed);
    stepperX.move(50 * stepsPerMmX);    // go max. 50mm back
    while (refSwitchX()) stepperX.run();// until switch is released
    stepperX.stop(); // Stop as fast as possible: sets new target
    stepperX.runToPosition();

    stepperX.setCurrentPosition(0); // Reset position to zero
    stepperX.setMaxSpeed(maxSpeed); // switch back to full speed!
    displayHoming(6);
    operationMode = 1; // 1 = Stop
    modeDidChange = true;
    //refreshMode();

    // Travel Y axis to front side-------------------------------------------
    stepperY.moveTo(165 * stepsPerMmY);
  }
  // Initialize Type Variables
  typeVariables();
}

//
// =======================================================================================================
// ROBOT FSM (Finite State Machine)
// =======================================================================================================
//

void fsmRobot() {

  static unsigned long ts;  // To store the "current" time in for delays
  static byte state = 1; // This variable is holding the current "step" of the FSM

  if (operationMode == 2 && state > 1) state = 0;  // go to step 0 (Reset), if operation mode 2 (Reset) is active

  switch (state) {

    //----RESET step!!
    case 0:
      digitalWrite(DO_27_VACUUM, LOW);
      digitalWrite(DO_28_RELEASE, LOW);
      stepperX.stop(); // Stop as fast as possible: sets new target
      stepperX.runToPosition();
      stepperY.stop();
      stepperY.runToPosition();
      stepperZ.stop();
      stepperZ.runToPosition();
      stepperZ.setMaxSpeed(homingSpeed);
      stepperZ.moveTo(posZ(zError));  // pull z axis back to error height
      if (stepperZ.currentPosition() == posZ(zError)) {
        stepperX.setMaxSpeed(homingSpeed);
        stepperY.setMaxSpeed(homingSpeed);
        stepperX.moveTo(posX(0)); // move x & y axis to cycle end positions
        stepperY.moveTo(posY(0));
      }
      if (stepperX.currentPosition() == posX(0) && stepperY.currentPosition() == posY(0)) {
        if (!error) { // only, if no error is displayed!
          operationMode = 2; // 2 = Reset
          printPosition();
          modeDidChange = true;
          refreshSpindleDegrees(); // show spindle degrees on LCD
          state = 1;
        }
      }
      break;

    //----Is the operation mode = "Run" (0)?
    case 1:
      if (operationMode == 0) {
        screwNo = 1; // Set the position of the first screw!
        state = 2;
      }
      else state = 1;  // Else loop back
      break;

    //----Feed screw
    case 2:
      if (screwReady() && !screwIsFeeding) { // if no screw is present in the pawl and the prior screw feeding process is finished,
        screwRequest = true;  // request feeder for screw!
        ts = millis();  // Remember the current time
      }
      state = 3;
      break;

    //----Start X & Y Axis positioning
    case 3:
      if (operationMode == 0) {
        stepperX.setMaxSpeed(maxSpeed);
        stepperY.setMaxSpeed(maxSpeed);
        stepperX.moveTo(posX(screwNo));
        stepperY.moveTo(posY(screwNo));
      }

      if (stepperX.currentPosition() == posX(screwNo) && stepperY.currentPosition() == posY(screwNo)) {
        printPosition();
        repeated = false;
        state = 4;
      }
      break;

    //----If screw ready, Z Axis fast downwards to sucking position
    case 4:
      if (!screwReady()) {  // if screw present, go ahead!
        printLCD();
        stepperZ.setMaxSpeed(maxSpeed);
        if (operationMode == 0) stepperZ.moveTo(posZ(zSuck));
        digitalWrite(DO_23_SCREW_INJECTOR, LOW);
        digitalWrite(DO_27_VACUUM, HIGH);

        if (stepperZ.currentPosition() == posZ(zSuck)) {
          ts = millis();  // Remember the current time
          state = 5;
        }
      } else { // if screw not ready
        displayMessage(5); // display "Waiting for screw"
        if (!repeated) {
          digitalWrite(DO_23_SCREW_INJECTOR, HIGH); // give a new injection pulse (if screw did not enter the pawl)
          repeated = true;
        }
      }
      break;

    //----Suck screw, wait for vacuum to build up
    case 5:
      if (millis() > ts + 100) {
        state = 6;
      }
      break;

    //----Z Axis fast downwards to screwing start height
    case 6:
      stepperZ.setMaxSpeed(maxSpeedSucked);
      if (operationMode == 0) stepperZ.moveTo(posZ(zScrewBegin));

      if (stepperZ.currentPosition() == posZ(zScrewBegin)) {
        stepperA.setCurrentPosition(0); // reset A axis position
        state = 7;
      }
      break;

    //----Rotate spindle 180°, in order to "catch" the torx
    case 7:
      analogWrite(STEPPER_A_CURRENT_PIN, currentA); // Spindle torque on
      stepperA.setMaxSpeed(aRpm());
      stepperA.moveTo(400); // 400 = 180°

      if (stepperA.currentPosition() == 400) {
        stepperA.setCurrentPosition(0); // reset A axis position
        spindleDegreesRaw = 0; // reset A axis angle (raw value from sensor)
        spindleRpmRaw = 0;
        spindleWasAtFullRpm = false;
        spindleStalled = false;
        stallDetectionActive = true;
        ts = millis();  // Remember the current time
        state = 8;
      }
      break;

    //----Start screwing, Z Axis slowly downwards (speed in accordance with screw pitch)
    case 8:
      stepperZ.setMaxSpeed(zFeedRate());
      stepperZ.moveTo(posZ( (zScrewBegin - zScrewingDepth) - 1 )); // 1mm deeper, as well as in the aRotationAngle() function
      stepperA.moveTo(aRotationAngle());
      digitalWrite(DO_27_VACUUM, LOW);


      if (stepperZ.currentPosition() == posZ( (zScrewBegin - zScrewingDepth - 1 ) )) { // as soon as Z axis is in position,
        stallDetectionActive = false; // disable spindle stall detection, if Z axis is in position!
        stepperA.setCurrentPosition(0); // stop A axis immediately and reset position!
        analogWrite(STEPPER_A_CURRENT_PIN, 5); // reduce spindle torque
        refreshSpindleDegrees(); // calculate the measured angle and display it!
        if (!screwIsFeeding) {
          screwRequest = true;  // only if prior feeding process is finished (12.9.16 MS), request feeder for screw! (time saving)
        }
        state = 9;
      }
      break;

    //----Wait max. 3s for spindle stall detection signal
    case 9:
      if (millis() > ts + 3000 || spindleStalled) {
        state = 10;
      }
      break;

    //----Screw rotation angle OK and spindle stalled?
    case 10:
      // screwing angle OK?
      if (spindleDegrees < spindleDegreesMin || spindleDegrees > spindleDegreesMax) {
        displayMessage(1); // show screw error on display!
        error = true;
        operationMode = 1; // 1 = Stop
        state = 0;
      }
      // Has the spindle stalled? = screw head did hit part surface = torque OK
      else if (!spindleStalled) {
        displayMessage(3); // show torque error on display!
        error = true;
        operationMode = 1; // 1 = Stop
        state = 0;
      }
      // otherwise proceed with next step
      else {
        spindleStalled = false; // enable rotation interrupt again!
        stallDetectionActive = false; // disable spindle stall detection for screw presence test!
        state = 11;
      }
      break;

    //----Check, if bit is still engaged with screw head (= screw is present)
    case 11:
      stepperA.setMaxSpeed(5000);
      stepperA.moveTo(2400);   // rotate spindle 3 turns with almost no torque
      if (stepperA.currentPosition() == 2400) {
        if (spindleDegreesRaw > 30) { // 12 x 30° = 1 turn
          displayMessage(2); // show error on display! (screw missing)
          error = true;
          operationMode = 1; // 1 = Stop
          state = 0;
        } else {
          state = 12;
          stepperA.setCurrentPosition(0); // reset A axis position
          stepperA.setMaxSpeed(500);
        }
      }
      break;

    //----Rotate screw bit about 1° backwards, in order to loosen it
    case 12:
      digitalWrite(DO_28_RELEASE, HIGH); // Release (blow off) screw
      stepperA.moveTo(-6);   // rotate spindle a bit backwards
      if (stepperA.currentPosition() == -6) {
        state = 13;
      }
      break;

    //----Retract Z Axis
    case 13:
      analogWrite(STEPPER_A_CURRENT_PIN, 0);  // Shut torque off for retracting!
      stepperZ.setMaxSpeed(maxSpeed);
      if (operationMode == 0) stepperZ.moveTo(posZ(zRetract));

      if (stepperZ.currentPosition() == posZ(zRetract)) {
        digitalWrite(DO_28_RELEASE, LOW);
#ifdef DEBUG
        Serial.print("Screw ");
        Serial.println(screwNo); // print the number of the currently mounted screw
#endif
        // screwRequest = true;  // request feeder for screw! (time saving)
        screwNo++; // add 1 screw to counter
        state = 14;
      }
      break;

    //----Final screw?
    case 14:
      if (screwNo >= numberOfScrews) { // Yes
        state = 15; // pattern is finished
      }
      else state = 2;  // Else loop back to step 2 for next screw
      break;

    //----Go to Cycle End Position
    case 15:
      screwNo = 0; // Screw 0 = Cycle End Position!
      if (operationMode == 0) {
        stepperX.moveTo(posX(screwNo));
        stepperY.moveTo(posY(screwNo));
      }

      if (stepperX.currentPosition() == posX(screwNo) && stepperY.currentPosition() == posY(screwNo)) {
#ifdef DEBUG
        Serial.println("Process Finished, Waiting For Start!");
#endif
        operationMode = 1; // 1 = Stop
        printPosition();
        modeDidChange = true;
        //refreshMode();
        state = 1;
      }
      break;
  }
}

//
// =======================================================================================================
// SCREW FEEDER ON / OFF (delay 2s)
// =======================================================================================================
//

unsigned long millisFeederOff = 0;
const long delayFeeder = 2000;  // Delay = 2s
boolean feederState = false;

void feederOnOff() {

  unsigned long currentMillis = millis();

  // Stay on, if:
  if (operationMode == 0 && screwInSeparator()) { // Mode 0 = Run & no screw present in separator
    feederState = true;
    millisFeederOff = currentMillis;
  }

  // Delay timer
  if (currentMillis - millisFeederOff >= delayFeeder) {
    feederState = false;
  }

  // Feeder ON / OFF
  if (feederState) {
    digitalWrite(DO_26_SCREW_FEEDER, HIGH);
    digitalWrite(DO_24_BLOW_OFF_NOZZLE, HIGH);
  } else {
    digitalWrite(DO_26_SCREW_FEEDER, LOW);
    digitalWrite(DO_24_BLOW_OFF_NOZZLE, LOW);
  }
}

//
// =======================================================================================================
// SCREW READY FOR SEPARATING (delay 200ms)
// =======================================================================================================
//

unsigned long millisScrewReady = 0;
const long delayScrewReady = 200;  // Delay = 0.2s
boolean screwReadyForSeparating = false;

void screwReadyDetection() {

  unsigned long currentMillis = millis();

  // Not ready, if:
  if (operationMode == 0 && screwInSeparator()) { // Mode 0 = Run & no screw present in separator
    screwReadyForSeparating = false;
    millisScrewReady = currentMillis;
  }

  // Delay timer
  if (currentMillis - millisScrewReady >= delayScrewReady) {
    screwReadyForSeparating = true;
  }
}

//
// =======================================================================================================
// MAX. SCREW INBJECTION DURATION TIMER
// =======================================================================================================
//

unsigned long millisScrewInPawl = 0;
const long delayScrewInPawl = 500;  // Delay = 0.5s

void screwInjectionTime() {

  unsigned long currentMillis = millis();

  // Reset delay timer, if screw injector air is off
  if (!digitalRead(DO_23_SCREW_INJECTOR)) {
    millisScrewInPawl = currentMillis;
  }

  // Switch air off, if on for > 0.5s
  if (currentMillis - millisScrewInPawl >= delayScrewInPawl) {
    digitalWrite(DO_23_SCREW_INJECTOR, LOW);
  }
}

//
// =======================================================================================================
// SCREW FEEDER FSM (Finite State Machine)
// =======================================================================================================
//

void fsmFeeder() {



  static unsigned long tsF;  // To store the "current" time in for delays
  static byte stateFeeder = 1; // This variable is holding the current "step" of the FSM

  if (operationMode == 2 && stateFeeder > 1) stateFeeder = 0;  // go to step 0 (Reset), if operation mode 2 (Reset) is active


#ifdef DEBUG
  // Serial.print("Feeder state: ");
  // Serial.println(stateFeeder);
#endif

  switch (stateFeeder) {

    //----RESET step!!
    case 0:
      digitalWrite(DO_22_SCREW_SEPARATOR, LOW);
      digitalWrite(DO_23_SCREW_INJECTOR, LOW);
      digitalWrite(DO_24_BLOW_OFF_NOZZLE, LOW);
      digitalWrite(DO_25_PIN, LOW);
      digitalWrite(DO_26_SCREW_FEEDER, LOW);
      digitalWrite(DO_27_VACUUM, LOW);

      screwRequest = false;
      screwIsFeeding = false;

      if (separatorInStartingPosition()) {
        stateFeeder = 1;
      }
      break;

    //----Is the operation mode = "Run" (0)?
    case 1:
      if (operationMode == 0) {
        stateFeeder = 2;
      }
      else stateFeeder = 1;  // Else loop back
      break;

    //----Separate screw
    case 2:
      if (!screwInSeparator() && screwRequest && screwReadyForSeparating) {  // screw is present, request for screw active and screw ready for separating (delayed)!
        digitalWrite(DO_22_SCREW_SEPARATOR, HIGH);
        screwRequest = false;
        screwIsFeeding = true;
      }

      if (separatorInWorkingPosition()) {
        stateFeeder = 3;
        tsF = millis();  // Remember the current time
      }
      break;

    //----Screew needs some time to leave the separator!
    case 3:
      if (millis() > tsF + 300) {
        stateFeeder = 4;
      }
      break;

    //----Separator back
    case 4:
      digitalWrite(DO_22_SCREW_SEPARATOR, LOW);

      if (separatorInStartingPosition()) {  // if separator is back in working position...
        stateFeeder = 5;
      }
      break;

    //----Inject screw
    case 5:

      digitalWrite(DO_23_SCREW_INJECTOR, HIGH);
      tsF = millis();  // Remember the current time
      stateFeeder = 6;
      break;

    //----Injection impulse duration
    case 6:
      if (millis() > tsF + 150) {
        stateFeeder = 7;
      }
      break;

    //----Injection air off, separator back to start position
    case 7:
      if (!repeated) {
        digitalWrite(DO_23_SCREW_INJECTOR, LOW);
      }
      digitalWrite(DO_22_SCREW_SEPARATOR, LOW);

      if (separatorInStartingPosition()) {
        tsF = millis();  // Remember the current time
        stateFeeder = 8;
      }
      break;

    //----Allow the screw to arrive in the screwing head before looping back!
    case 8:
      if (millis() > tsF + 1000) {
        screwIsFeeding = false;
        stateFeeder = 1;
      }
      break;
  }
}

//
// =======================================================================================================
// FUNCTIONS FOR READING AXIS POSITIONS FROM PROGRAM MEMORY (BECAUSE THERE IS NOT ENOUGH RAM AVAILABLE)
// =======================================================================================================
// Max. 20 programs @ the moment!

// X Axis
float posX(int screw) {
  float x;
  switch (activeProgram) {
    case 1:
      x = pgm_read_float(&pos1[screw][0]);
      break;
    case 2:
      x = pgm_read_float(&pos2[screw][0]);
      break;
    case 3:
      x = pgm_read_float(&pos3[screw][0]);
      break;
    case 4:
      x = pgm_read_float(&pos4[screw][0]);
      break;
    case 5:
      x = pgm_read_float(&pos5[screw][0]);
      break;
    case 6:
      x = pgm_read_float(&pos6[screw][0]);
      break;
    case 7:
      x = pgm_read_float(&pos7[screw][0]);
      break;
    case 8:
      x = pgm_read_float(&pos8[screw][0]);
      break;
    case 9:
      x = pgm_read_float(&pos9[screw][0]);
      break;
    case 10:
      x = pgm_read_float(&pos10[screw][0]);
      break;
    case 11:
      x = pgm_read_float(&pos11[screw][0]);
      break;
    case 12:
      x = pgm_read_float(&pos12[screw][0]);
      break;
    case 13:
      x = pgm_read_float(&pos13[screw][0]);
      break;
    case 14:
      x = pgm_read_float(&pos14[screw][0]);
      break;
    case 15:
      x = pgm_read_float(&pos15[screw][0]);
      break;
    case 16:
      x = pgm_read_float(&pos16[screw][0]);
      break;
    case 17:
      x = pgm_read_float(&pos17[screw][0]);
      break;
    case 18:
      x = pgm_read_float(&pos18[screw][0]);
      break;
    case 19:
      x = pgm_read_float(&pos19[screw][0]);
      break;
    case 20:
      x = pgm_read_float(&pos20[screw][0]);
      break;
  }
  x = (x + offset[(activeProgram - 1)][0]) * stepsPerMmX; // offset 0 for program 1 etc. 0 = X axis
  return (int) x; // only if Start is active!
}

// Y Axis
float posY(int screw) {
  float y;
  switch (activeProgram) {
    case 1:
      y = pgm_read_float(&pos1[screw][1]);
      break;
    case 2:
      y = pgm_read_float(&pos2[screw][1]);
      break;
    case 3:
      y = pgm_read_float(&pos3[screw][1]);
      break;
    case 4:
      y = pgm_read_float(&pos4[screw][1]);
      break;
    case 5:
      y = pgm_read_float(&pos5[screw][1]);
      break;
    case 6:
      y = pgm_read_float(&pos6[screw][1]);
      break;
    case 7:
      y = pgm_read_float(&pos7[screw][1]);
      break;
    case 8:
      y = pgm_read_float(&pos8[screw][1]);
      break;
    case 9:
      y = pgm_read_float(&pos9[screw][1]);
      break;
    case 10:
      y = pgm_read_float(&pos10[screw][1]);
      break;
    case 11:
      y = pgm_read_float(&pos11[screw][1]);
      break;
    case 12:
      y = pgm_read_float(&pos12[screw][1]);
      break;
    case 13:
      y = pgm_read_float(&pos13[screw][1]);
      break;
    case 14:
      y = pgm_read_float(&pos14[screw][1]);
      break;
    case 15:
      y = pgm_read_float(&pos15[screw][1]);
      break;
    case 16:
      y = pgm_read_float(&pos16[screw][1]);
      break;
    case 17:
      y = pgm_read_float(&pos17[screw][1]);
      break;
    case 18:
      y = pgm_read_float(&pos18[screw][1]);
      break;
    case 19:
      y = pgm_read_float(&pos19[screw][1]);
      break;
    case 20:
      y = pgm_read_float(&pos20[screw][1]);
      break;

  }
  y = (y + offset[(activeProgram - 1)][1]) * stepsPerMmY ; // offset 0 for program 1 etc. 1 = Y axis
  return (int) y; // only if Start is active!
}

// Z Axis
long posZ(float height) {
  height = (height + offset[(activeProgram - 1)][2]) * stepsPerMmZ ; // offset 0 for program 1 etc. 2 = Z axis
  return (long) height; // only if Start is active!
}

//
// =======================================================================================================
// SERIAL PRINT X & Y POSITIONS
// =======================================================================================================
//

void printPosition() {
#ifdef DEBUG
  Serial.print("X ");
  Serial.print((stepperX.currentPosition() / stepsPerMmX) - offset[(activeProgram - 1)][0]);
  Serial.print(", Y ");
  Serial.println((stepperY.currentPosition() / stepsPerMmY) - offset[(activeProgram - 1)][1]);
#endif
  refreshXY();
}

//
// =======================================================================================================
// LCD & DISPLAY PRINT FUNCTIONS
// =======================================================================================================
//

// NOTE: the display communication is slow! So we never clear and re-write the whole display content
// during normal operation. Only the relevant content is refreshed as needed!

// NEVER write to the LCD during axis movement!
// Otherwise the axis position could be lost because of some blocking routines inside the LCD library!

// this function refreshes the complete main display content
void printLCD() {
  if (stepperX.isRunning() == false && stepperY.isRunning() == false && stepperZ.isRunning() == false ) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pr.");

    lcd.setCursor(0, 1);
    lcd.print("X");

    lcd.setCursor(8, 1);
    lcd.print("Y");
    refreshXY();
    refreshProgNo();
    modeDidChange = true;
    currentMessage = 999;
  }
}

// refresh x and y positions
void refreshXY() {
  lcd.setCursor(1, 1);
  lcd.print("       "); // clear old x position
  lcd.setCursor(1, 1);
  lcd.print((stepperX.currentPosition() / stepsPerMmX) - offset[(activeProgram - 1)][0]);

  lcd.setCursor(9, 1);
  lcd.print("       "); // clear old y position
  lcd.setCursor(9, 1);
  lcd.print((stepperY.currentPosition() / stepsPerMmY) - offset[(activeProgram - 1)][1]);
}

// refresh rotation count
void refreshSpindleDegrees() {
  lcd.setCursor(12, 0);
  lcd.print("   "); // clear old rotation count
  lcd.setCursor(12, 0);
  spindleDegrees = spindleDegreesRaw * 30; // 1 rotation count = 30 degrees (for "CHANGE" interrupt with 6 teeth)
  spindleDegreesRaw = 0;
  lcd.print(spindleDegrees); //spindleDegreesRaw
}

// program number entry screen
void programNumberEntryScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Prog. Nummer:");
  lcd.setCursor(0, 1);
  lcd.print("# =Enter * =Esc.");
}

// refresh program number
void refreshProgNo() {
  lcd.setCursor(3, 0);
  lcd.print("  "); // clear old Prog No.
  lcd.setCursor(3, 0);
  lcd.print(activeProgram);
}

// refresh mode
void refreshMode() {
  if (modeDidChange && stepperX.isRunning() == false && stepperY.isRunning() == false && stepperZ.isRunning() == false ) {
    lcd.setCursor(6, 0);
    if (operationMode == 0) lcd.print("RUN "); // all same length!
    if (operationMode == 1) lcd.print("STOP");
    if (operationMode == 2) lcd.print("RES.");
    if (operationMode == 3) lcd.print("P.S.");
    if (operationMode == 4) lcd.print("HOM.");
    modeDidChange = false;
  }
}

// Refresh numerical input value (Program number)
void refreshInputValue() {
  lcd.setCursor(14, 0);
  lcd.print("      "); // clear old value
  lcd.setCursor(14, 0);
  lcd.print(value); //show new value
}

// homing
void displayHoming(int phase) {
  lcd.setCursor(8, 0);
  if (phase == 0) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Referenzfahrt");
    lcd.setCursor(1, 1);
    lcd.print(".");
  }
  if (phase == 1) {
    lcd.setCursor(1, 1);
    lcd.print("..");
  }
  if (phase == 2) {
    lcd.setCursor(1, 1);
    lcd.print("...");
  }
  if (phase == 3) {
    lcd.setCursor(1, 1);
    lcd.print("....");
  }
  if (phase == 4) {
    lcd.setCursor(1, 1);
    lcd.print(".....");
  }
  if (phase == 5) {
    lcd.setCursor(1, 1);
    lcd.print("......");
  }
  if (phase == 6) {
    lcd.setCursor(1, 1);
    lcd.print(".......OK");
    delay(1000);
    printLCD();
  }
}

// Error messages

void displayMessage(int message) {
  if (message == 0 && currentMessage != message) { // 0 = Emergency Stop
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Emergency Stop");
    lcd.setCursor(3, 1);
    lcd.print("Not - Stop");
    currentMessage = message;
  }
  if (message == 1 && currentMessage != message) { // 1 = screw tightening error (angle)
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Schraubtiefe!");
    lcd.setCursor(1, 1);
    lcd.print("Winkel:");
    lcd.setCursor(9, 1);
    lcd.print(spindleDegrees);
    currentMessage = message;
  }
  if (message == 2 && currentMessage != message) { // 2 = screw missing
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Schraube");
    lcd.setCursor(1, 1);
    lcd.print("fehlt!");
    currentMessage = message;
  }
  if (message == 3 && currentMessage != message) { // 2 = screw tightening error (torque too small, spindle not stalled)
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Drehmoment nicht");
    lcd.setCursor(0, 1);
    lcd.print("erreicht!");
    currentMessage = message;
  }
  if (message == 4 && currentMessage != message) { // 4 = screw separator error
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Schraubenverein-");
    lcd.setCursor(0, 1);
    lcd.print("zeler Stoerung!");
    currentMessage = message;
  }
  if (message == 5 && currentMessage != message) { // 5 = waiting for screw...
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Warte auf");
    lcd.setCursor(0, 1);
    lcd.print("Schraube...");
    currentMessage = message;
  }
}

//
// =======================================================================================================
// COMPUTE SCREW MOTOR ROTATION ANGLE
// =======================================================================================================
//
long aRotationAngle() {
  return (zScrewingDepth + 1) * stepsPerRoundA / screwPitch ; // add 1mm depth!
}

//
// =======================================================================================================
// COMPUTE SCREW MOTOR RPM
// =======================================================================================================
//
int aRpm() {
  return screwRpm * stepsPerRoundA / 60;
}

//
// =======================================================================================================
// COMPUTE Z AXIS FEED RATE (mm/s)
// =======================================================================================================
//
float zFeedRate() { // returns steps per s for the z axis feed motor
  return (screwRpm * screwPitch) / 60 * stepsPerMmZ ;
}


//
// =======================================================================================================
// SPINDLE RPM CALCULATION (every 60ms)
// =======================================================================================================
//

static long lastRpm;
static long deltaRpm; // The RPM drop since last measurement
static unsigned long spindleTimeStart;

// This function calculates the current spindle RPM
void getSpindleRPM() {
  lastRpm = rpm;
  rpm = spindleRpmRaw * 6000 / ( millis() - spindleTimeStart );  // 6000 for "CHANGE" interrupt with 6 teeth
  spindleRpmRaw = 0;
  spindleTimeStart = millis();
  deltaRpm = lastRpm - rpm;
#ifdef DEBUG
  Serial.print(rpm);
  Serial.print(" RPM   ");
  Serial.print(deltaRpm);
  Serial.println(" RPM delta");
#endif
  spindleStallDetection();
}

//
// =======================================================================================================
// SPINDLE STALL DETECTION
// =======================================================================================================
//

void spindleStallDetection() {
  if (rpm >= (screwRpm - 50)) {
    spindleWasAtFullRpm = true;
  }
  if (stallDetectionActive && spindleWasAtFullRpm  && !spindleStalled) {
    if ((deltaRpm >= 300) || (rpm <= 150) || rpm >= (screwRpm + 150)) { // if the RPM drop per 60ms is > 300 = screw head arrived on part surface!
      spindleStalled = true;
      stepperA.setCurrentPosition(0); // stop A axis immediately and reset position!
      analogWrite(STEPPER_A_CURRENT_PIN, 5); // Reduce spindle torque!
#ifdef DEBUG
      Serial.println("Spindle stalled!");
#endif
    }
  }
}

//
// =======================================================================================================
// ROTATION SENSOR INTERRUPT
// =======================================================================================================
//
void rotationInterrupt() {
  spindleRpmRaw ++; // for RPM detection
  if (!spindleStalled) {
    spindleDegreesRaw ++; // for spindle angle detection (only, if not stalled)
  }
}

//
// =======================================================================================================
// BUTTONS
// =======================================================================================================
//

// operationMode: 0 = Run, 1 = Stop, 2 = Reset, 3 = Program Select 4 = Axis homing

void readButtonFunction() {

  // Start button
  if (DRE(digitalRead(DI_53_START_BUTTON), startButtonState)) {
    if (operationMode > 0 && !error) { // only, if no screwing error is displayed!
      delay(20);  // NOTE! The display refresh is delayed a bit, because of possible interferences on the I2C bus,
      // caused from capacitive coupling effects in the cable, during pressing the 24V start button...
      operationMode = 0; // 0 = Start
      modeDidChange = true;
      //refreshMode();
    }
  }
}

//
// =======================================================================================================
// ERROR DETECTION
// =======================================================================================================
//
void errorDetection() {

  // Emergency stop button is pressed---------------------------------------------------------
  if (emergencyStop()) {
    if (!emergencyWasPressed) { // only once!
      delay(20);  // NOTE! The display refresh is delayed a bit, because of possible interferences on the I2C bus,
      // caused from capacitive coupling effects in the cable, during pressing the 24V emegrency stop button...
      displayMessage(0);
#ifdef DEBUG
      Serial.print("Emergency Stop");
#endif
      emergencyWasPressed = true;
    }
  }

  // Emergency stop button was pulled out------------------------------------------------------
  if (!emergencyStop() && emergencyWasPressed) {
    homingCycle(); // execute homing cycle, because the position is always lost during emergency stop!
    emergencyWasPressed = false;
  }

  // Cylinder position error detection (do not trigger message, during axis movement)-----------
  if (stepperX.isRunning() == false && stepperY.isRunning() == false && stepperZ.isRunning() == false ) {

    // Screw separator
    if (separatorMonitoring.monitoringRlAl(DO_22_SCREW_SEPARATOR, DI_44_SEPARATOR_IN_STARTING_POSITION, DI_45_SEPARATOR_IN_WORKING_POSITION)) {
      if (!error) {
        operationMode = 1; // 1 = Stop
        displayMessage(4);
        error = true;
#ifdef DEBUG
        Serial.println("Separator Error");
#endif
      }
    }
  }
}

//
// =======================================================================================================
// READ NUMERIC KEYPAD
// =======================================================================================================
//

void readKeypad() {
  char key = customKeypad.getKey();
  switch (key) {
    case NO_KEY:
      // No key read. Do nothing-----------------------
      break;

    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      if (ProgNoInput) {
        // Digits from 0 to 9
        // Don't forget we deal with ASCII characters and not numbers
        value = value * 10 + key - '0';
        if (value > 99) value = 0;
        refreshInputValue(); // Show the typed value on the LCD
      }
      break;

    case '#':
      // Enter key-------------------------------------
      if (value <= numberOfPrograms) {
        activeProgram = value;
      } else {
        activeProgram = numberOfPrograms;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Programmnummer");
        lcd.setCursor(0, 1);
        lcd.print("ungueltig!");
        delay(2000);
      }
      printLCD();
      ProgNoInput = false;
      typeVariables(); // Refresh the type specific Z axis variables!! IMPORTANT!!
      // Reset value. The last number is done
      value = 0;
      break;

    case '*':
      // Esc key----------------------------------------
      // Reset value. The last number is done
      value = 0;
      refreshInputValue(); // Show the value on the LCD
      break;

    case 'A':
      // Stop key--------------------------------------
      operationMode = 1;  // 1 = Stop
      screwRequest = false;
      modeDidChange = true;
      error = false;
      printLCD();
      break;

    case 'B':
      // Reset key--------------------------------------
      operationMode = 2;  // 2 = Reset
      screwRequest = false;
      modeDidChange = true;
      error = false;
      printLCD();
      break;

    case 'C':
      // Menu key----------------------------------------
      if (operationMode > 0) { // > 0 = Machine not running
        ProgNoInput = true;
        programNumberEntryScreen(); // Show program number entry screen
      }
      break;

    case 'D':
      // Next key (Homing)------------------------------
      homingCycle();
      break;

    default:
      // All other keys--------------------------------
      // Reset value. The last number is done
      value = 0;
  }
}

//
// =======================================================================================================
// READ TEXT FILE FROM SD CARD
// =======================================================================================================
//

void readSdText() {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File sdText = SD.open("datalog.txt");

  // if the file is available, read it:
  if (sdText) {
    while (sdText.available()) {
      Serial.write(sdText.read());
    }
    sdText.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

//
// =======================================================================================================
// READ CSV FILE FROM SD CARD INTO AN ARRAY
// =======================================================================================================
//

String inString = "";
float myArray[512];

void readSdCsv() {

  File sdCsv = SD.open("test.csv");
  int i = 0;
  // if the file is available, read it:
  if (sdCsv) {
    while (sdCsv.available() > 0) {
      int inChar = sdCsv.read();
      if (inChar != ';') {
        inString += (char)inChar;
      }
      else {
        myArray[i] = inString.toFloat();
        i++;
        inString = "";
      }
    }
    Serial.println("test.csv successfully read");
    sdCsv.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening test.csv");
  }
}


