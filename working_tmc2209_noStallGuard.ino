#include <TMCStepper.h>
#include <AccelStepper.h>

// --- Pin Definitions ---
//#define DIAG_PIN         2  // Connected to DIAG pin on TMC2209
#define STEP_PIN         4  
#define DIR_PIN          5  
#define EN_PIN           6  
#define SW_TX            9  // Connected to Pin 4 (PDN_UART) on TMC2209

// --- Driver & Hardware Setup ---
#define R_SENSE       0.11f 
#define DRIVER_ADDRESS 0b00 
//#define STALL_VALUE 20     // Higher = more sensitive (0-255)

// --- Mechanical Specs for Limits ---
constexpr float MOTOR_STEPS_PER_REV = 200.0;
constexpr float GEAR_RATIO = 20.0;      // 20:1 Planetary Gearbox
constexpr float MICROSTEPS = 4.0;       // TMC2209 microstepping setting
constexpr float LEAD_SCREW_PITCH = 1.0; // TR8x1 lead screw (1mm advance per rev)
constexpr float MAX_MOTOR_RPM = 600.0;  // Maximum safe RPM for setup

// --- Calculated Hardware Limits ---
constexpr float STEPS_PER_MM = (MOTOR_STEPS_PER_REV * GEAR_RATIO * MICROSTEPS) / LEAD_SCREW_PITCH; // 16,000 steps/mm
constexpr float MAX_MOTOR_REV_PER_SEC = MAX_MOTOR_RPM / 60.0; 
constexpr float MAX_STEPS_PER_SEC = MAX_MOTOR_REV_PER_SEC * MOTOR_STEPS_PER_REV * MICROSTEPS; // 8000 steps/sec at 600 RPM
constexpr float MAX_ALLOWED_SPEED_MM_S = MAX_STEPS_PER_SEC / STEPS_PER_MM; // Physical limit in mm/s (0.500 mm/s)
constexpr float MIN_ALLOWED_SPEED_MM_S = 1.0 / STEPS_PER_MM;               // 0.0000625 mm/s (1 step/sec lower bound)

// --- Serial Input & Processing Variables ---
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

// User inputs
float mm_per_mL = 0.0;     
float input_flowrate_uL_min = 0.0; 
float input_volume_uL = 0.0;  

// Calculated state variables
float flowrate_mL_s = 0.0;  // Standardized rate in mL/s
float volume_mL = 0.0;      // Standardized volume in mL
float feedrate = 0.0;       // Axis speed in mm/s
float pumpLength = 0.0;     // Axis distance in mm
bool commandValid = false;  // Safety flag to block execution
bool isPumping = false;     // Tracks active execution cycle

// Initialize Objects
TMC2209Stepper driver(-1, SW_TX, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// volatile bool stallDetected = false;

// void handleStall() {
//   stallDetected = true;
// }

void printSyringeBoundaryLine(const __FlashStringHelper* label, float scale) {
  float min_uL_m = (MIN_ALLOWED_SPEED_MM_S / scale) * 60000.0;
  float max_uL_m = (MAX_ALLOWED_SPEED_MM_S / scale) * 60000.0;

  Serial.print(label);
  Serial.print(F(" | "));
  
  if (scale < 10.0) Serial.print(F("   "));
  else if (scale < 100.0) Serial.print(F("  "));
  Serial.print(scale, 1);
  Serial.print(F(" mm/mL | "));
  
  if (min_uL_m < 0.01) Serial.print(min_uL_m, 5);
  else Serial.print(min_uL_m, 3);
  
  Serial.print(F(" uL/min | "));
  if (max_uL_m < 100.0) Serial.print(F("  "));
  else if (max_uL_m < 1000.0) Serial.print(F(" "));
  Serial.print(max_uL_m, 1);
  Serial.println(F(" uL/min"));
}

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  // pinMode(DIAG_PIN, INPUT);
  digitalWrite(EN_PIN, LOW); // Active LOW enable

  // Attach Hardware Interrupt to D2 for StallGuard
  // attachInterrupt(digitalPinToInterrupt(DIAG_PIN), handleStall, RISING);

  // --- Configure TMC2209 via UART ---
  driver.begin();
  delay(50);
  driver.toff(5);                   
  driver.rms_current(800);          // Set motor current (mA)
  driver.microsteps(MICROSTEPS);            
  driver.pwm_autoscale(true);       // StealthChop autoscale
  driver.en_spreadCycle(false);     // FALSE = StealthChop2

  // driver.TCOOLTHRS(0xFFFFF);        
  // driver.SGTHRS(STALL_VALUE);       // Stall Sensitivity Threshold
  
  delay(50);
  driver.push();

  // --- Configure AccelStepper ---
  stepper.setAcceleration(5000);    // Steps/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true); // true = Active LOW enable
  stepper.disableOutputs();

  Serial.println(F("ready"));
  Serial.println(F("\n--- TYPICAL SYRINGE FLOW RATE BOUNDARIES ---"));
  Serial.println(F("Syringe Size   | Scale Scale | Min Flow Rate | Max Flow Rate"));
  Serial.println(F("-------------------------------------------------------------"));
  
  printSyringeBoundaryLine(F("100 uL Hamilton"), 597.3);
  printSyringeBoundaryLine(F("1 mL Standard  "), 55.7);
  printSyringeBoundaryLine(F("3 mL Standard  "), 17.0);
  printSyringeBoundaryLine(F("5 mL Standard  "), 8.7);
  printSyringeBoundaryLine(F("10 mL Standard "), 6.0);

  Serial.println(F("-------------------------------------------------------------"));
  Serial.println(F("Format:  <length_of_1mL_mm, flowrate_uL_min, volume_uL>"));
  Serial.println(F("Example: <17.0, 1000, 500>\n"));
}

void loop() {
  recvWithStartEndMarkers();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    calculateAndVerify();
    showParsedData();

    if (commandValid) {
      runStepper();
    } else {
      Serial.println(F("⛔ ABORTED: Command rejected due to safety or threshold error. Motor idle.\n"));
    }

    newData = false;
  }

  // 1. Direct Stall Detection Interrupt Handling
  // if (stallDetected) {
  //   Serial.println(F("\n⚠️ STALL DETECTED! Stopping motor execution immediately..."));
  //   stallDetected = false;
  //   isPumping = false;
  //   stepper.stop();
  //   stepper.setCurrentPosition(stepper.currentPosition()); 
  //   stepper.disableOutputs(); // Turn off torque
  // }
  // 2. Continuous Motion Loop
  // else {
    stepper.run(); 

    // Auto-disable driver outputs once movement is completed
    if (isPumping && stepper.distanceToGo() == 0) {
      isPumping = false;
      stepper.disableOutputs(); 
      Serial.println(F("✅ Pumping cycle complete. Motor driver outputs disabled."));
    }
  // }
}

void runStepper() {
  float targetSteps = pumpLength * STEPS_PER_MM;
  float speedStepsPerSec = feedrate * STEPS_PER_MM;

  if (targetSteps < 0) {
    speedStepsPerSec = -speedStepsPerSec;
  }

  stepper.setMaxSpeed(abs(speedStepsPerSec));
  stepper.move(targetSteps);
  
  // stallDetected = false; // Clear stale stall signals
  isPumping = true;      // Flag as actively pumping
  stepper.enableOutputs();
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  char* strtokIndx;

  strtokIndx = strtok(tempChars, ",");
  mm_per_mL = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  input_flowrate_uL_min = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  input_volume_uL = atof(strtokIndx);

  // Internal standardized conversions
  flowrate_mL_s = input_flowrate_uL_min / 60000.0;
  volume_mL = input_volume_uL / 1000.0;
}

void calculateAndVerify() {
  if (mm_per_mL <= 0) {
    commandValid = false;
    Serial.println(F("\n❌ ERROR: Syringe scale (mm/mL) must be greater than zero!"));
    return;
  }

  // Calculate physical feedrate (mm/s)
  feedrate = flowrate_mL_s * mm_per_mL;
  pumpLength = volume_mL * mm_per_mL;

  // Calculate exact uL/min limits for the entered syringe scale
  float min_uL_min = (MIN_ALLOWED_SPEED_MM_S / mm_per_mL) * 60000.0;
  float max_uL_min = (MAX_ALLOWED_SPEED_MM_S / mm_per_mL) * 60000.0;

  if (feedrate > MAX_ALLOWED_SPEED_MM_S) {
    commandValid = false;
    Serial.print(F("\n❌ ERROR: Requested flow rate exceeds maximum limit for scale "));
    Serial.print(mm_per_mL);
    Serial.print(F(" (Max allowed: "));
    Serial.print(max_uL_min, 1);
    Serial.println(F(" uL/min)!"));
  } 
  else if (feedrate < MIN_ALLOWED_SPEED_MM_S) {
    commandValid = false;
    Serial.print(F("\n❌ ERROR: Requested flow rate is below minimum limit for scale "));
    Serial.print(mm_per_mL);
    Serial.print(F(" (Min allowed: "));
    Serial.print(min_uL_min, 6);
    Serial.println(F(" uL/min)!"));
  } 
  else if (volume_mL <= 0) {
    commandValid = false;
    Serial.println(F("\n❌ ERROR: Target volume must be greater than zero!"));
  } 
  else {
    commandValid = true;
  }
}

void showParsedData() {
  float min_uL_min = (MIN_ALLOWED_SPEED_MM_S / mm_per_mL) * 60000.0;
  float max_uL_min = (MAX_ALLOWED_SPEED_MM_S / mm_per_mL) * 60000.0;

  Serial.println(F("\n--- Received Command Summary ---"));
  Serial.print(F("Syringe scale: ")); Serial.print(mm_per_mL); Serial.println(F(" mm/mL"));
  Serial.print(F("Allowed Range for this Syringe: ")); 
  Serial.print(min_uL_min, 6); Serial.print(F(" to ")); 
  Serial.print(max_uL_min, 1); Serial.println(F(" uL/min"));

  Serial.print(F("Requested rate: ")); Serial.print(input_flowrate_uL_min, 4); Serial.println(F(" uL/min"));
  Serial.print(F("Requested volume: ")); Serial.print(input_volume_uL, 4); Serial.println(F(" uL"));

  if (commandValid) {
    Serial.print(F("Calculated Plunger Speed: ")); Serial.print(feedrate, 8); Serial.println(F(" mm/s"));
    Serial.print(F("Plunger Travel Distance: ")); Serial.print(pumpLength, 4); Serial.println(F(" mm"));
    Serial.println(F("Status: VALID - Motor starting..."));
  } else {
    Serial.println(F("Status: INVALID - Execution rejected."));
  }
  Serial.println(F("--------------------------------\n"));
}