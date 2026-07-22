// cleaned up with gemini 

//syringe collision
//mm definition between ticks

// figure out if step dir en are needed, or js spi
// how many wires coming out of connector
// make separate syringe pump (bigger motor, shifted syringe holder)
// calc feedrates from commercial pump


/*
common gnd (5+12/24)
+5
+12/24
mosi
miso
sck
cs
step
en

dir?

*/

// 0.0014 uL /hr
/*
.1mm 
*/


#include <SPI.h>
#include <TMC2130Stepper.h>
#include <AccelStepper.h>

#define EN_PIN 4 
#define DIR_PIN 7 
#define STEP_PIN 8 
#define CS_PIN 10

#define STALL_VALUE 40
bool isStalled = false; // Flag to track stall status
uint32_t moveStartTime = 0; // Tracks when the current move started

constexpr uint32_t steps_per_mm = 200 * 8 / 8;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

// input values from master
float mm_per_mL = 0.0; // (mm/mL)
float flowrate = 0.0; // (mL/s)
float requested_mL = 0.0; // (mL)

// calculated values on slave
float feedrate = 0.0; // (mm/s)
float pumpLength = 0.0; // (mm)

boolean newData = false;

TMC2130Stepper driver = TMC2130Stepper(CS_PIN);
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  SPI.begin();
  Serial.begin(115200);
  while (!Serial);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MISO, INPUT_PULLUP); // Helps stabilize the SPI read line

  driver.begin();             
  driver.rms_current(1000);    // Set stepper current to 600mA
  driver.microsteps(8);

  // --- SPI Hardware Communication Diagnostic ---
  uint8_t conn_status = driver.test_connection();
  Serial.print("SPI Connection Test Status Code: ");
  Serial.println(conn_status);
  if (conn_status == 0) {
    Serial.println("-> SUCCESS: TMC2130 SPI Read/Write verified!\n");
  } else {
    Serial.println("-> ERROR: SPI Read failed! Check SDO/MISO wiring or power.\n");
  }

  // --- Force Raw Registers for StallGuard (SpreadCycle) ---
  driver.stealthChop(0);      // Explicitly disables StealthChop -> enables SpreadCycle
  driver.toff(4);             // Enable driver chopper (TOFF > 0 required)
  driver.blank_time(24);

  driver.TCOOLTHRS(0xFFFFF);  // Directly sets raw TCOOLTHRS register to max
  driver.THIGH(0);            // Ensures high-speed mode is disabled
  driver.sg_stall_value(STALL_VALUE); // Sets sensitivity threshold

  // --- AccelStepper Setup ---
  stepper.setMaxSpeed(50 * steps_per_mm); 
  stepper.setAcceleration(500 * steps_per_mm); 
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  Serial.println("System Ready!");
  Serial.println("Input data as: <length of 1mL(mm),flowrate(mL/s),target volume(mL)>\n");
}

void loop() {
  recvWithStartEndMarkers();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    calculate();
    isStalled = false; // *** ADD: Reset stall status for new command ***
    runStepper();

    showParsedData();
    newData = false;
  }

  
  if (stepper.distanceToGo() != 0 && !isStalled) {
    checkStallGuard(); // Poll driver status over SPI
    stepper.runSpeed();
  } else {
    stepper.disableOutputs();  
  } 
  
}

void runStepper() {
  float targetSteps = pumpLength * steps_per_mm;
  float speedStepsPerSec = feedrate * steps_per_mm;

  if (targetSteps < 0) {
    speedStepsPerSec = -speedStepsPerSec;
  }

  stepper.move(targetSteps);
  stepper.setSpeed(speedStepsPerSec);
  stepper.enableOutputs();

  moveStartTime = millis(); // Record exact timestamp when movement starts!
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
  flowrate = atof(strtokIndx);
  //flowrate *= 1000; // convert mL/s to mm^3/s

  strtokIndx = strtok(NULL, ",");
  requested_mL = atof(strtokIndx);
}

void calculate() {
  feedrate = flowrate * mm_per_mL;
  pumpLength = requested_mL * mm_per_mL;
}

/*
given:
  length of 1mL (mm)
  flowrate requested (mL/s)
  requested volume (mL)

calc:
  feedrate (mm/s) -> mm_per_mL * flowrate --> mm/mL * mL/s = mm/s
  pump length (mm) -> requested_mL * mm_per_mL --> mL * mm/mL = mm
*/


void showParsedData() {
  Serial.print("length of 1mL (mm): ");
  Serial.println(mm_per_mL);
  Serial.print("requested flow rate (mL/s): ");
  Serial.println(flowrate);
  Serial.print("requested volume (mL): ");
  Serial.println(requested_mL);
  Serial.print("pump feed rate (mm/s): ");
  Serial.println(feedrate);
  Serial.print("pump length (mm): ");
  Serial.println(pumpLength);
  Serial.println();
}

void checkStallGuard() {
  if (stepper.speed() == 0) return;

  // REDUCED: Ignore only the first 200ms (0.2 seconds) of movement!
  if (millis() - moveStartTime < 200) return;

  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < 100) return; // Poll every 100ms for faster reaction
  lastPoll = millis();

  uint32_t drv_status = driver.DRV_STATUS();
  uint16_t sg_result = (drv_status & 0x3FF); 
  bool is_standstill = (drv_status >> 31) & 0x01; 

  // Fast stall trigger condition
  if (sg_result < 100 && !is_standstill) {
    isStalled = true;
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition()); 
    stepper.disableOutputs();

    Serial.println("\n*** STALL DETECTED! MOTOR STOPPED. ***\n");
  }
}
