#define EN_PIN 4 
#define DIR_PIN 7 
#define STEP_PIN 8 
#define CS_PIN 10

constexpr uint32_t steps_per_mm = 200 * 16 / 8;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

//input values from master
float syringeID = 0.0;
float flowrate = 0.0; // (mL/s)
float mL_toPump = 0.0;

//calculated values on slave
float feedrate = 0.0; // (mm/s)
float pumpLength = 0.0; // (mm)

boolean newData = false;

#include <TMC2130Stepper.h>
TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);

#include <AccelStepper.h>
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  SPI.begin();
  Serial.begin(9600);
  while(!Serial);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  driver.begin();             // Initiate pins and registeries
  driver.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.stealthChop(1);      // Enable extremely quiet stepping
  driver.stealth_autoscale(1);
  driver.microsteps(16);

  stepper.setMaxSpeed(15*steps_per_mm); 
  stepper.setAcceleration(500*steps_per_mm); 
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  Serial.println("Input data as: <syringe inner diam(mm),flowrate(mL/s),target volume(mL).>\n");
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    calculate();
    runStepper();

    showParsedData();
    newData = false;
  }
  if (stepper.distanceToGo() == 0) {
    stepper.disableOutputs();  
    stepper.stop();
  }
  else {
    stepper.runSpeed();
  }
    
}

void runStepper() {
  stepper.setSpeed(feedrate);
  stepper.move(pumpLength*steps_per_mm);
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
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {  // split the data into its parts
  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");
  syringeID = atof(strtokIndx);  // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  flowrate = atof(strtokIndx);
  flowrate *= 1000;

  strtokIndx = strtok(NULL, ",");
  mL_toPump = atof(strtokIndx);
}

void calculate() {
  feedrate = (4.0*flowrate)/(PI*pow(syringeID,2));
  pumpLength = (4.0*1000*mL_toPump)/(PI*pow(syringeID,2));
}

void showParsedData() {
  Serial.print("syringeID (mm): ");
  Serial.println(syringeID);
  Serial.print("flowrate (mm^3/s): ");
  Serial.println(flowrate);
  Serial.print("pump volume (mL): ");
  Serial.println(mL_toPump);
  Serial.print("feed rate (mm/s): ");
  Serial.println(feedrate);
  Serial.print("pump length (mm): ");
  Serial.println(pumpLength);
  Serial.println();
}
