
constexpr uint32_t steps_per_mm = 200 * 32 / 8;

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

void setup() {
  Serial.begin(9600);
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
    showParsedData();
    newData = false;
  }
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

//============

void parseData() {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");
  syringeID = atof(strtokIndx);  // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  flowrate = atof(strtokIndx);
  flowrate *= 1000;

  strtokIndx = strtok(NULL, ",");
  mL_toPump = atof(strtokIndx);

  /*
float syringeID = 0.0; 
float flowrate = 0.0;
float mL_toPump = 0.0;
*/
}

void calculate() {
  feedrate = (4.0*flowrate)/(PI*pow(syringeID,2));
  pumpLength = (4.0*1000*mL_toPump)/(PI*pow(syringeID,2));
}


//============

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
