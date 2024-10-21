// SD card notes
// pin 27: SD CS
// pin 26: MISO - data sent from SD to pico
// pin 25: MOSI - data sent from pico to SD
// pin 24: CLOCK

// TODO: check page 275 of the raspberry pi pico documentation

const uint32_t BLINK_INTERVAL = 1000;
uint32_t lastBlink = 0;
bool ledState = false;

//These are the max and min motor positions! Set properly before running, so arm doesn't break!!!
//They should be numbers from 0 to 1

float maxVals[5]={0.9,0.9,0.9,0.9,0.9};
float minVals[5]={0.1,0.1,0.1,0.1,0.1};

// Also make sure these are set properly!
int motorPinNums[5]={0,0,0,0,0};
int sensorPinNums[3]={0,0,0};

void setup() {
  // Initialize LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn LED on for initialization
  digitalWrite(LED_BUILTIN, !ledState);


  // setup motor pins
  for (int i =0;i<5;i++){
    pinMode(motorPinNums[i],OUTPUT);
  }


  // Configure serial transport
  Serial.begin(115200);
  delay(1000);

  // Turn LED off after serial initialization
  digitalWrite(LED_BUILTIN, ledState);
}

void loop() {
  // Blink
  if (millis() - lastBlink > BLINK_INTERVAL) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }

  // Respond to serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    // expecting input in the form: motor:1.234,1.234,1.234,1.234,1.234
    command.trim();
    String type = command.substring(0,command.indexOf(":"));

    if (type == "motor"){

      String temp[5] = {""};

      String partsStr=command.substring(command.indexOf(":")+1,command.length());

      for (int i=0;i<5;i++){
        if(partsStr.indexOf(",") != std::string::npos){
          temp[i]=partsStr.substring(0,partsStr.indexOf(","));
          partsStr=partsStr.substring(partsStr.indexOf(",")+1,partsStr.length());
        }
        else{
          temp[i]=partsStr;
          break;
        }
      }

      //convert array of strings to array of floats
      float values[5] = {0};
      for (int i = 0; i<5; i++){
        values[i] = temp[i].toFloat();
      }
      
      //clamp values so they are in the right range and wont physically break the arm!

      for (int i = 0; i<5; i++){
        if (values[i]>maxVals[i]){
          values[i]=maxVals[i];
        }
        if (values[i]<minVals[i]){
          values[i]=minVals[i];
        }
      }


      //set motor positions to values

      for (int i =0;i<5;i++){
        analogWrite(motorPinNums[i],values[i]);
      }
    }


  }
}