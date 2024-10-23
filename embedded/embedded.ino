#include <string>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"

#define I2C i2c0
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define WIRE Wire1
#define SEALEVELPRESSURE_HPA (1013.25)
#define MOTOR_DRIVER_BASE_IN1 14
#define MOTOR_DRIVER_BASE_IN2 15
#define MOTOR_DRIVER_BASE_PWM 12
#define MOTOR_DRIVER_CLAW_IN1 7 // these two just for direction
#define MOTOR_DRIVER_CLAW_IN2 6
#define MOTOR_DRIVER_CLAW_PWM 11 // These are GPIO nums


// SD card notes
// pin 26: SD CS
// pin 25: MISO - data sent from SD to pico
// pin 24: MOSI - data sent from pico to SD
// pin 23: CLOCK



// Motor driver has five modes:
// IN1/IN2
// CW: LH
// CCW: HL
// Short Brake: HH
// Stop: LL
// Standby: standby pin is L, but we didn't wire it 0_o
// Additionally, setting PWM to L causes short brake, unless on standby

const uint32_t BLINK_INTERVAL = 1000;
uint32_t lastBlink = 0;
bool ledState = false;
String get_sensors();
void write_to_SD(String dataline, String filename);
const int chipSelect = 21;
//These are the max and min motor positions! Set properly before running, so arm doesn't break!!!
//They should be numbers from 0 to 1
// Actually, Alex said they range from 0-255

float maxVals[3]={180,180,180};
float minVals[3]={0,0,0};

// Also make sure these are set properly!
int servoPinNums[3]={0,1,4}; // these are set to GPIO servo outs 0-2
Servo servos[3];

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &WIRE);
Adafruit_BMP3XX bmp;

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  delay(1000);
  // Initialize LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn LED on for initialization
  digitalWrite(LED_BUILTIN, !ledState);

  // setup motor driver pins
  gpio_set_function(MOTOR_DRIVER_BASE_PWM, GPIO_FUNC_PWM);
  gpio_set_function(MOTOR_DRIVER_CLAW_PWM, GPIO_FUNC_PWM);
  pinMode(MOTOR_DRIVER_BASE_PWM, OUTPUT);
  pinMode(MOTOR_DRIVER_CLAW_PWM, OUTPUT);
  pinMode(MOTOR_DRIVER_BASE_IN1, OUTPUT);
  pinMode(MOTOR_DRIVER_BASE_IN2, OUTPUT);
  pinMode(MOTOR_DRIVER_CLAW_IN1, OUTPUT);
  pinMode(MOTOR_DRIVER_CLAW_IN2, OUTPUT);

  // Reset pins to L, just in case
  digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
  digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
  digitalWrite(MOTOR_DRIVER_CLAW_IN1, LOW);
  digitalWrite(MOTOR_DRIVER_CLAW_IN2, LOW);

  Serial.println("log:Setup motor pins");

  // setup servo pins
  for (int i =0;i<3;i++){
    servos[i].attach(servoPinNums[i]);
  }

  // setup sensor pin
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  WIRE.setSCL(I2C_SCL_PIN);
  WIRE.setSDA(I2C_SDA_PIN);
  WIRE.begin();

  // BMP is 0x77
  // BNO is 0x28

    /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("log:Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }else{
    Serial.println("log:BNO055 connected");
  }

  Serial.println("log:Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C(0x77, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("log:Could not find a valid BMP3 sensor, check wiring!");
    // while (1);
  }
  Serial.println("log:BMP388 connected");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize SD card
  // Serial.print("Initializing SD card...");

  // if (!SD.begin(chipSelect)) {
  //   Serial.println("initialization failed. Things to check:");
  //   Serial.println("1. is a card inserted?");
  //   Serial.println("2. is your wiring correct?");
  //   Serial.println("3. did you change the chipSelect pin to match your shield or module?");
  //   Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
  //   while (true);
  // }

  // Serial.println("initialization done.");

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


  // Read sensor data
  // I2C, pin 4 and 5 



  // Respond to serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    // expecting input in the form: motor:1.234,1.234,1.234,1.234,1.234
    command.trim();
    String type = command.substring(0,command.indexOf(":"));
    if (type == "ping"){
      Serial.println("pong");
    }else if (type == "motor"){

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

      // servo values, 0-2
      for (int i = 0; i<3; i++){
        if (values[i]>maxVals[i]){
          values[i]=maxVals[i];
        }
        if (values[i]<minVals[i]){
          values[i]=minVals[i];
        }
      }

      // motor values, 3-4
      // BASE directions
      // if(values[3] > 0){
      //   // CW
      //   digitalWrite(MOTOR_DRIVER_BASE_IN2, HIGH);
      //   digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
      // }else if(values[3] < 0){
      //   // CCW
      //   digitalWrite(MOTOR_DRIVER_BASE_IN1, HIGH);
      //   digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
      // }else{
      //   // STOP
      //   digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
      //   digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
      // }

      // // CLAW
      // if(values[4] > 0){
      //   // CW
      //   digitalWrite(MOTOR_DRIVER_CLAW_IN2, HIGH);
      //   digitalWrite(MOTOR_DRIVER_CLAW_IN1, LOW);
      // }else if(values[4] < 0){
      //   // CCW
      //   digitalWrite(MOTOR_DRIVER_CLAW_IN1, HIGH);
      //   digitalWrite(MOTOR_DRIVER_CLAW_IN2, LOW);
      // }else{
      //   // STOP
      //   digitalWrite(MOTOR_DRIVER_CLAW_IN1, LOW);
      //   digitalWrite(MOTOR_DRIVER_CLAW_IN1, LOW);
      // }

      //set motor positions to values

      for (int i =0;i<3;i++){
        servos[i].write(values[i]);
      }

      // digitalWrite(MOTOR_DRIVER_BASE_IN1, HIGH);
      // digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
      // analogWrite(MOTOR_DRIVER_BASE_PWM, 230);
      //analogWrite(MOTOR_DRIVER_CLAW_PWM, abs(values[4]));

    }


  }

  // Display sensor data
  //Serial.println("log_data:" + get_sensors());
  delay(250);
}


// Outputs string of comma separated values, in order of acceleration,altitude,temp
  String get_sensors(){
    String dataline = "";
    double x,y,z = -999;
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    dataline += printEvent(&linearAccelData);
    dataline += bmp.readAltitude(SEALEVELPRESSURE_HPA);
    dataline += ",";
    dataline += bno.getTemp();
    return dataline;
  }

String printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  String data = "";
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
  data += x;
  data += ",";
  data += y;
  data += ",";
  data += z;
  data += ",";
  return data;
}

void write_to_SD(String dataline, String filename){
    File datafile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (datafile) {
    datafile.println(dataline);
    datafile.close();
    // print to the serial port too:
    Serial.println("Wrote: " + dataline + " to the file " + filename);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening file " + filename);
  }
}