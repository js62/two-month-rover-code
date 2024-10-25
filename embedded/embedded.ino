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
#define MOTOR_DRIVER_CLAW_IN1 14
#define MOTOR_DRIVER_CLAW_IN2 15
#define MOTOR_DRIVER_CLAW_PWM 12
#define MOTOR_DRIVER_BASE_IN1 7 // these two just for direction
#define MOTOR_DRIVER_BASE_IN2 6
#define MOTOR_DRIVER_BASE_PWM 11 // These are GPIO nums
#define SD_PIN_CS 21
#define SD_PIN_MISO 20
#define SD_PIN_MOSI 19
#define SD_PIN_SCK 18


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
const uint32_t READ_INTERVAL = 100;
uint32_t lastBlink = 0;
uint32_t lastRead = 0;
bool ledState = false;
String get_sensors();
void write_to_SD(String dataline, String filename);
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
  // gpio_set_function(SD_PIN_CS, GPIO_FUNC_SPI);
  // gpio_set_function(SD_PIN_MISO, GPIO_FUNC_SPI);

  // setup
  SPI.setRX(SD_PIN_MISO);
  SPI.setTX(SD_PIN_MOSI);
  SPI.setSCK(SD_PIN_SCK);

  // setup motor driver pins
  // gpio_set_function(MOTOR_DRIVER_BASE_PWM, GPIO_FUNC_PWM);
  // gpio_set_function(MOTOR_DRIVER_CLAW_PWM, GPIO_FUNC_PWM);
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
  servos[0].attach(servoPinNums[0], 1050, 2400);
  servos[1].attach(servoPinNums[1], 400, 2400);
  servos[2].attach(servoPinNums[2], 400, 2400);

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
    Serial.println("log:Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // while (1);
  }else{
    Serial.println("log:BNO055 connected");
  }

  Serial.println("log:Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C(0x77, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("log:Could not find a valid BMP3 sensor, check wiring!");
    // while (1);
  }else{
    Serial.println("log:BMP388 connected");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // Initialize SD card
  // Serial.print("Initializing SD card...");

  if (!SD.begin(SD_PIN_CS)) {
    Serial.println("log:initialization failed. Check if the SD card is inserted properly.");
  }else{
    Serial.println("log:SD card initialization done.");
    write_to_SD("data.csv", "ACCELERATION_X,ACCELERATION_Y,ACCELERATION_Z,ALTITUDE,PRESSURE,TEMPERATURE");
  }

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
    if (type == "ping"){
      Serial.println("pong");
    } else if (type == "reboot"){
      watchdog_reboot(0,0,0);
    } else if (type == "motor"){

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
      if(values[3] > 0){
        // CW
        digitalWrite(MOTOR_DRIVER_BASE_IN2, HIGH);
        digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
      }else if(values[3] < 0){
        // CCW
        digitalWrite(MOTOR_DRIVER_BASE_IN1, HIGH);
        digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
      }else{
        // STOP
        digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
        digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
      }

      // CLAW
      if(values[4] > 0){
        // CW
        digitalWrite(MOTOR_DRIVER_CLAW_IN2, HIGH);
        digitalWrite(MOTOR_DRIVER_CLAW_IN1, LOW);
      }else if(values[4] < 0){
        // CCW
        digitalWrite(MOTOR_DRIVER_CLAW_IN1, HIGH);
        digitalWrite(MOTOR_DRIVER_CLAW_IN2, LOW);
      }else{
        // STOP
        digitalWrite(MOTOR_DRIVER_CLAW_IN1, LOW);
        digitalWrite(MOTOR_DRIVER_CLAW_IN2, LOW);
      }

      //set motor positions to values

      for (int i =0;i<3;i++){
        servos[i].write(values[i]);
      }

      analogWrite(MOTOR_DRIVER_BASE_PWM, abs(values[3]));
      analogWrite(MOTOR_DRIVER_CLAW_PWM, abs(values[4]));
    }


  }

  // Display sensor data
  if (millis() - lastRead > READ_INTERVAL) {
    lastRead = millis();
      String sensor_data = get_sensors();
      Serial.println("sensor_data:" + sensor_data);
      write_to_SD("data.csv", sensor_data);
  }
}


// Outputs string of comma separated values, in order of acceleration,altitude,pressure,temp
  String get_sensors(){
    String dataline = "";
    double x,y,z = -999;
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    if (!bmp.performReading()) {
      Serial.println("log:Failed to perform BMP reading :(");
    }
    dataline += printEvent(&linearAccelData);
    dataline += bmp.readAltitude(SEALEVELPRESSURE_HPA);
    dataline += ",";
    dataline += bmp.pressure;
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

void write_to_SD(String filename, String dataline){
    File datafile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (datafile) {
    datafile.println(dataline);
    datafile.close();
    // print to the serial port too:
    Serial.println("log:Wrote: " + dataline + " to the file " + filename);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("log:Error opening file " + filename);
  }
}