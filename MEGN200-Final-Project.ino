#include <Servo.h>  // include the servo library  to control the servos
#include <Wire.h>
#include <PID_v1.h>
#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <SFE_BMP180.h>

BNO080 myIMU;
SFE_BMP180 bmp180;

float ALT = 0.0000;

float P0 = 1013.0;
double pressure;
double temperature = 24.12;

const int led = LED_BUILTIN;

unsigned long now;  // timing variables to update  data at a regular interval
unsigned long bmp_update;
unsigned long rc_update;
const int channels = 5;  // specify the number of receiver channels
float RC_in[channels];   // an array to store the calibrated input from  receiver
int servo_channels[] = { 8, 9, 10, 11 };
Servo servos[channels - 1];

//Define Variables we'll be connecting to
double RollSetpoint, RollInput, RollOutput;     // Roll setpoint should be around 0 but can set to other to make plane circle.
double PitchSetpoint, PitchInput, PitchOutput;  // Pitch setpoint will be set when autopilot is activated and will be the default angle of attack.

//Define the aggressie and conservative Tuning Parameters for roll
double rollKp1 = 0.02, rollKi1 = 0.000, rollKd1 = 0.0003;
double rollKp2 = 0.025, rollKi2 = 0.000, rollKd2 = 0.00035;

//Define the aggressive and conservative Tuning Parameters for pitch
double pitchKp1 = 0.02, pitchKi1 = 0.000, pitchKd1 = 0.0003;
double pitchKp2 = 0.025, pitchKi2 = 0.000, pitchKd2 = 0.00035;

//Specify the links and initial tuning parameters
PID rollPID(&RollInput, &RollOutput, &RollSetpoint, rollKp1, rollKi1, rollKd1, DIRECT);
PID pitchPID(&PitchInput, &PitchOutput, &PitchSetpoint, pitchKp1, pitchKi2, pitchKd2, DIRECT);

float roll;
float pitch;
float yaw;

uint16_t flightMode = 0;
uint16_t oldflightMode = 0;
uint16_t inputmode = 1;


//boolean servo_mix_on = true;
float RollTrim = 0;
float PitchTrim = 0;

float max_roll = 30;
float max_pitch = 15;

float targetGPS[] = { 39.74961, -105.22508 };
float currentGPS[] = { 0, 0 };  //{ 39.749525, -105.22510 };

float angle = 0.00;
float targetALT = 1750;

uint16_t index = 0;

char currentlat[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };
char currentlng[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };
char targetlat[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };
char targetlng[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };

void setup() {
  setup_pwmRead();
  Serial.begin(9600);

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  Serial.println("Startup");
  for (int i = 0; i < channels - 1; i++) {
    servos[i].attach(servo_channels[i]);
  }


  Wire.begin();
  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    digitalWrite(led, HIGH);
    while (1)
      ;
  }
  if (bmp180.begin() == false) {
    Serial.println("BMP180 not detected at default I2C address. Freezing...");
    digitalWrite(led, HIGH);
    while (1)
      ;
  }

  Wire.setClock(3400);  //Increase I2C data rate to 3.4kHz

  myIMU.enableGyroIntegratedRotationVector(50);  //Send data update every 50ms

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-1, 1);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-1, 1);
}

void loop() {

  now = millis();

  if (RC_avail() || now - rc_update > 25) {  // if RC data is available  or 25ms has passed since last update (adjust to be equal or greater than the frame  rate of receiver)

    rc_update = now;

    //print_RCpwm();                        // uncommment to print raw  data from receiver to serial

    for (int i = 0; i < channels; i++) {  // run through each RC channel

      RC_in[i] = RC_decode(i);  // decode receiver channel and apply failsafe
    }

    flightMode = -(round(RC_in[channels - 1])) + 1;  // More efficent conversion to flight mode based on switch position
    if (flightMode != oldflightMode) {
      //only update pid values when necesarry;
      oldflightMode = flightMode;
      if (flightMode == 1) {
        RollTrim = RC_in[1];
        PitchTrim = RC_in[2];
        RollSetpoint = roll;
        PitchSetpoint = pitch;
        rollPID.SetTunings(rollKp1, rollKi1, rollKd1);
        pitchPID.SetTunings(pitchKp1, pitchKi1, pitchKd1);
        targetALT = ALT;
      } else if (flightMode == 2) {
        RollTrim = RC_in[1];
        PitchTrim = RC_in[2];
        RollSetpoint = roll;
        PitchSetpoint = pitch;
        rollPID.SetTunings(rollKp2, rollKi2, rollKd2);
        pitchPID.SetTunings(pitchKp2, pitchKi2, pitchKd2);
        targetALT = ALT;
      }
    }
    // If the switch position is 0 (mode 0)then the value stored in the array is 1
    // If the switch position is 2 (mode 2) then the value stored in the array is -1
  }
  // This sketch displays information every time gps data is availible.
  if (Serial.available() > 0) {
    char inputchar = Serial.read();
    if (inputchar == 'A') {
      inputmode = 1;
      index = 0;
    } else if (inputchar == 'B') {
      inputmode = 2;
      index = 0;
    } else if (inputchar == 'C') {
      inputmode = 3;
      index = 0;
    } else if (inputchar == 'D') {
      inputmode = 4;
      index = 0;
    } else if (inputchar == '\n') {
      displayInfo();
      float num = ((String)currentlat).toFloat();
      if(37<num&&41>num){
      currentGPS[0] = (0.00 != num ? num : currentGPS[0]);}
      num = ((String)currentlng).toFloat();
      if(-103>num &&-110<num){
      currentGPS[1] = (0.00 != num ? num : currentGPS[1]);}
      num = ((String)targetlat).toFloat();
      if(37<num&&41>num){
      targetGPS[0] = (0.00 != num ? num : targetGPS[0]);}
      num = ((String)targetlng).toFloat();
      if(-103>num &&-110<num){
      targetGPS[1] = (0.00 != num ? num : targetGPS[1]);}
    } else {
      if (inputmode == 1) {
        currentlat[index] = inputchar;
        index++;
      } else if (inputmode == 2) {
        currentlng[index] = inputchar;
        index++;
      } else if (inputmode == 3) {
        targetlat[index] = inputchar;
        index++;
      } else if (inputmode == 4) {
        targetlng[index] = inputchar;
        index++;
      }
      if (index >8) {
        index = 8;
      }
    }
  }


  if (bmp_update <= now) {
    bmp180.getPressure(pressure, temperature);
    ALT = bmp180.altitude(pressure, P0);
    bmp_update = now + bmp180.startPressure(3);  //set bmp_update to the future time when we need to read the pressure sensor
    //the 3 parameter is for max oversampling
  }

  if (myIMU.dataAvailable() == true) {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();

    // Calculate pitch, yaw, and roll angles
    // Convert angles from radians to degrees
    pitch = asin(2.0f * (quatReal * quatI + quatJ * quatK)) * 57.32;  //57.32 is ratio of degress to rad
    roll = atan2(2.0f * (quatReal * quatJ - quatK * quatI), 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ)) * 57.32;
    yaw = atan2(2.0f * (quatReal * quatK - quatI * quatJ), 1.0f - 2.0f * (quatJ * quatJ + quatK * quatK)) * 57.32;
    angle = -atan2(targetGPS[1] - currentGPS[1], targetGPS[0] - currentGPS[0]) * 57.32;

    /*
    //Serial.print(pitch, 2);
    //Serial.print(F(","));
    Serial.print(yaw, 2);
    Serial.print(F(","));
    //Serial.print(roll, 2);
    Serial.print(angle,4);
    Serial.print(F(","));
    Serial.print(angle-yaw,4);
    Serial.print(F(","));
    Serial.println(flightMode);
    */
  }

  if (flightMode == 0) {
    //Serial.println("Flight Mode 0");
    //Serial.println(RC_in[channels-1]);
    for (int i = 0; i < channels - 1; i++) {
      servos[i].writeMicroseconds(calc_uS(RC_in[i], i));
    }
  } else {
    //Serial.println("Flight Mode 1 or 2");
    //the PID gains are different for mode 1 and mode 2 but we dont change that here.
    if (flightMode == 2) {
      PitchInput = pitch - constrain((targetALT - ALT), -15, 15);
      RollInput = roll - constrain((angle - yaw) / 2, -30, 30);
    } else {
      PitchInput = pitch;
      RollInput = roll;
    }
    rollPID.Compute();
    pitchPID.Compute();
    servos[0].writeMicroseconds(calc_uS(RC_in[0], 0));
    servos[3].writeMicroseconds(calc_uS(RC_in[3], 3));
    servos[1].writeMicroseconds(calc_uS(RollOutput + RollTrim, 1));    // Make sure to turn off autopilot before steering
    servos[2].writeMicroseconds(calc_uS(PitchOutput + PitchTrim, 2));  // This part is just to make trim effective
    /*
    Serial.print(yaw, 2);
    Serial.print(F(","));
    Serial.print(angle, 2);
    Serial.print(F(","));
    Serial.print(angle-yaw, 2);
    Serial.print(F(","));
    Serial.print(RollInput, 2);
    Serial.print(F(","));
    Serial.println(RollOutput,2);
    */
  }
}


void displayInfo() {
  Serial.print("Location:");
  Serial.print(currentGPS[0], 6);
  Serial.print(F(","));
  Serial.print(currentGPS[1], 6);
  Serial.print(F(","));
  Serial.print(targetGPS[0], 6);
  Serial.print(F(","));
  Serial.print(targetGPS[1], 6);
  Serial.print(F(","));
  Serial.print("Altitude:");
  Serial.print(ALT, 6);
  Serial.print(F(","));
  Serial.print("Angle");
  Serial.print(angle, 6);
  Serial.print(F(","));
  Serial.print(angle-yaw, 6);
  Serial.println();
}



int calc_uS(float cmd, int servo) {  //  cmd = commanded position +-100%
                                     //  servo = servo num (to apply correct direction, rates and trim)

  cmd = 1500 + (cmd)*500;  // apply servo rates and sub trim, then convert to a  uS value

  if (cmd > 2500) cmd = 2500;  //  limit pulsewidth to the range 500 to 2500us
  else if (cmd < 500) cmd = 500;

  return cmd;
}
