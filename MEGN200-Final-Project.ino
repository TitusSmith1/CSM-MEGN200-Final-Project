#include <TinyGPSPlus.h>
#include <Servo.h>  // include the servo library  to control the servos
#include <Wire.h>
#include <PID_v1.h>
#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

TinyGPSPlus gps;  // the TinyGPS++ object
BNO080 myIMU;

float LAT = 0.0000;  // temp values to hold GPS data
float LNG = 0.0000;
float ALT = 0.0000;

const int led = LED_BUILTIN;

unsigned long now;  // timing variables to update  data at a regular interval
unsigned long rc_update;
const int channels = 5;  // specify the number of receiver channels
float RC_in[channels];   // an array to store the calibrated input from  receiver
int servo_channels[] = { 8, 9, 10, 11 };
Servo servos[channels - 1];

//Define Variables we'll be connecting to
double RollSetpoint, RollInput, RollOutput;     // Roll setpoint should be around 0 but can set to other to make plane circle.
double PitchSetpoint, PitchInput, PitchOutput;  // Pitch setpoint will be set when autopilot is activated and will be the default angle of attack.

//Define the aggressive and conservative Tuning Parameters for roll
double rollKp1 = 0.01, rollKi1 = 0.002, rollKd1 = 0.0001;
double rollKp2 = 0.01, rollKi2 = 0.002, rollKd2 = 0.0001;

//Define the aggressive and conservative Tuning Parameters for pitch
double pitchKp1 = 0.01, pitchKi1 = 0.002, pitchKd1 = 0.0001;
double pitchKp2 = 0.01, pitchKi2 = 0.002, pitchKd2 = 0.0001;

//Specify the links and initial tuning parameters
PID rollPID(&RollInput, &RollOutput, &RollSetpoint, rollKp1, rollKi1, rollKd1, DIRECT);
PID pitchPID(&PitchInput, &PitchOutput, &PitchSetpoint, pitchKp1, pitchKi2, pitchKd2, DIRECT);

float roll;
float pitch;
float yaw;

uint16_t flightMode = 0;
uint16_t oldflightMode = 0;

boolean servo_dir[] = { 1, 1, 1, 1, 1 };              // Direction: 1 is normal,  -1 is reverse
float servo_rates[] = { 1, 1, 1, 1, 1 };              // Rates: range 0 to 2 (1 = +-500us  (NORMAL), 2 = +-1000us (MAX)): The amount of servo deflection in both directions
float servo_subtrim[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };  // Subtrimrange -1 to +1 (-1 = 1000us, 0 = 1500us,  1 = 2000us): The neutral position of the servo
//boolean servo_mix_on = true;

void setup() {
  setup_pwmRead();
  Serial.begin(9600);

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  Serial.println("Startup");
  for (int i = 0; i < channels; i++) {
    servos[i].attach(servo_channels[i]);
  }


  Wire.begin();
  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }

  Wire.setClock(3400);  //Increase I2C data rate to 3.4kHz

  myIMU.enableGyroIntegratedRotationVector(50);  //Send data update every 50ms

  Serial.println(F("Gyro integrated rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, gyroX, gyroY, gyroZ"));

  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
}

void loop() {

  now = millis();

  if (RC_avail() || now - rc_update > 25) {  // if RC data is available  or 25ms has passed since last update (adjust to be equal or greater than the frame  rate of receiver)

    rc_update = now;

    //print_RCpwm();                        // uncommment to print raw  data from receiver to serial

    for (int i = 0; i < channels; i++) {  // run through each RC channel

      RC_in[i] = RC_decode(i);  // decode receiver channel and apply failsafe
    }
    flightMode = -((int)RC_in[channels - 1]) + 1;  // More efficent conversion to flight mode based on switch position
    if(flightMode != oldflightMode){
      //only update pid values when necesarry;
      oldflightMode = flightMode;
      if(flightMode==1){
        rollPID.SetTunings(rollKp1,rollKi1,rollKd1);
        pitchPID.SetTunings(pitchKp1,pitchKi1,pitchKd1);
      }
      else if(flightMode==2){
        rollPID.SetTunings(rollKp2,rollKi2,rollKd2);
        pitchPID.SetTunings(pitchKp2,pitchKi2,pitchKd2);
      }
    }
    // If the switch position is 0 (mode 0)then the value stored in the array is 1
    // If the switch position is 2 (mode 2) then the value stored in the array is -1
  }

  // This sketch displays information every time gps data is availible.
  if (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid()) {  // check if the data is valid
        LAT = gps.location.lat();    // store the lattidude
        LNG = gps.location.lng();    // stor the longitude
        //ALT = gps.altitude.meters(); //we will get altitude from barometric pressure sensor
      }
      //displayInfo(); // print out data to serial
    }
  }

  if (myIMU.dataAvailable() == true) {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();

    // Calculate pitch, yaw, and roll angles
    // Convert angles from radians to degrees
    roll = asin(2.0f * (quatReal * quatI + quatJ * quatK)) * 57.32;  //57.32 is ratio of degress to rad
    pitch = atan2(2.0f * (quatReal * quatJ - quatK * quatI), 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ)) * 57.32;
    yaw = atan2(2.0f * (quatReal * quatK - quatI * quatJ), 1.0f - 2.0f * (quatJ * quatJ + quatK * quatK)) * 57.32;


    /*
    Serial.print(pitch, 2);
    Serial.print(F(","));
    Serial.print(yaw, 2);
    Serial.print(F(","));
    Serial.println(roll, 2);
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
    PitchInput = pitch;
    RollInput = roll;
    rollPID.Compute();
    pitchPID.Compute();
    servos[0].writeMicroseconds(calc_uS(RC_in[0], 0));
    servos[3].writeMicroseconds(calc_uS(RC_in[3], 3));
    servos[1].writeMicroseconds(calc_uS(RollOutput, 1));
    servos[2].writeMicroseconds(calc_uS(PitchOutput, 2));
  }
}


void displayInfo() {
  Serial.print(F("Location: "));
  Serial.print(LAT, 6);
  Serial.print(F(","));
  Serial.print(LNG, 6);
  Serial.println();
}



int calc_uS(float cmd, int servo) {  //  cmd = commanded position +-100%
                                     //  servo = servo num (to apply correct direction, rates and trim)
  int i = servo;
  float dir;
  if (servo_dir[i] == 0) dir = -1;
  else dir = 1;  //  set the direction of servo travel

  cmd = 1500 + (cmd * servo_rates[i] * dir + servo_subtrim[i]) * 500;  // apply servo rates and sub trim, then convert to a  uS value

  if (cmd > 2500) cmd = 2500;  //  limit pulsewidth to the range 500 to 2500us
  else if (cmd < 500) cmd = 500;

  return cmd;
}
