/*
Titus Smith
CSM MEGN200 project 'Autonomous RC Plane'
12/11/2023
https://www.youtube.com/channel/UC8rHDX951rChz6TMxnTbdBA
*/


#include <Servo.h>  // include the servo library  to control the servos
#include <Wire.h>
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library
#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <SFE_BMP180.h> //https://github.com/LowPowerLab/SFE_BMP180


BNO080 myIMU; // IMU object 
SFE_BMP180 bmp180; // barometric pressure sensor object

float ALT = 0.0000; // current altitude with barometric pressure

float P0 = 1013.0; // Sea level pressure.
double pressure; // current pressure
double temperature = 24.12; // standard temp in *C

const int led = LED_BUILTIN; // builtin LED for debugging

unsigned long now;  // timing variables to update  data at a regular interval
unsigned long bmp_update; // variable to schedule reading the barometric pressure sensor;
unsigned long rc_update; // last time the RC reciever was updated

const int channels = 5;  // specify the number of receiver channels
float RC_in[channels];   // an array to store the calibrated input from  receiver
int servo_channels[] = { 8, 9, 10, 11 }; // Servo channels
Servo servos[channels - 1]; // Servo objects

//Define Variables we'll be connecting to
double RollSetpoint, RollInput, RollOutput;     // Roll setpoint should be around 0 but can set to other to make plane circle.
double PitchSetpoint, PitchInput, PitchOutput;  // Pitch setpoint will be set when autopilot is activated and will be the default angle of attack.

//Define the aggressie and conservative Tuning Parameters for roll (These values can be different if you want to use a different pid gains in different modes
// or have different gains depending on angle)
double rollKp1 = 0.03, rollKi1 = 0.000, rollKd1 = 0.0003; // Note: these pid values are model specific and are calibrated to the input as degrees and the output as
double rollKp2 = 0.03, rollKi2 = 0.000, rollKd2 = 0.0003;//         scaled to 1 to -1 for the servo ouput.

//Define the aggressive and conservative Tuning Parameters for pitch
double pitchKp1 = 0.02, pitchKi1 = 0.000, pitchKd1 = 0.0002;
double pitchKp2 = 0.025, pitchKi2 = 0.000, pitchKd2 = 0.0002;

//Specify the links and initial tuning parameters
PID rollPID(&RollInput, &RollOutput, &RollSetpoint, rollKp1, rollKi1, rollKd1, DIRECT); // PID objects with inputs and outputs as **pointer** values
PID pitchPID(&PitchInput, &PitchOutput, &PitchSetpoint, pitchKp1, pitchKi2, pitchKd2, DIRECT);

float roll;// variables for current orientation based on IMU
float pitch;
float yaw;

uint16_t flightMode = 0; // Flight mode (updated from switch on RC transmitter)
uint16_t oldflightMode = 0;
uint16_t inputmode = 1;


//boolean servo_mix_on = true;
float RollTrim = 0; // Trims (updated when transitioning flight modes)
float PitchTrim = 0;

float max_roll = 30;// max roll and pitch for altitude and GPS tracking
float max_pitch = 15;

float targetGPS[] = { 39.74961, -105.22508 }; // Target GPS (updated from SMS message from arduino Mega board)
float currentGPS[] = { 0, 0 };                              // See other repository on GitHub for Mega Code

float angle = 0.00; // target angle of attack (minimum angle of atack required for level flight)(updated when switching modes)
float targetALT = 1750; // target altitude based on barometric pressure sensor (not currently enabled due to bugs)

uint16_t index = 0;

char currentlat[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' }; // Char arrays to store data from arduino mega board
char currentlng[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };
char targetlat[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };
char targetlng[10] = { '0', '.', '0', '0', '0', '0', '0', '0', '0', '0' };

void setup() {
  setup_pwmRead(); // Initialize code to read RC reciever PWM channels
  Serial.begin(9600); // Initailize Serial port to communicate with Mega board and debut output to computer

  pinMode(led, OUTPUT); // Set LED status
  digitalWrite(led, LOW);
  Serial.println("Startup");
  for (int i = 0; i < channels - 1; i++) { // Initialize Servo objects
    servos[i].attach(servo_channels[i]);
  }


  Wire.begin(); // Initialize barometric pressure sensor and IMU on I2C
  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    digitalWrite(led, HIGH);
    while (1); // Freeze the program if IMU sensor is'nt registered
  }
  if (bmp180.begin() == false) {
    Serial.println("BMP180 not detected at default I2C address. Freezing...");
    digitalWrite(led, HIGH);
    while (1); // Freeze the program
  }

  Wire.setClock(3400);  //Set I2C data rate to 3.4kHz (to preserve CPU time for other things)

  myIMU.enableGyroIntegratedRotationVector(50);  //Set the IMU into Sensor fusion mode (only availible on BNO080) update every 50ms

  rollPID.SetMode(AUTOMATIC); // Initialize PID controllers and set min and max values
  rollPID.SetOutputLimits(-1, 1);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-1, 1);
}

void loop() {

  now = millis();// get the current CPU time

  if (RC_avail() || now - rc_update > 25) {  // if RC data is available  or 25ms has passed since last update (adjust to be equal or greater than the frame  rate of receiver)

    rc_update = now;

    //print_RCpwm();                        // uncommment to print raw  data from receiver to serial

    for (int i = 0; i < channels; i++) {  // run through each RC channel

      RC_in[i] = RC_decode(i);  // decode receiver channel and apply failsafe
    }

    flightMode = -(round(RC_in[channels - 1])) + 1;  // More efficent conversion to flight mode based on switch position
    if (flightMode != oldflightMode) { // If the flight mode changed
      //only update pid values when necesarry;
      oldflightMode = flightMode;
      if (flightMode == 1) {
        RollTrim = RC_in[1];// Set pitch and roll setpoints and trim to the current value
        PitchTrim = RC_in[2];
        RollSetpoint = roll;
        PitchSetpoint = pitch;
        rollPID.SetTunings(rollKp1, rollKi1, rollKd1);
        pitchPID.SetTunings(pitchKp1, pitchKi1, pitchKd1);
        targetALT = ALT;
      } else if (flightMode == 2) {
        RollTrim = RC_in[1];// Set pitch and roll setpoints and trim to the current value
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
  // This sketch decodes the custom serial data packet when availible from the mega board.
  if (Serial.available() > 0) { // Use if statement to not block code execution
    char inputchar = Serial.read(); // If new character availible from mega
    /*
    Serial Sentence in the form of "A<currentlat>B<currentlng>C<targetlat>D<targetlng>\n"
    */
    if (inputchar == 'A') {// Set flags to hold the current part of the sentence being parsed
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
    } else if (inputchar == '\n') {// If we have read the entire sentence into memory then decode it
      displayInfo();
      float num = ((String)currentlat).toFloat();
      if(37<num&&41>num){ // Hard check to make sure that the GPS data is within a few hundred KM 
      currentGPS[0] = num; // ***********Change this if you don't live in Colorado****************
      digitalWrite(led,HIGH);}
      num = ((String)currentlng).toFloat();// Parse char array to float
      if(-103>num &&-110<num){
      currentGPS[1] = num;}
      num = ((String)targetlat).toFloat();
      if(37<num&&41>num){
      targetGPS[0] = num;}
      num = ((String)targetlng).toFloat();
      if(-103>num &&-110<num){
      targetGPS[1] = num;}
    } else {                // If we are reading digits of the sentence then append the digit to the correct Char array
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
      if (index >9) {// If we get to many characters for the array rewrite the last digit as it will have the least impact
        index = 9;
      }
    }
  }


  if (bmp_update <= now) { // If the current CPU time is less than the scheduled bmp update time
    bmp180.getPressure(pressure, temperature);// get the pressure
    ALT = bmp180.altitude(pressure, P0);// Convert to altitude
    bmp_update = now + bmp180.startPressure(3);  //set bmp_update to the future time when we need to read the pressure sensor
    //the 3 parameter is for max oversampling
  }

  if (myIMU.dataAvailable() == true) {
    // If data is availible from the IMU read the values in to memory.
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();

    // Quaternian coversion supplied by ChatGPT https://openai.com/chatgpt
    // Calculate pitch, yaw, and roll angles
    // Convert angles from radians to degrees
    pitch = asin(2.0f * (quatReal * quatI + quatJ * quatK)) * 57.32;  //57.32 is ratio of degress to rad
    roll = atan2(2.0f * (quatReal * quatJ - quatK * quatI), 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ)) * 57.32;
    yaw = atan2(2.0f * (quatReal * quatK - quatI * quatJ), 1.0f - 2.0f * (quatJ * quatJ + quatK * quatK)) * 57.32;

    // Calculate the angle (relative to north) between the target gps and current GPS locations (CCW+)
    angle = -atan2(targetGPS[1] - currentGPS[1], targetGPS[0] - currentGPS[0]) * 57.32;
  }

  if (flightMode == 0) { // If we are in flight mode 0 pass on the servo signals from the reciever to the servos verbatim
    for (int i = 0; i < channels - 1; i++) {
      servos[i].writeMicroseconds(calc_uS(RC_in[i], i));
    }
  } else {// if we are in flight mode 1 or 2
    //the PID gains may be different for mode 1 and mode 2 but we dont change that here.
    if (flightMode == 2) {// If we are in GPS fly to positon mode
      PitchInput = pitch ;//- constrain((targetALT - ALT)*40, -15, 15);     // commented code is to enable barometric pressure alltitude hold (not working)
      RollInput = roll - constrain((angle - yaw) / 2, -30, 30); // update the roll pid value with current roll and target roll angle (we are tricking the PID by 
                                                                  //subtracting the target angle so the plane thinks it is banking left when actualy the gps wants to go right)
    } else {// if we are in mode 1 (active stabilize but no GPS or manual control) (not recommended for long periods)
      PitchInput = pitch ;//- constrain((targetALT - ALT)*40, -15, 15); // commented code is to enable barometric pressure alltitude hold (not working)
      RollInput = roll;
    }
    rollPID.Compute();// Update ouput PID values (values automatically updated since they are pointers)
    pitchPID.Compute();
    servos[0].writeMicroseconds(calc_uS(RC_in[0], 0));// Update servo position
    servos[3].writeMicroseconds(calc_uS(RC_in[3], 3));
    servos[1].writeMicroseconds(calc_uS(RollOutput + RollTrim, 1));    // Make sure to turn off autopilot before steering
    servos[2].writeMicroseconds(calc_uS(PitchOutput + PitchTrim, 2));  // This part is just to make trim effective
  }
}


void displayInfo() {
  // Debugging printout
  /*Serial.print("Location:");
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
  Serial.print(F(","));*/
  Serial.print("Angle:");
  Serial.print(angle, 6);
  Serial.print(F(",Yaw:"));
  Serial.print(yaw, 6);
  Serial.println();
}



int calc_uS(float cmd, int servo) { // Function to convert -1 to +1 to servo PWM
                                     //  cmd = commanded position +-100%
                                     //  servo = servo num (to apply correct direction, rates and trim)

  cmd = 1500 + (cmd)*500;  // apply servo rates and sub trim, then convert to a  uS value

  if (cmd > 2500) cmd = 2500;  //  limit pulsewidth to the range 500 to 2500us
  else if (cmd < 500) cmd = 500;

  return cmd;
}
