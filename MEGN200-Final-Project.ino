#include <TinyGPSPlus.h>
#include <Servo.h>  // include the servo library  to control the servos
#include <Wire.h>
#include <PID_v1.h>

#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

uint16_t flightMode = 0;

float LAT = 0.0000;  // temp values to hold GPS data
float LNG = 0.0000;
float ALT = 0.0000;

const int led = LED_BUILTIN;

TinyGPSPlus gps;  // the TinyGPS++ object

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
      int CH = i + 1;

      RC_in[i] = RC_decode(CH);  // decode receiver channel and apply failsafe
      //if(i==0&&RC_in[0] >0){ digitalWrite(led,HIGH); Serial.println(RC_in[i]);}
      //else if(i==0&&RC_in[0] <=0){ digitalWrite(led,LOW);}

      //print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated  receiver input (+-100%) to serial
    }
    if (RC_in[channels-1] < -0.5) {
      digitalWrite(led, HIGH);
      flightMode = 2;
    } else if (RC_in[channels-1] > 0.5) {
      digitalWrite(led, LOW);
      flightMode = 0;
    } else {
      digitalWrite(led, LOW);
      flightMode = 1;
    }
    //Serial.println();                       // uncomment when printing calibrated receiver input to serial.
    /*
      int servo1_uS;      // variables to store the pulse widths to be sent to the servo
      int  servo2_uS;      
      
      if (servo_mix_on == true){              // MIXING  ON
        
        float mix1 = RC_in[1] - RC_in[2];     // Channel 2 (ELV)  - Channel 3 (AIL)
        float mix2 = RC_in[1] + RC_in[2];     // Channel 2  (ELV) + Channel 3 (AIL)
  
        if(mix1 > 1) mix1 = 1;                //  limit mixer output to +-1
        else if(mix1 < -1) mix1 = -1;
  
        if(mix2  > 1) mix2 = 1;                // limit mixer output to +-1
        else if(mix2  < -1) mix2 = -1;  
  
        // Calculate the pulse widths for the servos
      
        servo1_uS = calc_uS(mix1, 0);         // apply the servo rates,  direction and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds,  uS)
        servo2_uS = calc_uS(mix2, 1);         // apply the servo rates, direction  and sub_trim for servo 2, and convert to a RC pulsewidth (microseconds, uS)
            
      }
      else{                                   // MIXING  OFF
        servo1_uS = calc_uS(RC_in[1],0);      // apply the servo rates, direction  and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds, uS)
        servo2_uS = calc_uS(RC_in[2],1);      // apply the servo rates, direction  and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds, uS)
      }

      servo1.writeMicroseconds(servo1_uS);   // write the pulsewidth  to the servo.
      servo2.writeMicroseconds(servo2_uS);   // write the pulsewidth  to the servo. 
      */
  }

  // If there is any GPS data availible read it out with optional printout parameter
  //if (Serial.available() > 0) getGPS(false);

  // This sketch displays information every time a new sentence is correctly encoded.
  if (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      //displayInfo();
    }
  }

  if (myIMU.dataAvailable() == true) {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();

    // Calculate pitch, yaw, and roll angles
    roll = asin(2.0f * (quatReal * quatI + quatJ * quatK));
    pitch = atan2(2.0f * (quatReal * quatJ - quatK * quatI), 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ));
    yaw = atan2(2.0f * (quatReal * quatK - quatI * quatJ), 1.0f - 2.0f * (quatJ * quatJ + quatK * quatK));

    // Convert angles from radians to degrees
    pitch = pitch * 180.0 / PI;
    yaw = yaw * 180.0 / PI;
    roll = roll * 180.0 / PI;


    
    Serial.print(pitch, 2);
    Serial.print(F(","));
    Serial.print(yaw, 2);
    Serial.print(F(","));
    Serial.println(roll, 2);
    
  }

  if (flightMode == 0) {
    //Serial.println("Flight Mode 0");
    //Serial.println(RC_in[channels-1]);
    for (int i = 0; i < channels - 1; i++) {
      servos[i].writeMicroseconds(calc_uS(RC_in[i], i));
    }
  } else if (flightMode == 1) {
    //Serial.println("Flight Mode 1");
    PitchInput = pitch;
    RollInput = roll;
    rollPID.Compute();
    pitchPID.Compute();
    servos[0].writeMicroseconds(calc_uS(RC_in[0], 0));
    servos[3].writeMicroseconds(calc_uS(RC_in[3], 3));
    servos[1].writeMicroseconds(calc_uS(RollOutput, 1));
    servos[2].writeMicroseconds(calc_uS(PitchOutput, 2));
  } else {
    //Serial.println("Flight Mode 2");
    for (int i = 0; i < channels - 1; i++) {
      servos[i].writeMicroseconds(1000);
    }
  }
}

/*
bool startupGPS(){
  // See if we have recieved any valid GPS locations yet.
  return LAT !=0.0000;
}

void getGPS(bool printout) {
  // Update gps location
  if (gps.encode(Serial.read())) {// if there is data availible from the GPS module
    if (gps.location.isValid()) { // check if the data is valid
      LAT = gps.location.lat(); // store the lattidude
      LNG = gps.location.lng();// stor the longitude
      ALT = gps.altitude.meters();
      if (printout){ // if the printout parameter is true, print out data to serial/
        Serial.print(F("- latitude: "));
        Serial.println(LAT,6);// Print out latitude and longitude to serial (the 6 specifies 6 decimal places).
        Serial.print(F("- longitude: "));
        Serial.println(LNG,6);
        Serial.println(ALT,6);
      }

    }
    else if (printout) {// if the data from the GPS is invalid and printout is true.
      Serial.println(F("- location: INVALID"));
    }
  }
}
*/

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    LAT = gps.location.lat();  // store the lattidude
    LNG = gps.location.lng();  // stor the longitude
    //ALT = gps.altitude.meters();
    Serial.print(LAT, 6);
    Serial.print(F(","));
    Serial.print(LNG, 6);
  } else {
    Serial.print(F("GPS INVALID"));
  }

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
