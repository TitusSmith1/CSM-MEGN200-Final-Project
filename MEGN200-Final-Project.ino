#include <TinyGPSPlus.h>

float LAT = 0.0000;// temp values to hold GPS data
float LNG = 0.0000;
float ALT = 0.0000;

const int led = LED_BUILTIN;
TinyGPSPlus gps;                         // the TinyGPS++ object

unsigned long now;                        // timing variables to update  data at a regular interval                  
unsigned long rc_update;
const  int channels = 5;                   // specify the number of receiver channels
float  RC_in[channels];                    // an array to store the calibrated input from  receiver 

void setup() {
    setup_pwmRead();                      
    Serial.begin(9600);

    pinMode(led,OUTPUT);
  digitalWrite(led,LOW);
  Serial.println("Startup");
}

void loop() {  
  
  now = millis();
  
  if(RC_avail() || now - rc_update > 25){   // if RC data is available  or 25ms has passed since last update (adjust to be equal or greater than the frame  rate of receiver)
    
    rc_update = now;                           
    
    //print_RCpwm();                        // uncommment to print raw  data from receiver to serial
    
    for (int i = 0; i<channels; i++){       // run through each RC channel
      int CH = i+1;
      
      RC_in[i]  = RC_decode(CH);             // decode receiver channel and apply failsafe
      if(i==0&&RC_in[0] >0){ digitalWrite(led,HIGH); Serial.println(RC_in[i]);}
      else if(i==0&&RC_in[0] <=0){ digitalWrite(led,LOW);}
      
      //print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated  receiver input (+-100%) to serial       
    }
    //Serial.println();                       // uncomment when printing calibrated receiver input to serial.
  }

  // If there is any GPS data availible read it out with optional printout parameter
  //if (Serial.available() > 0) getGPS(false);

  // This sketch displays information every time a new sentence is correctly encoded.
  if(Serial.available() > 0){
    if (gps.encode(Serial.read())){
      displayInfo();
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

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("GPS INVALID"));
  }

  Serial.println();
}
