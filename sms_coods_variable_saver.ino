#include <SoftwareSerial.h>
//#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27, 16, 2);

String Grsp;

bool readingmsg = false;
char sms = ' ';
String latitude;
String longitude;


SoftwareSerial mySerial(3, 2); //SIM800L Tx & Rx is connected to Arduino #3 & #2

void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);

  

  //Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  Serial.println("Initializing...");
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();

  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  updateSerial();

}

void loop()
{
  updateSerial();


}

void updateSerial()
{

  delay(500);
  while (Serial.available())
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port

  }
  while (mySerial.available())
  {
    //Serial.write(mySerial.read());
    char sms = mySerial.read();

    //latitude
    if (sms == '@'){
      
      readingmsg = true;
  
    }
    else if (sms == '&'){
      readingmsg = false;
      Serial.println(latitude);
      
      latitude= "";
      longitude= "";
    }
    else if (readingmsg == true){
      latitude +=sms;
      //Serial.print(sms);
      //Serial.print(sms.toFloat);
      //readingmsg = false;
    }

    //Longitude
    if (sms == ';'){
      
      readingmsg = true;
  
    }
    else if (sms == '!'){
      readingmsg = false;
      Serial.println(longitude);
      
      longitude= "";
      latitude="";
    }
    else if (readingmsg == true){
      longitude +=sms;
      //Serial.print(sms);
      //Serial.print(sms.toFloat);
      //readingmsg = false;
    }
    
    //int index = sms.indexOf(';');
    //String message = sms.substring(index+1);
    //Serial.println("Message is :" + message );
  
    //Serial.print(message);



  }
}

