#include <SimpleKalmanFilter.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>

/*
 This sample code demonstrates how the SimpleKalmanFilter object can be used with a
 pressure sensor to smooth a set of altitude measurements. 
 This example needs a BMP180 barometric sensor as a data source.
 https://www.sparkfun.com/products/11824

 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SFE_BMP180 pressure;
//Declare Software Serial ---> this will talk to the Bluetooth Module
//    Arduino Rx ----> Tx of BT
//    Arduino Tx ----> Rx of BT ---- this needs to be stepped down from 5 to 3.3 volts 
SoftwareSerial mySSerial(10,11); // Rx, Tx

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
const int buttonPin = 2;
long refresh_time;
int buttonState = 0;

float baseline; // baseline pressure
float PrevAlt;
float CurrentAlt;
double getPressure() {
  char status;
  double T,P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0) {
          return(P);
        }
      } 
    }  
  } 
}

void setup() {

  Serial.begin(115200);
  mySSerial.begin(57600);
   
  // BMP180 Pressure sensor start
  if (!pressure.begin()) {
    Serial.println("BMP180 Pressure Sensor Error!");
    while(1); // Pause forever.
  }
  baseline = getPressure();

  //Target deployment altitude
  #define TARGET 100/3.28084
  #define BASEALT 210


  //Actuation pin start
  pinMode(8,OUTPUT);
  

  //Setup Button Input
  pinMode(buttonPin,INPUT);
 
}

void loop() {
  PrevAlt = CurrentAlt;
  float p = getPressure();
  float altitude = pressure.altitude(p,baseline);
  float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);
  CurrentAlt = estimated_altitude;
  
  delay(SERIAL_REFRESH_TIME);
  if (millis() > refresh_time) {
    Serial.print(altitude*3.28084,6);
    Serial.print(",");
    Serial.print(estimated_altitude*3.28084,6);
    Serial.println(); 
   // Serial.println(CurrentAlt*3.28084,6);
 //   Serial.println(PrevAlt*3.28084,6);

//    mySSerial.print(altitude*3.28084,6);
    mySSerial.println();
    mySSerial.print(estimated_altitude*3.28084,6);
    mySSerial.print(","); 

    //If button is pressed put legs back in place

    buttonState = digitalRead(buttonPin);
     while (buttonState == HIGH){
      Serial.print("Pressed");
      digitalWrite(8,HIGH);
      buttonState = digitalRead(buttonPin); // read the button again
    }
    //Deployment Criteria, still need to add on descent criteria, and after takeoff. Only turns LED on or off
    
     if (PrevAlt > CurrentAlt)
     {
       Serial.println("Descending");
       mySSerial.print("Descending");
       mySSerial.print(",");
       if(estimated_altitude <= TARGET)
        {
        Serial.println("BOOM");
        mySSerial.print("Deploy");
        digitalWrite(8, HIGH);
        }
        else
        {
        Serial.println("No BOOM");
        mySSerial.print("Stay");
        digitalWrite(8,LOW);
        }
     }
     else
     {
      Serial.println("Ascending");
      mySSerial.print("Ascending");
      mySSerial.print(",");
      mySSerial.print("Stay");
      digitalWrite(8,LOW);
     } 
      refresh_time=millis()+SERIAL_REFRESH_TIME;
  }
  }
  




