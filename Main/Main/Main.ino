
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_BMP280.h>
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

File DATA;
const int chipSelect = BUILTIN_SDCARD;
double Alt_Offset =0;
#define LED 35
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Wire.begin();
  Serial.begin(115200);
  
  delay(100);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // power is on
  
  delay(100);
/* Initalize the SD Card */
  Serial.print("Initializing SD card...");
  //Delete old data instance
  SD.remove("DATA.txt");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    for(int i = 1; i<200; i++){
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW); 
      delay(500);    
    } 
    return;
  }
  Serial.println("initialization Successful.");Serial.println("");
  delay(100);
  
/* Initialise the MEMS sensor */
  Serial.print("Initializing Orientation Sensor...");

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    for(int i = 1; i<200; i++){
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW); 
      delay(500);      
    } 
   while(1);
 }  
  Serial.println("Initialization Successful"); Serial.println("");
//Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");Serial.println("");
  delay(100);
 
///* Initalize the Pressure */
    Serial.print("Initializing BMP280 - Tepmerature & Pressure Sensor...");
    
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));Serial.println("");
    for(int i = 1; i<200; i++){
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW); 
      delay(500);        

  }
 }

  // Determine altitude offset
  delay(100);
 double b = 0;
  
  for(int j = 0; j < 25; j++){  
   double a = bmp.readAltitude(1007.5)+50;
   b = b+a; // sum readings
   delay(100);
  }
  
 Alt_Offset = b/25; // take average
  
  Serial.println("Initialization Successful");
  Serial.print("Altitude offset: ");  Serial.println(Alt_Offset);   Serial.println("");

  delay(100);
// Print Collumn Headers
  Serial.print("Time (sec)");
  Serial.print("\t");
  Serial.print("AccX");
  Serial.print("\t"); 
  Serial.print("AccY");
  Serial.print("\t"); 
  Serial.print("AccZ");
   Serial.print("\t"); 
  Serial.print("Altitude (m)");
  Serial.print("\t"); 
  Serial.print("Altitude (ft)");
  Serial.println("\t"); 
 

// open the file. note that only one file can be open at a time,
// so you have to close this one before opening another.
  DATA = SD.open("DATA.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (DATA) {
  DATA.print("Altitude offset: ");  DATA.println(Alt_Offset);   DATA.println("");
  DATA.print("Time (sec)");
  DATA.print("\t");  
  DATA.print("AccX");
  DATA.print("\t"); 
  DATA.print("AccY");
  DATA.print("\t"); 
  DATA.print("AccZ");
  DATA.print("\t"); 
  DATA.print("Altitude (m)");
  DATA.print("\t");  
  DATA.print("Altitude (ft)");
  DATA.println("\t"); 
  
  // close the file:
    DATA.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  
}

void loop(void)
{
 float Time = millis(); 
  Serial.print(Time/1000);
  Serial.print("\t");
    Serial.print("\t");
  
  // Possible vector values can be:
                // - VECTOR_ACCELEROMETER - m/s^2 CURRENT SELECTION!
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

/* Display the floating point data */
  Serial.print(euler.x());
  Serial.print("\t");
  Serial.print(euler.y());
  Serial.print("\t");
  Serial.print(euler.z());
  Serial.print("\t");

/* Display calibration status for each sensor. */
/*
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
*/

/* Read & Display Pressure Sensor */
  Serial.print(bmp.readAltitude(1007.5)+50-Alt_Offset); // meters
  Serial.print("\t");
  Serial.print("\t");
  Serial.print((bmp.readAltitude(1007.5)-Alt_Offset+50)*3.28084); //feet
  Serial.println("\t");

// open the file. note that only one file can be open at a time,
// so you have to close this one before opening another.
  DATA = SD.open("DATA.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (DATA) {
  DATA.print(Time/1000);
  DATA.print("\t");
  DATA.print("\t"); 
  DATA.print(euler.x());
  DATA.print("\t");
  DATA.print(euler.y());
  DATA.print("\t");
  DATA.print(euler.z());
  DATA.print("\t");
  DATA.print(bmp.readAltitude(1007.5)-Alt_Offset+50); // meters
  DATA.print("\t");  
  DATA.print("\t");  
  DATA.print((bmp.readAltitude(1007.5)-Alt_Offset+50)*3.28084); // ft
  DATA.println("\t");
 
  // close the file:
    DATA.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  delay(100);
}
