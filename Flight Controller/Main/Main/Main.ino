/*
  SD card read/write
 
 This example shows how to read and write data to and from an SD card file   
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4, pin 10 on Teensy with audio board
 
 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
   
 */
 
#include <SD.h>
#include <SPI.h>

File DATA;

const int chipSelect = BUILTIN_SDCARD;

void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.print("Initializing SD card...");
  //Delete old data instance
  SD.remove("DATA.txt");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
//  // open the file. note that only one file can be open at a time,
//  // so you have to close this one before opening another.
//  Data = SD.open("DATA.txt", FILE_WRITE);
//  
//  // if the file opened okay, write to it:
//  if (DATA) {
//    Serial.print("Writing to test.txt...");
//    myFile.println("testing 1, 2, 3.");
//  // close the file:
//    myFile.close();
//    Serial.println("done.");
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//  }
//  
//  // re-open the file for reading:
//  DATA = SD.open("DATA.txt");
//  if (DATA) {
//    Serial.println("DATA.txt:");
//    
//    // read from the file until there's nothing else in it:
//    while (myFile.available()) {
//      Serial.write(myFile.read());
//    }
//    // close the file:
//    myFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//  }
}

void loop()
{
 // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  DATA = SD.open("DATA.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (DATA) {
    DATA.println("HELLO");
  // close the file:
    DATA.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  
  // re-open the file for reading:
  DATA = SD.open("DATA.txt");
  if (DATA) {
    Serial.println("DATA.txt:");
    // read from the file until there's nothing else in it:
    while (DATA.available()) {
      Serial.write(DATA.read());
    }
    // close the file:
    DATA.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
