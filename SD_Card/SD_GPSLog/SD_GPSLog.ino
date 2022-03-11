
//This program do the following
//  --Starts commnication using I2C address 0x44
//  --Opens a file on the local SD card named gpslog.txt.
//  --The file is appended every time the program is executed.
//
//Programmer: Irak Mayer
//Original Date: 03/06/2022
//License: MIT.
//
// This program was based on "Wire Slave Receiver" by Nicholas Zambetti <http://www.zambetti.com>


#include <Wire.h>
#include <SPI.h>
#include <SD.h>

//Pin for the Adafruit Feather M0 Adalogger
const int chipSelect = 4;

void setup()
{
  Wire.begin(44);                // join i2c bus with address #44
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           

  //initialize SD card 
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");

   //Create file if not existing or open for append.
   //Mark the begining of new data.
   File dataFile = SD.open("gpslog.txt", FILE_WRITE);
   if (dataFile) {
      dataFile.println("Begin Data");
    }
  dataFile.close();
 
}

void loop()
{
  delay(100);
}

//Event that records the transmitted data into the open file.
void receiveEvent(int howMany)
{
  (void)howMany; // avoid compiler warning about unused parameter

  //Open file to append received data.
  File dataFile = SD.open("gpslog.txt", FILE_WRITE);
  while(0 < Wire.available()) // as long as there is data in the wire
  {
    // Read data
    char c = Wire.read(); 
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(c);
    }
  }
  //Input line feed in file
  dataFile.println("");
  dataFile.close();
}
