//This program do the following
//  --Sends data to the server
//  --Collects the GPS data.
//  --Keeps track of buttons pressed with special geospatial information.
//  --Displays the information on an OLED.
//
//Programmer: Irak Mayer
//Original Date: 03/06/2022
//License: MIT.


#include <SPI.h>
#include <Wire.h>
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>

// Sets TX/RX communication with the FeatherWing GPS card
#define GPSSerial Serial1

// Initialize connection with the GPS hardware
Adafruit_GPS GPS(&GPSSerial);
//Sample timer for GPS data
uint32_t timer = millis();

// Server address. Easy setting
#define DEST_ADDRESS   2

// My address. Client address.
#define MY_ADDRESS     4

//OLED object
Adafruit_SSD1306 oled = Adafruit_SSD1306();

//Button pins with Adafruit M0 Radio RFM69HCW
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

// Set frequency to 915 MHz
#define RF69_FREQ 915.0

// Pins definition for Feather M0 with Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4

// Radio instance.
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Sent and receive manager
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

//Communication buffer
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

//Initialize GPS hardware
void initGPS()
{
  //Communication speed and GPS settings.
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE);

  Serial.print("\nSTARTING LOGGING....");
  //Use internal GPS logger
  if (GPS.LOCUS_StartLogger())
    Serial.println(" STARTED!");
  else
    Serial.println(" no response :("); 
  
}

// Initialize OLED display
void initDisplay(){
  // initialize with the I2C addr 0x3C 
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
  oled.display();
  delay(500);
  oled.clearDisplay();
  oled.display();  

  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.println("RFM69 @ ");
  oled.print((int)RF69_FREQ);
  oled.println(" MHz");
  oled.display();

}

//Initialize buttons
void initButtons(){
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
}

//Initialize radio.
void initRadio(){
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true);
}

void setup() 
{
  Serial.begin(115200);

  initDisplay();  
  initButtons();
  initRadio(); 
  initGPS();
  
}

//Read GPS data and send it to the server.
//Parameter indicates if a geospatial label was pressed.
void refreshGPS(int btnPressed)
{
  // read data from the GPS
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    Serial.print(GPS.lastNMEA()); 
    if (!GPS.parse(GPS.lastNMEA())) 
      return; 
  }

  //  Get a new point every 2 seconds 
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    if (GPS.fix) {
      char radiopacket[23] = "";

      //If one of the buttons pressed send a geospatial tag.
      switch (btnPressed ){
        case 0:
            //Format GPS string.
            sprintf(radiopacket,"%.2f,%.6f,%.6f,%.2f",GPS.speed,GPS.latitudeDegrees,GPS.longitudeDegrees,GPS.altitude);
          break;
        case 1:
            sprintf(radiopacket,"Tree,%.6f,%.6f,%.2f",GPS.latitudeDegrees,GPS.longitudeDegrees,GPS.altitude);
          break;
        case 2:
            sprintf(radiopacket,"Home,%.6f,%.6f,%.2f",GPS.latitudeDegrees,GPS.longitudeDegrees,GPS.altitude);
          break;
        case 3:
            sprintf(radiopacket,"Drv,%.6f,%.6f,%.2f",GPS.latitudeDegrees,GPS.longitudeDegrees,GPS.altitude);
          break;
        
      }
      //Send package to server
      if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;   
        if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
          buf[len] = 0;
          Serial.print("Reply from #");
          Serial.print(from); Serial.print(": ");
          Serial.println((char*)buf);
  
          oled.clearDisplay();
          oled.setCursor(0,0);
          oled.print("Reply:"); oled.println((char*)buf);
          oled.print("RSSI: "); oled.print(rf69.lastRssi());
          oled.display(); 
          
        } else {
          Serial.println("No reply");
        }
      } else {
        Serial.println("sendtoWait failed");
      }

    }
  }
  
}

//Program main loop.
void loop()
{  
  int btnPressed = 0;
   if (rf69_manager.available()) {
    // Wait for a message addressed to us from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    //Check for acknowledge message
    if (rf69_manager.recvfromAck(buf, &len, &from))
    {
      buf[len] = 0;
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      Serial.print("RSSI: "); Serial.println(rf69.lastRssi(), DEC);

      // echo last button       
      data[0] = buf[8];
      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
    
      oled.clearDisplay();
      oled.setCursor(0,0);
      oled.println((char*)buf);
      oled.print("RSSI: "); oled.print(rf69.lastRssi());
      oled.display(); 
    }
  }

//Check for button pressed. 
  if (!digitalRead(BUTTON_A) || !digitalRead(BUTTON_B) || !digitalRead(BUTTON_C))
  {
    Serial.println("Button pressed!");

    //Assign geospatial code.
    if (!digitalRead(BUTTON_A)) btnPressed = 1; //Tree
    if (!digitalRead(BUTTON_B)) btnPressed = 2; //Home structure
    if (!digitalRead(BUTTON_C)) btnPressed = 3; //Driveway
  }

  refreshGPS(btnPressed);
}
