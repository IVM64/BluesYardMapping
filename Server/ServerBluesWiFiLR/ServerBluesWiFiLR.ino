//This program do the following
//  --manages a server long range communication.
//  --creates a communication channel with NoteHub.io by means of Blues NoteCard.
//  --Backups all data received in an SD card thru an I2C channel.
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
#include <Notecard.h>

//Pace data submittion since there is a limit of 30 samples/sec as of the writing of this progam for Adafruit.io
unsigned long startTime;
unsigned long delayBetweenSamples = 2500;

//NoteCard product id. Please add your product id below.
#define myProductID "Add your product ID here"
//Create Notecard object instance.
Notecard notecard;

// Client address. Easy setting.
#define CLIENT_ADDRESS   4

// Server address. Easy setting
#define MY_ADDRESS     2

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();

/************ Radio Setup ***************/
#define RF69_FREQ 915.0
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4


// Radio instance.
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Sent and received manager.
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

//Initialize Blues WiFi notecard.
void initNoteCard()
{
    //Use I2C interface to communicate with the notecard.
    Wire.begin();
    notecard.begin(0x17,30,Wire);  

    //Initial settings for notecard.
    J *req = notecard.newRequest("hub.set");
    //Product identification string.
    JAddStringToObject(req, "product", myProductID);
    JAddStringToObject(req, "mode", "continuous");
    notecard.sendRequest(req);
    
}

//Initialize long range radio
void initRadio(){
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  //Set frequency to 915Mhz
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  //Set transmission power
  rf69.setTxPower(20, true);
  
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
 
}

void initOLED(){
  // Initialize OLED display
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.display();
  delay(500);
  oled.clearDisplay();
  oled.display();
  
}

void setup() 
{
  Serial.begin(115200);
  //Initialize OLED display
  initOLED();
  //Initialize long range radion with 915Mhz
  initRadio();
  //Initialize Blues Notecard WiFi carrier.
  initNoteCard();
  startTime = millis();

  // OLED text display use frequency
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.println("RFM69 @ ");
  oled.print((int)RF69_FREQ);
  oled.println(" MHz");
  oled.display();

}

void loop()
{  
  //if communications with radio establish
   if (rf69_manager.available()) {
    
    uint8_t len = sizeof(buf);
    uint8_t from;

    //Check if there is a message from our client
    if (rf69_manager.recvfromAck(buf, &len, &from))
    {
      buf[len] = 0;
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      Serial.print("RSSI: "); Serial.println(rf69.lastRssi(), DEC);

      if (from == CLIENT_ADDRESS)
      {
         //Transmit the data to the Adafruit M0 Adalogger. I2C address 44
         //Save every single received point
         Wire.beginTransmission(44);
         Wire.write((char *)buf);
         Wire.endTransmission();

         //Sample the received points so we do not exceed the 30 samples per second.
         if (millis() - startTime > delayBetweenSamples)
         {
            //Reset the sampling timer
            startTime = millis();

            //Build a NoteCard request object
            J *req = notecard.newRequest("note.add");
            if (req != NULL) {
              //Add a sync request to the NoteCard.
                JAddBoolToObject(req, "sync", true);
                //Create a JSON object
                J *body = JCreateObject();
                if (body != NULL) 
                {
                    //Add key and value to the JSON object
                    JAddStringToObject(body, "loc", (char *)buf);
                    JAddItemToObject(req, "body", body);
                }
                //Send the request to the NoteCard.
                notecard.sendRequest(req);
            }      
            else
            {
              //Something went wrong.
              Serial.print("req failed");
            }
  
         }
        // echo last button       
        data[0] = buf[8];
        // Send a reply back to the client
        if (!rf69_manager.sendtoWait(data, sizeof(data), from))
          Serial.println("sendtoWait failed");

        //Print received buffer and link strength on OLED display
        oled.clearDisplay();
        oled.setCursor(0,0);
        oled.println((char*)buf);
        oled.print("RSSI: "); oled.print(rf69.lastRssi());
        oled.display(); 
      }
    }
  }

}
