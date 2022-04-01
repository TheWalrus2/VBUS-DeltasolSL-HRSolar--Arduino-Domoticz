/* VBus decoder to Domoticz sketch for Arduino MEGA
 * * 
 * Version 1.0 - April 2022
 * 
 * Copyright (c) 2022 - 'TheWalrus'
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Legal Notices
 * RESOL, VBus, VBus.net and others are trademarks or registered trademarks of RESOL - Elektronische Regelungen GmbH.
 * All other trademarks are the property of their respective owners.
 *
 * * Hardware info:
 * VBus is NOT RS485. So you need a specific converter circuit to make the VBus data
 * readable for the Arduino UART.
 * 
 * This sketch uses the Arduino Mega and the ENC28J60 ethernet shield. Using the EthernetENC library https://github.com/jandrassy/EthernetENC
 * You can also use another Arduino like the Uno but it only has one hardware serial port.
 *  
 * Serial1 is used for the VBus module.
 * Serial is used to debug the output to PC. 
 * Vbus serial works with 9600 Baudrate and 8N1.
 * 
 * Arduino Mega:
 * Serial  on pins  0 (RX)  and 1 (TX),
 * Serial1 on pins 19 (RX) and 18 (TX),
 * 
 * Arduino non-Mega:
 * You can use f.i. AltSoftSerial() instead of Serial1.
 * This sketch should work with AltSoftSerial.
 * If you do use it, you need to put the VBus input on the correct softserial pin!
 * If you do not need the debugging option, use 'normal' Serial.
 * 
 */

// SPI and Ethernet are used for Ethernet shield
#include <SPI.h>
#include <EthernetENC.h>

// MAC and IP of the Arduino
 byte mac[] = {
  0xDE, 0xAD, 0xBE, 0x30, 0x39, 0x32};
IPAddress ip(192,168,2,99); //The IP addres of this Arduino

// IP address and port of Domoticz
IPAddress domoticz(192,168,2,1); //The IP addres of the Domoticz server
int port = 8080;

EthernetClient client;

long lastTimevbus = 0;
long intervalvbus = 10000;           // interval at which to send data (milliseconds)

// Change the IDX values below to your own!
int IDXtempCollectoren = 59; //(Resol)
int IDXtempBoilervat = 58; //(Resol)

// Settings for the VBus decoding
#define Sync  0xAA  // Synchronisation bytes
#define FLength 6   // Framelength
#define FOffset 10  // Offset start of Frames
#define FSeptet 4   // Septet byte in Frame
#define ResolAddress 0x3271  //   ConergyDT5 (0x3271) Original default Address of the controller 
#define SENSORNOTCONNECTED 8888 // Sometimes this might be 888 instead.

uint16_t networkaddress;
float Sensor1_temp;
float Sensor2_temp;

float Sensor1_temp_max;
float Sensor2_temp_max;

unsigned char Buffer[180];
volatile unsigned char Bufferlength;

unsigned int Destination_address;
unsigned int Source_address;
unsigned char ProtocolVersion;
unsigned int Command;
unsigned char Framecnt;
unsigned char Septet;
unsigned char Checksum;

long lastTimeTimer;
long timerInterval;
bool all;

// Set to "1" when debugging 
// If enabled, the Arduino sends the decoded values over the Serial port.
// If enabled it also prints the source address in case you do not
// know what controller(s) you have.
#define DEBUG 0

// Clear all maximum values
void VBusClearMax() {
    Sensor1_temp_max=0.0;
    Sensor2_temp_max=0.0;
}

// Initialize some parameters
// Currently setting the controller does nothing special.
void VBusInit(int addr=0) {
    timerInterval=2000;
    if (addr!=0)
        networkaddress=addr;
    else
        networkaddress=ResolAddress;
}

void setup() {

  Ethernet.init(53);  // use pin 53 for Ethernet CS
  Ethernet.begin(mac, ip);

  Serial.begin(57600);
  Serial1.begin(9600);

  Serial.println("Arduino debugging started");

  all=false;
  VBusClearMax();
  VBusInit(0x3271);   //Default
} // end void setup()

void loop() {

if (VBusRead())
//  //#if DEBUG
        Serial.println("------Decoded VBus data------");
        Serial.print("Destination: ");
        Serial.println(Destination_address, HEX);
        Serial.print("Source: ");
        Serial.println(Source_address, HEX);
        Serial.print("Protocol Version: ");
        Serial.println(ProtocolVersion);
        Serial.print("Command: ");
        Serial.println(Command, HEX);
        Serial.print("Framecount: ");
        Serial.println(Framecnt);
        Serial.print("Checksum: ");
        Serial.println(Checksum);
        Serial.println("------Values------");
        Serial.print("Sensor Collectoren: ");
        Serial.println(Sensor1_temp);
        Serial.print("Sensor Boiler tank: ");
        Serial.println(Sensor2_temp);
        Serial.println("------END------");
//  //#endif
// } //end VBusRead

/*
 * S1 = Sensor 1 (sensor Collectoren)
   S2 = Sensor 2 (sensor Boiler tank)
 */

  // loop for VBus readout and http GET requests
  // This loop is executed every intervalvbus milliseconds.
  if(millis() - lastTimevbus > intervalvbus) {
    lastTimevbus = millis(); 
    
    Serial.print("Sending to domoticz: ");
    if (Sensor1_temp != 0){httpRequestTemp(IDXtempCollectoren, Sensor1_temp);}
    if (Sensor2_temp != 0){httpRequestTemp(IDXtempBoilervat, Sensor2_temp);}
    
  } //end loop VBus readout 

}// end void loop()

// The following is needed for decoding the data
void  InjectSeptet(unsigned char *Buffer, int Offset, int Length) {
    for (unsigned int i = 0; i < Length; i++) {
        if (Septet & (1 << i)) {
            Buffer [Offset + i] |= 0x80;
        }
    }
}


// The following function reads the data from the bus and converts it all
// depending on the used VBus controller.
bool VBusRead() {
    int F;
    char c;
    bool start,stop,quit;

    start = true;
    stop = false;
    quit = false;
    Bufferlength=0;
    lastTimeTimer = 0;
    lastTimeTimer = millis();

    while ((!stop) and (!quit))  {
          if (Serial1.available()) {
              c=Serial1.read();

            char sync1 = 0xAA;
            if (c == sync1) {         
               if (start) {
                    start=false;
                    Bufferlength=0;

               } else {
                    if (Bufferlength<20) {
                       lastTimeTimer = 0;
                       lastTimeTimer = millis();
                        Bufferlength=0;
                    } else
                        stop=true;
               }
            }
            #if DEBUG
           // Serial.println(c, HEX);
            #endif
            if ((!start) and (!stop)) {
                Buffer[Bufferlength]=c;
                Bufferlength++;
            }
        }
        
        if ((timerInterval > 0) &&  (millis() - lastTimeTimer > timerInterval )  ) {
            quit=true;
        }
    }

    lastTimeTimer = 0;

    if (!quit) {
        Destination_address = Buffer[2] << 8;
        Destination_address |= Buffer[1];
        Source_address = Buffer[4] << 8;
        Source_address |= Buffer[3];
        ProtocolVersion = (Buffer[5]>>4) + (Buffer[5] &(1<<15));

        Command = Buffer[7] << 8;
        Command |= Buffer[6];
        Framecnt = Buffer[8];
        Checksum = Buffer[9];  //TODO check if Checksum is OK

//#if DEBUG
//        Serial.println("---------------");
//        Serial.print("Destination: ");
//        Serial.println(Destination_address, HEX);
//        Serial.print("Source: ");
//        Serial.println(Source_address, HEX);
//        Serial.print("Protocol Version: ");
//        Serial.println(ProtocolVersion);
//        Serial.print("Command: ");
//        Serial.println(Command, HEX);
//        Serial.print("Framecount: ");
//        Serial.println(Framecnt);
//        Serial.print("Checksum: ");
//        Serial.println(Checksum);
//        Serial.println("---------------");    
//#endif
        // Only analyse Commands 0x100 = Packet Contains data for slave
        // with correct length = 10 bytes for HEADER and 6 Bytes  for each frame

        if ((Command==0x0100) and (Bufferlength==10+Framecnt*6)) {
            F=FOffset;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);
//
//            // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
//            Sensor1_temp =CalcTemp(Buffer[F+1], Buffer[F]);
//            // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
//            Sensor2_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);
//
//            //*******************  Frame 2  *******************
            F=FOffset+FLength;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);
//
            Sensor1_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            Sensor2_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);

            ///******************* End of frames ****************


//            if (Sensor1_temp>Sensor1_temp_max)
//                Sensor1_temp_max=Sensor1_temp;
//            if (Sensor2_temp>Sensor2_temp_max)
//                Sensor2_temp_max=Sensor2_temp;
//            if (Sensor3_temp>Sensor3_temp_max)
//                Sensor3_temp_max=Sensor3_temp;
//            if (Sensor4_temp>Sensor4_temp_max)
//                Sensor4_temp_max=Sensor4_temp;
          
        } // end if command 0x0100
    } // end !quit

    return !quit;
} // end VBusRead()


// This function converts 2 data bytes to a temperature value.
float CalcTemp(int Byte1, int Byte2) {
   int v;
    v = Byte1 << 8 | Byte2; //bit shift 8 to left, bitwise OR

    if (Byte1 == 0x00){
    v= v & 0xFF;  
    }

    if (Byte1 == 0xFF)
        v = v - 0x10000;

    if (v==SENSORNOTCONNECTED)
        v=0;

    return (float)((float) v * 0.1);
    }    

//The part below contain all the httprequest functions.

// Send a temperature value to Domoticz
// /json.htm?type=command&param=udevice&idx=IDX&nvalue=0&svalue=TEMP
void httpRequestTemp(int IDX, float temp) {   
  Serial.print("Sending to (domoticz before connected): ");
  Serial.println(temp);
  Serial.print("Sending to domoticz IDX (before connected): ");
  Serial.println(IDX);
  Serial.print("ip domoticz: ");
  Serial.println(domoticz);
  Serial.print("port domoticz: ");
  Serial.println(port);

  // if there's a successful connection:
  if (client.connect(domoticz, port)) {
    Serial.print("Connected with domoticz: ");
      
    client.print( "GET /json.htm?type=command&param=udevice&idx=");
    client.print(IDX);
    client.print("&nvalue=0&svalue=");
    client.print(temp);
    client.println( " HTTP/1.1");
    client.println( "Host: 192.168.2.99"); //change to your IP
    client.println( "Connection: close");
    client.println();

    client.println();
    client.stop();
    delay(150);
  } 
  else {
    client.stop();
  }
}

// Send a switch command to Domoticz
// /json.htm?type=command&param=switchlight&idx=XX&switchcmd=On
void httpRequestSwitch(int IDX, String status) {
  // if there's a successful connection:
  if (client.connect(domoticz, port)) {
    client.print( "GET /json.htm?type=command&param=switchlight&idx=");
    client.print(IDX);
    client.print("&switchcmd=");
    client.print(status);
    client.println( " HTTP/1.1");
    client.println( "Host: 192.168.2.99"); //change to your IP
    client.println( "Connection: close");
    client.println();

    client.println();
    client.stop();
    delay(150);
  } 
  else {
    client.stop();
  }
}

// Total heat value to Domoticz in Wh
// Only send the total from start, not the daily or weekly!
// Domoticz will calculate daily production by itself.
// json.htm?type=command&param=udevice&idx=IDX&svalue=COUNTER
void httpRequestPower(int IDX, float counter) {
  // if there's a successful connection:
  if (client.connect(domoticz, port)) {
    client.print( "GET json.htm?type=command&param=udevice&idx=");
    client.print(IDX);
    client.print("&svalue=");
    client.print(counter);
    client.println( " HTTP/1.1");
    client.println( "Host: 192.168.2.99"); //change to your IP
    client.println( "Connection: close");
    client.println();

    client.println();
    client.stop();
    delay(150);
  } 
  else {
    client.stop();
  }
}
