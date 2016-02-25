#include <Wire.h>
#include "Arduino.h"
#include "GY-85.h"
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45
// Wiring-S           5         6          4
// Sanguino          13        14         12

// The software serial object
AltSoftSerial altSerial;
String Data = "";
// The TinyGPS++ object
TinyGPSPlus gps;
  
void setup() {
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("AHRS Test Begin");
  altSerial.begin(4800);
  Wire.begin();
  SetAccelerometer();
  SetCompass();
  SetGyro();
}

void loop() {
  
  char c;
  int x;
  
  //Basic SoftSerial usage
  /*if (Serial.available()) {
    c = Serial.read();
    altSerial.print(c);
  }*/
  
   /*if (altSerial.available()) {
    c = altSerial.read();
    Serial.print(c);
  }*/
  
    /*while (altSerial.available())
    {
        char character = altSerial.read(); // Receive a single character from the software serial port
        Data.concat(character); // Add the received character to the receive buffer
        if (character == '\n')
        {
            Serial.print("Received: ");
            Serial.println(Data);

            // Add your code to parse the received line here....

            // Clear receive buffer so we're ready to receive the next line
            Data = "";
        }
    }*/
    
    int* accelp;
    int* compassp;
    float* gyrop;
    
    accelp = readFromAccelerometer();
    compassp = readFromCompass();
    gyrop = ReadGyro();
    
    Serial.print("acc");
    Serial.print(" x:");
    Serial.print(*(  accelp));
    Serial.print(" y:");
    Serial.print(*(++accelp));
    Serial.print(" z:");
    Serial.print(*(++accelp));
    
    Serial.print(" comp");
    Serial.print(" x:");
    Serial.print(*(  compassp));
    Serial.print(" y:");
    Serial.print(*(++compassp));
    Serial.print(" z:");
    Serial.print(*(++compassp));
    
    Serial.print(" gyro");
    Serial.print(" x:");
    Serial.print(*(  gyrop)/ 14.375);
    Serial.print(" y:");
    Serial.print(*(++gyrop)/ 14.375);
    Serial.print(" z:");
    Serial.print(*(++gyrop)/ 14.375);
    Serial.print(" temp:");
    Serial.println(35+(*(++gyrop)+13200)/280);
    
    delay(500); //1000ms for 1hz, 20ms for 50Hz, 10ms for 100Hz 
    
    while (altSerial.available() > 0)
       gps.encode(altSerial.read());
       
    Serial.print("Lat:");
    Serial.print(gps.location.lat(), 6); // Latitude in degrees (double)
    Serial.print(" Long:");
    Serial.print(gps.location.lng(), 6); // Longitude in degrees (double)
    Serial.print(" NumSats:");
    Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
  
}

