/* Read Tilt angles from Murata SCL3300 Inclinometer
 * Version 3.2.0 - September 3, 2021
 * Example1_BasicTiltLevelOffset
*/

#include <SPI.h>
#include <SCL3300.h>

#define SCL3300_SCK 14    // (Purple)
#define SCL3300_MISO 12   // (Blue)
#define SCL3300_MOSI 15   // (Yellow)
#define SCL3300_CS 13     // (Brown)

SCL3300 inclinometer;
//Default SPI chip/slave select pin is D10

//// Need the following define for SAMD processors
//#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
//  #define Serial SERIAL_PORT_USBVIRTUAL
//#endif

void setup() {
  Serial.begin(19200);
  delay(2000); //SAMD boards may need a long time to init SerialUSB
  Serial.println("Reading basic Tilt Level Offset values from SCL3300 Inclinometer");

    // Initialize the SPI and SCL3300 sensor
  SPI1.setSCK(SCL3300_SCK);
  SPI1.setCS(SCL3300_CS);
  SPI1.setRX(SCL3300_MISO);
  SPI1.setTX(SCL3300_MOSI);
//  
//  

  if (inclinometer.begin(SPI1,SCL3300_CS) == false) {
    Serial.println("Murata SCL3300 inclinometer not connected.");
    while(1); //Freeze
  }
}

void loop() {
  if (inclinometer.available()) { //Get next block of data from sensor
    //Serial.print("X Tilt: ");
    Serial.print(inclinometer.getTiltLevelOffsetAngleX());
    Serial.print("\t");
    //Serial.print("Y Tilt: ");
    Serial.print(inclinometer.getTiltLevelOffsetAngleY());
    Serial.print("\t");
    //Serial.print("Z Tilt: ");
    Serial.println(inclinometer.getTiltLevelOffsetAngleZ());
    delay(100); //Allow a little time to see the output
  } else inclinometer.reset();
}
