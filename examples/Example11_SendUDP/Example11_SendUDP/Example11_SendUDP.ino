#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SCL3300.h>

#define SCL3300_SCK 14    // (Purple)
#define SCL3300_MISO 12   // (Blue)
#define SCL3300_MOSI 15   // (Yellow)
#define SCL3300_CS 13     // (Brown)

SCL3300 inclinometer;

// Network configuration
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Unique MAC address for your device
IPAddress ip(192, 168, 1, 50); // Device's static IP address
IPAddress destip(192, 168, 1, 255); // IP address of the server you want to send data to

unsigned int localPort = 16384;
unsigned int destPort = 16384;
int counter = 0;
double incX = 0;
double incY = 0;
double incZ = 0;
EthernetUDP Udp;

void setup() {

  Serial.begin();  // Optional for debugging, not essential for UDP
  delay(2000);     // instead of while
  Serial.println("Reading basic Tilt Level Offset values from SCL3300 Inclinometer");

  // Initialize the SPI and SCL3300 sensor
  SPI1.setSCK(SCL3300_SCK);
  SPI1.setCS(SCL3300_CS);
  SPI1.setRX(SCL3300_MISO);
  SPI1.setTX(SCL3300_MOSI);

  if (inclinometer.begin(SPI1, SCL3300_CS) == false) {
    Serial.println("Murata SCL3300 inclinometer not connected.");
    while(1); // Freeze if inclinometer is not connected
  } else {
    Serial.println("Murata SCL3300 inclinometer detected.");
    // Put this inclinometer into Mode 2, since we are using it to measure 0-90 Degree angles
    inclinometer.setMode(2);
    if (inclinometer.begin()) {
      Serial.println("Murata SCL3300 inclinometer now in Mode 2.");
	  } else {
      Serial.println("Tube Murata SCL3300 inclinometer failed to transition to Mode 2.");
	    while(1); //Freeze
	  }

  }

  Ethernet.init(17);
  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
    while (true) {}
  }
  Udp.begin(localPort);
}

void loop() {
  char textstring[255];
  char textstring_plot[255];

  if (inclinometer.available()) { // Get next block of data from sensor
    incX = inclinometer.getTiltLevelOffsetAngleX();
    incY = inclinometer.getTiltLevelOffsetAngleY();
    incZ = inclinometer.getTiltLevelOffsetAngleZ();
    delay(14); // Allow a little time for sensor reading (70Hz ideal)
  } else inclinometer.reset();

  // Format the string with fixed-width fields
  sprintf(textstring_plot, "%+07.3f\t%+07.3f\t%+07.3f\n", incX, incY, incZ);
  sprintf(textstring, "X:%+07.3f Y:%+07.3f Z:%+07.3f\n", incX, incY, incZ);
  
  Udp.beginPacket(destip, destPort);
  Udp.write(textstring, strlen(textstring));
  Udp.endPacket();


  Serial.print(textstring_plot);  // Optional for debugging, not essential for UDP
  counter += 1;
 
 // delay(100);
}
