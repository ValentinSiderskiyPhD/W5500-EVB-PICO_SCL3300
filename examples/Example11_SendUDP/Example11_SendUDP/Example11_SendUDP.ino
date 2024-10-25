#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Network configuration
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Unique MAC address for your device
IPAddress ip(192, 168, 0, 50); // Device's static IP address
IPAddress destip(192, 168, 0, 3); // IP address of the server you want to send data to

unsigned int localPort = 16384;
unsigned int destPort = 16384;
int counter = 0;
EthernetUDP Udp;

void setup() {
  Serial.begin();
  while (!Serial) {}
  Ethernet.init(17);
  Ethernet.begin(mac,ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
    }
  }
  Udp.begin(localPort);
}

void loop() {
  char textstring[255];
  sprintf(textstring,"Hello %d\n",counter);
  Udp.beginPacket(destip,destPort);
  Udp.write(textstring,strlen(textstring));
  Udp.endPacket();
  Serial.print(textstring);
  counter+=1;
  delay(1000);
}