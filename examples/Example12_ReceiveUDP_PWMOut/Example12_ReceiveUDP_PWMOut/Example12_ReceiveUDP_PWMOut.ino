/*
  UDP-to-PWM Receiver for W5500-EVB-Pico (RP2040)

  Hardware Setup:
  - W5500-EVB-Pico running this sketch; static IP wired Ethernet
  - PWM outputs on:
      GPIO15 -> X-axis PWM -> RC filter -> Op-amp buffer -> BNC -> LabChart
      GPIO16 -> Y-axis PWM -> RC filter -> Op-amp buffer -> BNC -> LabChart
      GPIO17 -> Z-axis PWM -> RC filter -> Op-amp buffer -> BNC -> LabChart

  RC Filter Options (cutoff ≈150 Hz for 70 Hz updates, 30 kHz PWM):
    1) R = 10 kΩ, C = 100 nF  -> f_c ≈ 159 Hz (simple, easy parts)
    2) R = 4.7 kΩ, C = 220 nF -> f_c ≈ 154 Hz (lower output impedance)
    3) R = 2.2 kΩ, C = 470 nF -> f_c ≈ 154 Hz (stronger drive)
  - Optional: cascade two identical RC stages for 2nd-order (–40 dB/decade)
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ——— NETWORK SETTINGS ———
byte   mac[]       = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip       = { 192, 168, 1, 101 };  // this board’s static IP
unsigned int localPort = 5000;             // must match sender’s port

EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// ——— PWM PINS ———
const int pwmPinX = 15;  // X-axis PWM output
const int pwmPinY = 16;  // Y-axis PWM output
const int pwmPinZ = 17;  // Z-axis PWM output

// ——— MAPPING CONSTANTS ———
const float V_ZERO = 1.0f;           // voltage at 0°  (1 V)
const float V_REF  = 3.3f;           // PWM full-scale (3.3 V)
const float SLOPE  = (2.0f - V_ZERO) / 90.0f;  // (2V - 1V)/90° = 1/90 V per degree

/**
 * Map any tilt angle (°) to 8-bit PWM duty (0–255):
 *   volts = V_ZERO + angle * SLOPE
 * Then clamp volts to [0, V_REF] before scaling.
 */
uint8_t angleToDutyCycle(float angle) {
  float volts = V_ZERO + angle * SLOPE;
  // clamp to hardware limits
  if (volts < 0.0f) volts = 0.0f;
  if (volts > V_REF) volts = V_REF;
  return (uint8_t)((volts / V_REF) * 255.0f + 0.5f);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // bring up Ethernet & UDP
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  Serial.println("UDP→PWM receiver ready");

  // Print the angle-to-voltage mapping for user reference
  Serial.printf("Mapping: 0° -> %.3f V; 90° -> %.3f V; slope = %.5f V/°\n", \
                V_ZERO, V_ZERO + 90.0f * SLOPE, SLOPE);

  // configure 8-bit PWM outputs
  analogWriteResolution(8);
  pinMode(pwmPinX, OUTPUT);
  pinMode(pwmPinY, OUTPUT);
  pinMode(pwmPinZ, OUTPUT);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    // read UDP payload
    int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    packetBuffer[len] = '\0';

    Serial.print("RAW RX: "); Serial.println(packetBuffer);

    float incX, incY, incZ;
    if (sscanf(packetBuffer, "X:%f Y:%f Z:%f", &incX, &incY, &incZ) == 3) {
      // compute volts (for logging)
      float voltsX = V_ZERO + incX * SLOPE;
      float voltsY = V_ZERO + incY * SLOPE;
      float voltsZ = V_ZERO + incZ * SLOPE;

      // compute PWM duties
      uint8_t dutyX = angleToDutyCycle(incX);
      uint8_t dutyY = angleToDutyCycle(incY);
      uint8_t dutyZ = angleToDutyCycle(incZ);

      // log angle -> voltage -> PWM
      Serial.printf(
        "X=%.3f° -> %.3f V -> PWM %u\n"
        "Y=%.3f° -> %.3f V -> PWM %u\n"
        "Z=%.3f° -> %.3f V -> PWM %u\n\n",
        incX, voltsX, dutyX,
        incY, voltsY, dutyY,
        incZ, voltsZ, dutyZ
      );

      // output PWM
      analogWrite(pwmPinX, dutyX);
      analogWrite(pwmPinY, dutyY);
      analogWrite(pwmPinZ, dutyZ);
    } else {
      Serial.println("!!! parse error");
    }
  }
}
