/*
  UDP-to-PWM Receiver for W5500-EVB-Pico (RP2040)

  Hardware Setup:
  - W5500-EVB-Pico running this sketch; static IP wired Ethernet
  - PWM outputs on GPIO pins (15, 16, 17) for X, Y, Z axes

  RC Filter Connection (per axis):
      PWM_PIN ---- R ---- filter_node ----> Op-amp buffer input
                               |
                               C
                               |
                              GND

  RC Filter Options (f_c ≈150 Hz for 70 Hz updates, 30 kHz PWM):
    1) R = 10 kΩ, C = 100 nF  -> f_c ≈159 Hz
       • Pros: Common values, small caps, easy tuning
       • Cons: 10 kΩ source impedance (rise ~2.2 ms), may sag under load
    2) R = 4.7 kΩ, C = 220 nF -> f_c ≈154 Hz
       • Pros: Lower impedance (~4.7 kΩ), better drive for moderate loads
       • Cons: Slightly larger cap footprint
    3) R = 2.2 kΩ, C = 470 nF -> f_c ≈154 Hz
       • Pros: Strong drive (2.2 kΩ source), robust for long cables
       • Cons: Bulky cap, less common value

  If omitting op-amp buffer:
    • RC filter output (source impedance = R) drives PowerLab directly.
    • PowerLab's ≈1 MΩ input lightly loads the RC, preserving linearity.
    • Lower R (e.g., 2.2 kΩ) recommended to minimize rise time and noise.

  Optional: Cascade two identical RC stages for 2nd-order (–40 dB/decade)
  roll-off (double response time ~4–5 ms) for extra PWM ripple suppression.
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ——— NETWORK SETTINGS ———
// Note: Each W5500 device must have a unique MAC on the network.
byte   mac[]        = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
IPAddress ip        = { 192, 168, 1, 101 };  // this board’s static IP
unsigned int localPort = 16384;              // must match sender’s destPort

EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// ——— PWM PINS ———
const int pwmPinX = 14;  // X-axis PWM output
const int pwmPinY = 15;  // Y-axis PWM output
const int pwmPinZ = 11;  // Z-axis PWM output

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
  // Start Serial early for diagnostics (no blocking wait)
  Serial.begin(9600);
  delay(2000);
  Serial.println("UDP->PWM Receiver Booting...");

  // Print the angle-to-voltage mapping at startup for quick reference
  Serial.printf("Mapping: 0° -> %.3f V; 90° -> %.3f V; slope = %.5f V/°", V_ZERO, V_ZERO + 90.0f * SLOPE, SLOPE);

  // Initialize Ethernet hardware (W5500)
  Ethernet.init(17);  // CS pin for W5500
  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Error: Ethernet hardware not found!");
  } else {
    Serial.print("Ethernet OK. IP: ");
    Serial.println(Ethernet.localIP());
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Warning: Ethernet cable not connected.");
    } else {
      Serial.println("Ethernet link up.");
    }
  }

  // Start UDP listener
  Udp.begin(localPort);
  Serial.print("Listening for UDP on port ");
  Serial.println(localPort);

  // Print the angle-to-voltage mapping for reference
  Serial.printf("Mapping: 0° -> %.3f V; 90° -> %.3f V; slope = %.5f V/°\n",
                V_ZERO, V_ZERO + 90.0f * SLOPE, SLOPE);

  // Configure PWM outputs
  analogWriteResolution(8);
  pinMode(pwmPinX, OUTPUT);
  pinMode(pwmPinY, OUTPUT);
  pinMode(pwmPinZ, OUTPUT);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    packetBuffer[len] = '\0';

    Serial.print("RAW RX: ");
    Serial.println(packetBuffer);

    float incX, incY, incZ;
    if (sscanf(packetBuffer, "X:%f Y:%f Z:%f", &incX, &incY, &incZ) == 3) {
      float voltsX = V_ZERO + incX * SLOPE;
      float voltsY = V_ZERO + incY * SLOPE;
      float voltsZ = V_ZERO + incZ * SLOPE;

      uint8_t dutyX = angleToDutyCycle(incX);
      uint8_t dutyY = angleToDutyCycle(incY);
      uint8_t dutyZ = angleToDutyCycle(incZ);

      Serial.printf(
        "X=%.3f° -> %.3f V -> PWM %u\n"
        "Y=%.3f° -> %.3f V -> PWM %u\n"
        "Z=%.3f° -> %.3f V -> PWM %u\n\n",
        incX, voltsX, dutyX,
        incY, voltsY, dutyY,
        incZ, voltsZ, dutyZ
      );

      analogWrite(pwmPinX, dutyX);
      analogWrite(pwmPinY, dutyY);
      analogWrite(pwmPinZ, dutyZ);
    } else {
      Serial.println("!!! parse error");
    }
  }
}
