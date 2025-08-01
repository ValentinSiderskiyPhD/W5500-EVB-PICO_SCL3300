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
    3) R = 2.2 kΩ, C = 470 nF -> f_c ≈154 Hz  -> This one
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
#include "hardware/pwm.h"
#include <SCL3300.h>

#define SCL3300_SCK 14    // (Purple)
#define SCL3300_MISO 12   // (Blue)
#define SCL3300_MOSI 15   // (Yellow)
#define SCL3300_CS 13     // (Brown)

SCL3300 inclinometer;



// ——— PWM RESOLUTION ———
typedef uint32_t pwm_level_t;
const uint8_t PWM_BITS = 11;  // global PWM bit-depth (11 bits)                   // global PWM bit-depth
const pwm_level_t PWM_TOP = (1UL << PWM_BITS) - 1;  // top value for wrap

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
char packetBuffer[64];

// ——— PWM PINS ———
const int pwmPinX = 28;  // X-axis PWM output
const int pwmPinY = 27;  // Y-axis PWM output
const int pwmPinZ = 26;  // Z-axis PWM output

// ——— MAPPING CONSTANTS ———
const float V_ZERO = 1.0f;           // voltage at 0°  (0 V)
const float V_REF  = 3.3f;           // PWM full-scale (3.3 V)
const float SLOPE  = -1.0f / 90.0f;    // 1/90 V per degree


/**
 * Map any tilt angle (°) to PWM level (0–PWM_TOP):
 *   volts = V_ZERO + angle * SLOPE
 * Then clamp volts to [0, V_REF] and scale to PWM_TOP.
 */
pwm_level_t angleToDutyCycle(float angle) {
  float volts = V_ZERO + angle * SLOPE;
  if (volts < 0.0f) volts = 0.0f;
  if (volts > V_REF) volts = V_REF;
  return (pwm_level_t)((volts / V_REF) * PWM_TOP + 0.5f);
}



// Configure each pin for hardware PWM at fixed resolution and ~250 kHz carrier
void setupFastPWM(int pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(pin);
  pwm_set_wrap(slice, PWM_TOP);
  // For zero jitter at 11 bits, use divider=1 (wrap+1=2048 → 200MHz/2048≈97.656kHz)
  pwm_set_clkdiv_int_frac(slice, 1, 0);
  pwm_set_enabled(slice, true);
}

void setup() {
  // Start Serial early for diagnostics (no blocking wait)
  Serial.begin(2000000);
  delay(2000);
  Serial.println("UDP->PWM Receiver Booting...");

  // Print the angle-to-voltage mapping at startup for quick reference
  Serial.printf("Mapping: 0° -> %.3f V; 90° -> %.3f V; slope = %.5f V/°", V_ZERO, V_ZERO + 90.0f * SLOPE, SLOPE);

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
  

  // Print the angle-to-voltage mapping for reference
  Serial.printf("Mapping: 0° -> %.3f V; 90° -> %.3f V; slope = %.5f V/°\n",
                V_ZERO, V_ZERO + 90.0f * SLOPE, SLOPE);

  // Configure PWM outputs
  analogWriteResolution(10);  //check if you still need this line.
  pinMode(pwmPinX, OUTPUT);
  pinMode(pwmPinY, OUTPUT);
  pinMode(pwmPinZ, OUTPUT);

setupFastPWM(pwmPinX);
setupFastPWM(pwmPinY);
setupFastPWM(pwmPinZ);

}

pwm_level_t dutyX;
pwm_level_t dutyY;
pwm_level_t dutyZ;

float voltsX;
float voltsY;
float voltsZ;

void loop() {
  char textstring[255];
  char textstring_plot[255];

  if (inclinometer.available()) { // Get next block of data from sensor
    incX = inclinometer.getTiltLevelOffsetAngleX();
    incY = inclinometer.getTiltLevelOffsetAngleY();
    incZ = inclinometer.getTiltLevelOffsetAngleZ();
    delay(14); // Allow a little time for sensor reading (70Hz ideal)

      voltsX = V_ZERO + incX * SLOPE;
      voltsY = V_ZERO + incY * SLOPE;
      voltsZ = V_ZERO + incZ * SLOPE;

      dutyX = angleToDutyCycle(incX);
      dutyY = angleToDutyCycle(incY);
      dutyZ = angleToDutyCycle(incZ);

      pwm_set_gpio_level(pwmPinX, dutyX);
      pwm_set_gpio_level(pwmPinY, dutyY);
      pwm_set_gpio_level(pwmPinZ, dutyZ);

  } else inclinometer.reset();

  // Format the string with fixed-width fields
  sprintf(textstring_plot, "%+07.3f\t%+07.3f\t%+07.3f\n", incX, incY, incZ);
  sprintf(textstring, "X:%+07.3f Y:%+07.3f Z:%+07.3f\n", incX, incY, incZ);

        Serial.printf(
        "X=%.3f° -> %.3f V -> PWM %u\n"
        "Y=%.3f° -> %.3f V -> PWM %u\n"
        "Z=%.3f° -> %.3f V -> PWM %u\n\n",
        incX, voltsX, dutyX,
        incY, voltsY, dutyY,
        incZ, voltsZ, dutyZ
      );
  
  Udp.beginPacket(destip, destPort);
  Udp.write(textstring, strlen(textstring));
  Udp.endPacket();


  Serial.print(textstring_plot);  // Optional for debugging, not essential for UDP
  counter += 1;
 

    if (sscanf(packetBuffer, "X:%f Y:%f Z:%f", &incX, &incY, &incZ) == 3) {

    } else {
      Serial.println("!!! parse error");
    }
  
}
