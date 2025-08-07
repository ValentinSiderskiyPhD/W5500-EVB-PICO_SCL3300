#include "Adafruit_AD569x.h"
#include <math.h> // For sine function

Adafruit_AD569x ad5693; // Create an object of the AD5693 library

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial port to start
  Serial.println("Adafruit AD5693 Test Sketch");

  // 1) Start I²C at 100 kHz instead of 400 kHz
  Wire.begin();           
  Wire.setClock(100000);  
  Serial.println("I2C forced to 100 kHz");

  // 1) Initialize the AD5693, capture the error code
  uint8_t err = ad5693.begin(0x4C, &Wire);  // or 0x4E if A0=HIGH
  if (err != AD569X_OK) {
    Serial.print("❌ AD5693 begin() failed, code = ");
    Serial.println(err);
    Serial.println("   1 = I2C probe failed, 2 = setMode failed");
    uint8_t result = ad5693.begin(0x4C, &Wire);
    if (err == 2) {
      Serial.print("begin() failed, code=");
      Serial.print(result);
      Serial.print(", I2C error=");
      Serial.println(ad5693.getLastError());
    }
    while (1) delay(10);
  }
  Serial.println("✅ AD5693 initialization successful!");

  // Reset the DAC
  ad5693.reset();

  // Configure the DAC for normal mode, internal reference, and no 2x gain
  if (ad5693.setMode(NORMAL_MODE, true, false)) {
    Serial.println("AD5693 configured");
  } else {
    Serial.println("Failed to configure AD5693.");
    while (1) delay(10); // Halt
  }

  // You probably will want to set the I2C clock rate to faster
  // than the default 100KHz, try 400K or 800K or even 1M!
  Wire.setClock(800000);

  Serial.println("Writing 2.5Vpp sine wave to output");
}

void loop() {
  // Generate a sine wave and write it to the DAC
  for (float angle = 0; angle <= 2 * PI; angle += 0.1) {
    uint16_t value = (uint16_t)((sin(angle) + 1) * 32767.5); // Convert sine value to uint16_t
    if (!ad5693.writeUpdateDAC(value)) {
      Serial.println("Failed to update DAC.");
    }
  }
}