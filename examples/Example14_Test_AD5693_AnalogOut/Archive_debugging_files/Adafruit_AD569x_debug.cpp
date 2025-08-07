/*!
 * @file Adafruit_AD569x.cpp
 *
 * @mainpage Adafruit AD569x 12-/14-/16-bit I2C DAC driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's AD569x driver for the
 * Arduino platform. It is designed specifically to work with the
 * Adafruit AD5693 breakout: https://www.adafruit.com/product/XXXX
 *
 * @section dependencies Dependencies
 *
 * This library depends on the Adafruit BusIO library.
 *
 * @section author Author
 *
 * Written by ladyada.
 *
 * @section license License
 *
 * MIT License
 *
 */

#include "Adafruit_AD569x.h"

/**
 * @brief Construct a new Adafruit_AD569x object.
 */
Adafruit_AD569x::Adafruit_AD569x() {
  i2c_dev = nullptr; // Initialize the I2C device pointer to nullptr
  _lastError = 0;
  _wire = nullptr;
}

/**
 * @brief Initialize the AD569x chip for communication.
 *
 * This function initializes the I2C interface and sets up the AD569x chip
 * for communication. It should be called in the setup() function of your
 * Arduino sketch. Will perform a soft reset and configure for normal mode,
 * with Vref on, and 1x gain output.
 *
 * @param addr The I2C address of the AD569x chip, 0x4C default or 0x4E
 * @param wire The I2C interface to use (usually &Wire).
 * @return true if the I2C device was found, false otherwise.
 */
uint8_t Adafruit_AD569x::begin(uint8_t addr, TwoWire *wire) {
  // Clean up any existing I2CDevice
  if (i2c_dev) {
    delete i2c_dev;
    i2c_dev = nullptr;
  }

 _wire = wire;
 

  // Create the I2C device
  i2c_dev = new Adafruit_I2CDevice(addr, wire);
 
  // 1) Probe the device
  if (!i2c_dev->begin()) {
    return AD569X_ERR_I2C_INIT;
  }

  // 2) Soft-reset to default regs/output = 0V
  reset();
 
  // 3) Configure mode (normal, internal ref, 1× gain)
  if (!setMode(NORMAL_MODE, true /* enable internal ref */, false /* 1× gain */)) {
    return AD569X_ERR_MODE;
  }

  // Success
  return AD569X_OK;
}


/**
 * @brief Set the operating mode, reference, and gain for the AD569x chip.
 *
 * This function writes to the control register of the AD569x chip to set
 * the operating mode, enable or disable the reference, and set the gain.
 *
 * @param newmode The new operating mode to set (from ad569x_operating_modes
 * enum).
 * @param enable_ref Whether to enable the reference (true) or disable it
 * (false).
 * @param gain2x Whether to set the gain to 2xVref (true) or 1x (false).
 * @return true if the I2C write operation was successful
 */
bool Adafruit_AD569x::setMode(ad569x_operating_modes newmode, bool enable_ref,
                              bool gain2x) {
  // Prepare the command byte
  uint8_t command = ad569x_commands::WRITE_CONTROL;

  // Prepare the high and low data bytes
  uint16_t data = 0x0000;
  data |= ((uint16_t)newmode << 13); // Set D14 and D13 for the operating mode
  data |= ((uint16_t)!enable_ref << 12); // Set D12 for enable_ref
  data |= ((uint16_t)gain2x << 11);      // Set D11 for the gain

  uint8_t highByte = (data >> 8) & 0xFF;
  uint8_t lowByte = data & 0xFF;

  // Perform raw I2C transaction to capture error
  _wire->beginTransmission(i2c_dev->address());
  _wire->write(command);
  _wire->write(highByte);
  _wire->write(lowByte);
  uint8_t err = _wire->endTransmission();  // 0=ACK, others=error

  _lastError = err;
  return (err == 0);

  //// Write the 3-byte buffer to the I2C device and return the result
  //return i2c_dev->write(buffer, 3);
}

// Provide method to retrieve the last I2C error code
uint8_t Adafruit_AD569x::getLastError() {
  return _lastError;
}

/**
 * @brief Write a 16-bit value to the input register and update the DAC
 * register.
 *
 * This function writes a 16-bit value to the input register and then updates
 * the DAC register of the AD569x chip in a single operation
 *
 * @param value The 16-bit value to write to the input register and update the
 * DAC register.
 * @return true if the write and update operation was successful, false
 * otherwise.
 */
bool Adafruit_AD569x::writeUpdateDAC(uint16_t value) {
  // Prepare the command byte
  uint8_t command = ad569x_commands::WRITE_DAC_AND_INPUT;

  // Prepare the high and low data bytes
  uint8_t highByte = (value >> 8) & 0xFF;
  uint8_t lowByte = value & 0xFF;

  // Combine the command and data into a single 3-byte buffer
  uint8_t buffer[3] = {command, highByte, lowByte};

  // Write the 3-byte buffer to the I2C device and return the result
  return i2c_dev->write(buffer, 3);
}

/**
 * @brief Write a 16-bit value to the DAC register... does NOT output it!
 *
 * This function writes a 16-bit value to the input register of the AD569x chip.
 * The data does not appear on the output of the DAC till you run updateDAC()!
 *
 * @param value The 16-bit value to write to the DAC input register.
 * @return true if the write operation was successful, false otherwise.
 */

bool Adafruit_AD569x::writeDAC(uint16_t value) {
  uint8_t command = ad569x_commands::WRITE_INPUT;
  uint8_t highByte = (value >> 8) & 0xFF;
  uint8_t lowByte = value & 0xFF;

  // Combine the command and data into a single 3-byte buffer
  uint8_t buffer[3];
  buffer[0] = command;
  buffer[1] = highByte;
  buffer[2] = lowByte;

  // Write the buffer to the I2C device
  return i2c_dev->write(buffer, 3);
}

/**
 * @brief Update the DAC register from the input register.
 *
 * This function sends the UPDATE_DAC command to the AD569x chip to update
 * the DAC register based on the value stored in the input register.
 *
 * @return true if the update operation was successful, false otherwise.
 */
bool Adafruit_AD569x::updateDAC(void) {
  // Prepare the command byte
  uint8_t command = ad569x_commands::UPDATE_DAC; // Assuming you've defined the
                                                 // enum as ad569x_commands

  // Prepare a 3-byte buffer: command byte followed by two 0x00 bytes
  uint8_t buffer[3] = {command, 0x00, 0x00};

  // Write the 3-byte buffer to the I2C device and return the result
  return i2c_dev->write(buffer, 3);
}

/**
 * @brief Soft-reset the AD569x chip.
 *
 * This function writes 0x8000 to the control register of the AD569x chip
 * to perform a reset operation. Resets the DAC to zero-scale and
 * resets the input, DAC, and control registers to their default values.
 */
void Adafruit_AD569x::reset(void) {
  // Prepare the command byte
  uint8_t command =
      ad569x_commands::WRITE_CONTROL; // Assuming you've defined the enum as
                                      // ad569x_commands

  // Prepare the high and low data bytes for 0x8000
  // Combine the command and data into a single 3-byte buffer
  uint8_t buffer[3] = {command, 0x80, 0x00};

  // Write the 3-byte buffer to the I2C device note this will be an error
  // as it resets before it naks?
  i2c_dev->write(buffer, 3);

   // 2) Wait per datasheet for the internal ref to power up
  delayMicroseconds(600);

// 3) Perform bus recovery on Pico I²C0 (GP4 SDA, GP5 SCL)
  //    to clear any SCL/SDA held low by the reset
  const uint8_t SDA_PIN = 4;
  const uint8_t SCL_PIN = 5;
  // Shut down I²C peripheral
  i2c_dev->end();
  // Switch pins to manual i2c_wiremode and drive high
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  // Pulse SCL 9 times to free stuck slaves
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  // Release lines and re-enable pull-ups
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(10);
  // Re-initialize I²C peripheral
  i2c_dev->begin();
}
