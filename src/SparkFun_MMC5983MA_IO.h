/*
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file declares all functions used in the MMC5983MA High Performance Magnetometer Arduino Library I2C/SPI IO layer.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#ifndef _SPARKFUN_MMC5983MA_IO_
#define _SPARKFUN_MMC5983MA_IO_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

struct spi_config {
  spi_host_device_t spi_bus = SPI3_HOST;
  gpio_num_t sck; 
  gpio_num_t miso;
  gpio_num_t mosi; 
  gpio_num_t ss;

  // spi_config() : {};
  // spi_config(uint8_t _spi_bus, int _sck, int _miso, int _mosi, int _ss) : spi_bus(_spi_bus), sck(_sck), miso(_miso), mosi(_mosi), ss(_ss) {};
};

/// Context (config and data) of the spi_eeprom
struct mag_context_t {
    spi_config cfg;
    spi_device_handle_t spi;    ///< SPI device handle
    SemaphoreHandle_t ready_sem; ///< Semaphore for ready signal
};

class SFE_MMC5983MA_IO
{
public:
  // Communication interfaces
  mag_context_t mag_ctx; // spi configuration

  TwoWire *_i2cPort = nullptr;
  uint8_t _address = 0;
  bool useSPI = false;

  // Default empty constructor.
  SFE_MMC5983MA_IO() = default;

  // Default empty destructor
  ~SFE_MMC5983MA_IO() = default;

  // Builds default SPI settings if none are provided.
  void initSPISettings();

  // Configures and starts the I2C I/O layer.
  bool begin(TwoWire &wirePort);

  // Configures the SPI I/O layer using ESP32SPI DMA rather then Arduino SPI class
  bool begin(spi_config cfg, SPISettings userSettings);

  // Returns true if we get the correct product ID from the device.
  bool isConnected();

  // Read a single uint8_t from a register.
  bool readSingleByte(const uint8_t registerAddress, uint8_t *buffer);

  // Writes a single uint8_t into a register.
  bool writeSingleByte(const uint8_t registerAddress, uint8_t value);

  // Reads multiple bytes from a register into buffer uint8_t array.
  bool readMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, const uint8_t packetLength);

  // Writes multiple bytes to register from buffer uint8_t array.
  bool writeMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, const uint8_t packetLength);

  // Sets a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
  bool setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask);

  // Clears a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
  bool clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask);

  // Returns true if a specific bit is set in a register. Bit position ranges from 0 (lsb) to 7 (msb).
  bool isBitSet(const uint8_t registerAddress, const uint8_t bitMask);

  // Returns true if the interface in use is SPI
  bool spiInUse();
};

#endif
