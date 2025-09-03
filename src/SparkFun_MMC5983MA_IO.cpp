/*
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file implements all functions used in the MMC5983MA High Performance Magnetometer Arduino Library IO layer.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#include "SparkFun_MMC5983MA_IO.h"
#include "SparkFun_MMC5983MA_Arduino_Library_Constants.h"
#include <ESP32DMASPIMaster.h>

// Define SPI bus for magnetometer
ESP32DMASPI::Master master;

static constexpr size_t BUFFER_SIZE = 256;
static constexpr size_t QUEUE_SIZE = 2;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

// Read operations must have the most significant bit set
#define SPI_READ 0x80

bool SFE_MMC5983MA_IO::begin(TwoWire &i2cPort)
{
    useSPI = false;
    _i2cPort = &i2cPort;
    return isConnected();
}

bool SFE_MMC5983MA_IO::begin(spi_config config, SPISettings userSettings)
{
    // to use DMA buffer, use these methods to allocate buffer
    dma_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    dma_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

    //

    master.setDataMode(userSettings._dataMode);           // default: SPI_MODE0
    master.setFrequency(userSettings._clock);            // default: 8MHz
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    master.setQueueSize(QUEUE_SIZE);         // default: 1

    // begin() after setting
    master.begin(config.spi_bus, config.sck, config.miso, config.mosi, config.ss);  // default: HSPI (please refer README for pin assignments)

    return true;
}

bool SFE_MMC5983MA_IO::isConnected()
{
    bool result;
    if (useSPI)
    {
        return readMultipleBytes(PROD_ID_REG, dma_rx_buf, 1);
        result = (dma_rx_buf[0] == PROD_ID);
    }
    else
    {
        _i2cPort->beginTransmission(I2C_ADDR);
        result = (_i2cPort->endTransmission() == 0);
        if (result)
        {
            uint8_t id = 0;
            result &= readSingleByte(PROD_ID_REG, &id);
            result &= id == PROD_ID;
        }
    }
    return result;
}

bool SFE_MMC5983MA_IO::writeMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, uint8_t const packetLength)
{
    bool success = true;
    if (useSPI)
    {
        dma_tx_buf[0] = registerAddress;
        memset(dma_rx_buf, 0, BUFFER_SIZE);
        master.queue(dma_tx_buf, NULL, 1);
        master.queue(NULL, dma_rx_buf, packetLength);
        const std::vector<size_t> received_bytes = master.wait();
        memcpy(buffer, &received_bytes[1], packetLength);
    }
    else
    {
        _i2cPort->beginTransmission(I2C_ADDR);
        _i2cPort->write(registerAddress);
        for (uint8_t i = 0; i < packetLength; i++)
            _i2cPort->write(buffer[i]);
        success = _i2cPort->endTransmission() == 0;
    }
    return success;
}

bool SFE_MMC5983MA_IO::readMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, const uint8_t packetLength)
{
    bool success = true;
    if (useSPI)
    {
        dma_tx_buf[0] = registerAddress | SPI_READ;
        memset(dma_rx_buf, 0, BUFFER_SIZE);
        master.queue(dma_tx_buf, NULL, 1);
        master.queue(NULL, dma_rx_buf, packetLength);
        master.wait();
        const std::vector<size_t> received_bytes = master.wait();
        memcpy(buffer, &received_bytes[1], packetLength);
    }
    else
    {
        _i2cPort->beginTransmission(I2C_ADDR);
        _i2cPort->write(registerAddress);
        success = _i2cPort->endTransmission() == 0;

        uint8_t returned = _i2cPort->requestFrom(I2C_ADDR, packetLength);
        for (uint8_t i = 0; (i < packetLength) && (i < returned); i++)
            buffer[i] = _i2cPort->read();
        success &= returned == packetLength;
    }
    return success;
}

bool SFE_MMC5983MA_IO::readSingleByte(const uint8_t registerAddress, uint8_t *buffer)
{
    bool success = true;
    if (useSPI)
    {
        return readMultipleBytes(registerAddress, buffer, 1);
    }
    else
    {
        _i2cPort->beginTransmission(I2C_ADDR);
        _i2cPort->write(registerAddress);
        success = _i2cPort->endTransmission() == 0;
        
        uint8_t returned = _i2cPort->requestFrom(I2C_ADDR, 1U);
        if (returned == 1)
            *buffer = _i2cPort->read();
        success &= returned == 1;
    }
    return success;
}

bool SFE_MMC5983MA_IO::writeSingleByte(const uint8_t registerAddress, uint8_t *value)
{
    bool success = true;
    if (useSPI)
    {
        return writeMultipleBytes(registerAddress, value, 1);
    }
    else
    {
        _i2cPort->beginTransmission(I2C_ADDR);
        _i2cPort->write(registerAddress);
        _i2cPort->write(value[0]);
        success = _i2cPort->endTransmission() == 0;
    }
    return success;
}

bool SFE_MMC5983MA_IO::setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value |= bitMask;
    success &= writeSingleByte(registerAddress, &value);
    return success;
}

bool SFE_MMC5983MA_IO::clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value &= ~bitMask;
    success &= writeSingleByte(registerAddress, &value);
    return success;
}

bool SFE_MMC5983MA_IO::isBitSet(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    readSingleByte(registerAddress, &value);
    return (value & bitMask);
}

bool SFE_MMC5983MA_IO::spiInUse()
{
    return useSPI;
}
