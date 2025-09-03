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


// Setup SPI bus (yeah...)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"

// Define SPI bus for magnetometer


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

bool SFE_MMC5983MA_IO::begin(spi_config cfg, SPISettings userSettings)
{
    useSPI = true;
    esp_err_t err = ESP_OK;
    mag_ctx.cfg = cfg;

    spi_device_interface_config_t devcfg = {
        .command_bits = 1,
        .address_bits = 7,
        .mode = userSettings._dataMode,          //SPI mode 3
        .clock_speed_hz = userSettings._clock,
        .input_delay_ns = 50, // according to datasheet
        .spics_io_num = mag_ctx.cfg.ss,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
    };

    spi_bus_config_t buscfg = {
        .mosi_io_num = mag_ctx.cfg.mosi,
        .miso_io_num = mag_ctx.cfg.miso,
        .sclk_io_num = mag_ctx.cfg.sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    //Attach the magnetometer to the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(mag_ctx.cfg.spi_bus, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(mag_ctx.cfg.spi_bus, &devcfg, &mag_ctx.spi));

    // Initialised SPI
    return true;
}

bool SFE_MMC5983MA_IO::isConnected()
{
    bool result;
    if (useSPI)
    {
        uint8_t *buffer;
        readSingleByte(PROD_ID_REG, buffer);
        return result = (dma_rx_buf[0] == PROD_ID);
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
        esp_err_t err;
        err = spi_device_acquire_bus(mag_ctx.spi, portMAX_DELAY);
        if (err != ESP_OK) {
            return err;
        }

        spi_transaction_t t = {
            .cmd = SPI_CMD_HD_WRBUF,
            .addr = registerAddress,
            .length = packetLength,
            .user = &mag_ctx,
            .tx_buffer = buffer,
        };
        err = spi_device_polling_transmit(mag_ctx.spi, &t);
        spi_device_release_bus(mag_ctx.spi);
        if (err != ESP_OK) {
            return err;
        }
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
        spi_transaction_t t = {
            .cmd = SPI_CMD_HD_RDBUF,
            .addr = registerAddress,
            .rxlength = packetLength,
            .user = &mag_ctx,
            .rx_buffer = buffer,
        };
        esp_err_t err = spi_device_polling_transmit(mag_ctx.spi, &t);
        if (err != ESP_OK) {
            return err;
        }
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
        spi_transaction_t t = {
            .flags = SPI_TRANS_USE_RXDATA,
            .cmd = SPI_CMD_HD_RDBUF,
            .addr = registerAddress,
            .rxlength = 8,
            .user = &mag_ctx,
        };
        esp_err_t err = spi_device_polling_transmit(mag_ctx.spi, &t);
        if (err != ESP_OK) {
            return false;
        }
        *buffer = t.rx_data[0];
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

bool SFE_MMC5983MA_IO::writeSingleByte(const uint8_t registerAddress, uint8_t value)
{
    bool success = true;
    if (useSPI)
    {
        esp_err_t err;
        err = spi_device_acquire_bus(mag_ctx.spi, portMAX_DELAY);
        if (err != ESP_OK) {
            return err;
        }

        spi_transaction_t t = {
            .flags = SPI_TRANS_USE_TXDATA,
            .cmd = SPI_CMD_HD_WRBUF,
            .addr = registerAddress,
            .length = 8,
            .user = &mag_ctx,
            .tx_data = value,
        };
        err = spi_device_polling_transmit(mag_ctx.spi, &t);
        spi_device_release_bus(mag_ctx.spi);
        if (err != ESP_OK) {
            return err;
        }
    }
    else
    {
        _i2cPort->beginTransmission(I2C_ADDR);
        _i2cPort->write(registerAddress);
        _i2cPort->write(value);
        success = _i2cPort->endTransmission() == 0;
    }
    return success;
}

bool SFE_MMC5983MA_IO::setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value |= bitMask;
    success &= writeSingleByte(registerAddress, value);
    return success;
}

bool SFE_MMC5983MA_IO::clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value &= ~bitMask;
    success &= writeSingleByte(registerAddress, value);
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
