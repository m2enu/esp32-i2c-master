/**
 * Copyright (C) 2017 m2enu
 *
 * @file i2cmaster.c
 * @brief I2C Master driver for ESP-WROOM-32
 * @author m2enu
 * @date 2017.07.19
 */
#include <stdint.h>
#include <stdio.h>
#include "i2cmaster.h"

static i2c_port_t m_i2c_num = I2C_NUM_0; //!< I2C port number

/** <!-- i2c_error_check {{{1 -->
 * @brief wrapper of ESP_ERROR_CHECK
 * @param[in] err ESP32 error code
 * @return Result of error check
 * @retval false: OK
 * @retval true: NG
 */
bool i2c_error_check(esp_err_t err)
{
    ESP_ERROR_CHECK(err);
    return err != ESP_OK;
}

/** <!-- i2c_return_code {{{1 -->
 * @brief get return code of I2C communication
 * @param[in] err ESP32 error code of I2C communication
 * @return Result of I2C communication
 * @retval Zero: Success
 * @retval +ve_value: Warning
 * @retval -ve_value: Error
 */
int8_t i2c_return_code(esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return I2C_CODE_OK;
        case ESP_ERR_INVALID_STATE:
            // I2C driver not installed or not in master mode.
            return I2C_CODE_INSTALL;
        case ESP_FAIL:
            // Sending command error, slave doesn’t ACK the transfer.
            return I2C_CODE_NACK;
        case ESP_ERR_TIMEOUT:
            // Operation timeout because the bus is busy.
            return I2C_CODE_TIMEOUT;
        default:
            break;
    }
    return I2C_CODE_NG;
}

/** <!-- i2c_master_init {{{1 -->
 * @brief initialize I2C Master
 * @param[in] i2c_num I2C port number (0, 1)
 * @param[in] sda_io_num GPIO port number of SDA
 * @param[in] scl_io_num GPIO port number of SCL
 * @param[in] sda_pullup_en true: pullup SDA
 * @param[in] scl_pullup_en true: pullup SCL
 * @param[in] i2c_freq I2C clock frequency [Hz]
 * @return Result of I2C Master initialization
 * @retval Zero: Success
 * @retval +ve_value: Warning
 * @retval -ve_value: Error
 */
int8_t i2c_master_init(i2c_port_t i2c_num,
                       gpio_num_t sda_io_num,
                       gpio_num_t scl_io_num,
                       bool sda_pullup_en,
                       bool scl_pullup_en,
                       uint32_t i2c_freq
) {
    if ((i2c_num < 0) || (i2c_num > 1)) {
        return I2C_CODE_NUM;
    }
    m_i2c_num = i2c_num;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .sda_pullup_en = sda_pullup_en,
        .scl_pullup_en = scl_pullup_en,
        .master.clk_speed = i2c_freq,
    };
    esp_err_t err;
    err = i2c_param_config(m_i2c_num, &cfg);
    i2c_error_check(err);

    const size_t rxbuf = 0; // receiveing buffer size for slave mode
    const size_t txbuf = 0; // sending    buffer size for slave mode
    const int32_t flags = 0; // Flags used to allocate the interrupt
    err = i2c_driver_install(m_i2c_num, cfg.mode, rxbuf, txbuf, flags);
    i2c_error_check(err);

    return i2c_return_code(err);
}

/** <!-- i2c_wr {{{1 -->
 * @brief I2C master write function
 * @param dev_addr device address (w/o WR/RD flag)
 * @param reg_addr register address
 * @param *reg_data register write data
 * @param cnt write byte count
 * @return Result of I2C Master write
 * @retval Zero: Success
 * @retval +ve_value: Warning
 * @retval -ve_value: Error
 */
int8_t i2c_wr(uint8_t dev_addr,
              uint8_t reg_addr,
              uint8_t *reg_data,
              uint16_t cnt
) {
    uint8_t dev_adwr = (dev_addr << 1) | I2C_MASTER_WRITE;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_adwr, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    esp_err_t err;
    err = i2c_master_cmd_begin(m_i2c_num, cmd, I2C_MAX_WAIT_TICKS);
    i2c_cmd_link_delete(cmd);

    return i2c_return_code(err);
}

/** <!-- i2c_rd {{{1 -->
 * @brief I2C master read function
 * @param dev_addr device address (w/o WR/RD flag)
 * @param reg_addr register address
 * @param *reg_data register read data
 * @param cnt read byte count
 * @return Result of I2C Master write
 * @retval Zero: Success
 * @retval +ve_value: Warning
 * @retval -ve_value: Error
 */
int8_t i2c_rd(uint8_t dev_addr,
              uint8_t reg_addr,
              uint8_t *reg_data,
              uint16_t cnt
) {
    uint8_t dev_adwr = (dev_addr << 1) | I2C_MASTER_WRITE;
    uint8_t dev_adrd = (dev_addr << 1) | I2C_MASTER_READ;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_adwr, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_adrd, true);
    if (cnt > 1) {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err;
    err = i2c_master_cmd_begin(m_i2c_num, cmd, I2C_MAX_WAIT_TICKS);
    i2c_cmd_link_delete(cmd);

    return i2c_return_code(err);
}

// end of file {{{1
// vim:ft=c:et:nowrap:fdm=marker
