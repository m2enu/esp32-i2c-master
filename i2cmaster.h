/**
 * Copyright (C) 2017 m2enu
 *
 * @file i2cmaster.h
 * @brief I2C Master driver for ESP-WROOM-32
 * @author m2enu
 * @date 2017.07.19
 */
#include <stdint.h>
#include <stdio.h>
#include "driver/i2c.h"

#define I2C_MASTER_ACK          0   //!< I2C master ACK flag
#define I2C_MASTER_NACK         1   //!< I2C master NACK flag
#define I2C_MAX_WAIT_TICKS      1000 / portTICK_PERIOD_MS //!< maximum wait ticks

/** <!-- i2c_code_t {{{1 -->
 * @brief Result of I2C communication
 */
typedef enum {
    I2C_CODE_TIMEOUT = -5,  //!< ERROR: I2C communication timeout
    I2C_CODE_NACK = -4,     //!< ERROR: I2C NACK occured
    I2C_CODE_INSTALL = -3,  //!< ERROR: I2C driver install failed
    I2C_CODE_NUM = -2,      //!< ERROR: I2C port number out of range
    I2C_CODE_NG = -1,       //!< ERROR: I2C communication failed
    I2C_CODE_OK = 0,        //!< SUCCESS: I2C communication success
    I2C_CODE_WARN = 1,      //!< WARN: I2C communication failed
} i2c_code_t;

/** <!-- i2c_error_check {{{1 -->
 * @brief wrapper of ESP_ERROR_CHECK
 * @param[in] err ESP32 error code
 * @return Result of error check
 * @retval false: OK
 * @retval true: NG
 */
bool i2c_error_check(esp_err_t err);

/** <!-- i2c_return_code {{{1 -->
 * @brief get return code of I2C communication
 * @param[in] err ESP32 error code of I2C communication
 * @return Result of I2C communication
 * @retval Zero: Success
 * @retval +ve_value: Warning
 * @retval -ve_value: Error
 */
int8_t i2c_return_code(esp_err_t err);

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
                       uint32_t i2c_freq);

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
              uint16_t cnt);

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
              uint16_t cnt);

// end of file {{{1
// vim:ft=cpp:et:nowrap:fdm=marker
