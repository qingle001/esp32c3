#ifndef __MPU6_I2C_H
#define __MPU6_I2C_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define MPU_SDA     GPIO_NUM_9
#define MPU_SCL     GPIO_NUM_8

#define MPU_ADDR        0x68        //设备地址,AD0->0时地址位0x68,AD0->1时地址位0x69,
#define MPU_SCL_SPEED   400000      //SCK时钟频率，100K




esp_err_t mpu_i2c_init();      //I2C 初始化         
esp_err_t mpu_write_byte(uint8_t reg,uint8_t byte); //I2C 写一byte数据
esp_err_t mpu_write_buf(uint8_t reg,uint16_t write_len,uint8_t *buf);//I2C 写一buf数据
uint8_t mpu_read_byte(uint8_t reg); //I2C 读byte数据
esp_err_t mpu_read_buf(uint8_t reg,uint16_t read_len,uint8_t *read_buf);    ////I2C 读一buf数据
#endif
