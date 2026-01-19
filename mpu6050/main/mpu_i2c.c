#include "mpu_i2c.h"

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static const char *TAG = "MPU_I2C";

esp_err_t mpu_i2c_init()
{
    esp_err_t err;

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,                 //I2C 0
        .scl_io_num = MPU_SCL,
        .sda_io_num = MPU_SDA,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master init failed: %s", esp_err_to_name(err));        
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,          //I2C 地址位长度，7
        .device_address = MPU_ADDR,                     //I2C 地址
        .scl_speed_hz = MPU_SCL_SPEED,                  //SCL 时钟速度
        //.scl_wait_us = 10,
    };
    err =  i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c add device failed: %s", esp_err_to_name(err));        
    }
    ESP_LOGI(TAG, "i2c init ok");
    return err;
}

/// @brief MPU60xx,在指定寄存器写入一个数据
/// @param reg 寄存器
/// @param byte 数据
/// @return esp_err_t 写入状态
esp_err_t mpu_write_byte(uint8_t reg,uint8_t byte)
{
    uint8_t date[2]={reg,byte};
    esp_err_t err = i2c_master_transmit(dev_handle,date,2,-1);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master transmit failed: %s", esp_err_to_name(err));        
    }
    return err;
}
/// @brief MPU60xx,在指定寄存器写入一串数据
/// @param reg 寄存器
/// @param write_len 写入的长度
/// @param buf 数据组
/// @return esp_err_t 写入状态
esp_err_t mpu_write_buf(uint8_t reg,uint16_t write_len,uint8_t *buf)
{
    uint8_t write_buf[write_len+1];
    write_buf[0] = reg;
    for(uint16_t i=0;i<write_len;i++){
        write_buf[i+1] = buf[i];
    }
    esp_err_t err = i2c_master_transmit(dev_handle,write_buf,write_len+1,-1);    
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master transmit failed: %s", esp_err_to_name(err));        
    }
    return err;
}

/// @brief 读取指定寄存器数据，读一个
/// @param reg 寄存器地址
/// @return 读取的数据
uint8_t mpu_read_byte(uint8_t reg)
{
    uint8_t read_byte=0;
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1,&read_byte,1,-1);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master receive failed: %s", esp_err_to_name(err));    
        return 0;    
    }
    return read_byte;
}

esp_err_t mpu_read_buf(uint8_t reg,uint16_t read_len,uint8_t *read_buf)
{
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1,read_buf,read_len, -1);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "i2c master receive failed: %s", esp_err_to_name(err));              
    }
    return err;
}
