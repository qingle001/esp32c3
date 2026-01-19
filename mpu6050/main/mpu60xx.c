#include "mpu60xx.h"
#include"freertos/FreeRTOS.h"
static const char *TAG = "MPU";

//ms延时函数，调用freertos的延时函数
void mpu60xx_delay_ms(uint32_t cms)
{
    TickType_t xDelay = cms / portTICK_PERIOD_MS;
    
    vTaskDelay( xDelay );
}
/// @brief mpu初始化，相关寄存器的配置
/// @param  无
/// @return ESP_OK,成功；其他,失败
esp_err_t mpu60xx_init(void)
{
    esp_err_t err = mpu_i2c_init();
    if(err != ESP_OK){       
        return err;      
    }
        mpu_write_byte(MPU_PWR_MGMT1_REG,0X01);	//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
		mpu_write_byte(MPU_PWR_MGMT2_REG,0X00);	//电源管理寄存器2，保持默认值0，所有轴均不待机
		mpu_write_byte(MPU_SAMPLE_RATE_REG,0X09);	//采样率分频寄存器，配置采样率
		mpu_write_byte(MPU_CFG_REG,0X06);	//配置寄存器，配置DLPF
		mpu_write_byte(MPU_GYRO_CFG_REG,0X18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
		mpu_write_byte(MPU_ACCEL_CFG_REG,0X18);	//加速度计配置寄存器，选择满量程为±16g
		//mpu60xx_set_gyro_fsr(3);    //陀螺仪传感器,±2000dps
    	//mpu60xx_set_accel_fsr(0);   //加速度传感器,±2g
		//mpu60xx_set_rate(50);						//设置采样率为50Hz
		ESP_LOGI(TAG, "mpu init ok ");
    return err;
}

#pragma region MPU60xx



/// @brief 设置MPU6050陀螺仪传感器满量程范围
/// @param fsr 0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
/// @return ESP_OK,设置成功；其他,设置失败
esp_err_t mpu60xx_set_gyro_fsr(uint8_t fsr)
{
	return mpu_write_byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
/// @brief 设置MPU6050加速度传感器满量程范围
/// @param fsr 0,±2g;1,±4g;2,±8g;3,±16g
/// @return 
esp_err_t mpu60xx_set_accel_fsr(uint8_t fsr)
{
	return mpu_write_byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
/// @brief 设置MPU6050的数字低通滤波器
/// @param lpf 数字低通滤波频率(Hz)
/// @return ESP_OK,设置成功；其他,设置失败
esp_err_t mpu60xx_set_lfp(uint16_t lpf)  
{ 
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return mpu_write_byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}

/// @brief 设置MPU6050的采样率(假定Fs=1KHz)
/// @param rate 4~1000(Hz)
/// @return ESP_OK,设置成功，其他,设置失败 
esp_err_t  mpu60xx_set_rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=mpu_write_byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return mpu60xx_set_lfp(rate/2);	//自动设置LPF为采样率的一半
}
/// @brief 得到温度值
/// @param  无
/// @return 温度值(扩大了100倍)
float mpu60xx_get_temperature(void)
{
	uint8_t buf[2]; 
    short raw;
	float temp;
	mpu_read_buf(MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp;
}
/// @brief 得到陀螺仪值(原始值)
/// @param gx 陀螺仪x轴的原始读数(带符号)
/// @param gy 陀螺仪y轴的原始读数(带符号)
/// @param gz 陀螺仪z轴的原始读数(带符号)
/// @return ESP_OK,成功，其他,失败 
esp_err_t mpu60xx_get_gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6];
	esp_err_t res;  
	res=mpu_read_buf(MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
/// @brief 得到加速度值(原始值)
/// @param ax 陀螺仪x轴的原始读数(带符号)
/// @param ay 陀螺仪y轴的原始读数(带符号)
/// @param az 陀螺仪z轴的原始读数(带符号)
/// @return ESP_OK,成功，其他,失败 
esp_err_t mpu60xx_get_accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6];  
	esp_err_t res=mpu_read_buf(MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}



#pragma endregion
