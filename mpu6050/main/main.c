#include <stdio.h>
#include "esp_log.h" 
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include "mpu60xx.h"

static const char* TAG = "Main";

void mpu60xx_test()
{
    //float pitch=0,roll=0,yaw=0; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	float temp;					//温度	
	
    if(mpu60xx_get_accelerometer(&aacx,&aacy,&aacz)==ESP_OK){
        temp=mpu60xx_get_temperature();	//得到温度值
        //mpu60xx_get_accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
        mpu60xx_get_gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
        printf("tmp:%.02f,ax:%0d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d\r\n",temp,aacx,aacy,aacz,gyrox,gyroy,gyroz);
    }
}
void app_main(void)
{
	while(mpu60xx_init()!=ESP_OK)
    {
        ESP_LOGI(TAG,"mpu init failed");
    }
     while(1)
    {    
    	mpu60xx_test();
        vTaskDelay(pdTICKS_TO_MS(10));
   }
}
