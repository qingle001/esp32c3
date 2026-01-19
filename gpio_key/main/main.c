#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

static QueueHandle_t gpio_evt_queue = NULL;//队列

//中断处理函数
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//任务
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {//获取队列值
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));//获取到输出引脚电平，PRIu32是无符号32位数
        }
    }
}

void app_main(void)
{
    //配置GPIO
    gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_NEGEDGE,//中断类型，下降沿触发
      .mode = GPIO_MODE_INPUT,//模式
      .pin_bit_mask = 1ULL<<GPIO_NUM_10,//GPIO位号
      .pull_down_en = 0,//下拉
      .pull_up_en = 1//上拉
    };
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //创建了一个队列
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //创建了一个任务
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //GPIO中断服务安装函数，用于开启中断服务，与 gpio_isr_register() 不兼容，后者是为所有 GPIO 注册一个全局 ISR。
    gpio_install_isr_service(0);
    //为GPIO挂载中断处理函数
    gpio_isr_handler_add(GPIO_NUM_10, gpio_isr_handler, (void*) GPIO_NUM_10);
}
