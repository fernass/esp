#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "iot_servo.h"


#define SERVO_CH0_PIN 27

#define TX_CHANNEL 1
#define RX_CHANNEL 0
#define tx_config_GPIO_NUM GPIO_TRIGGER
#define RMT_RX_GPIO_NUM GPIO_ECHO
#define RMT_CLK_DIV 100
#define tx_config_CARRIER_EN 0
#define rmt_item32_tIMEOUT_US 9500  /*!< RMT receiver timeout value(us) */
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000)  /* Number of clock ticks that represent 10us.  10 us = 1/100th msec = 10E-5 sec ---- 1 us = 1 micro sec */
#define ITEM_DURATION(d) ((d & 0x7fff) * 10 / RMT_TICK_10_US) /* convert time from ticks to micro sec */


#define GPIO_TRIGGER 18
#define GPIO_ECHO 19
#define THRESHOLD_DISTANCE 300

// global variables
QueueHandle_t qh = NULL;

void init_hw_hcsr04_tx(void)
{
    rmt_config_t tx_config;
    tx_config.channel = TX_CHANNEL;
    tx_config.gpio_num = tx_config_GPIO_NUM;
    tx_config.mem_block_num = 1;
    tx_config.clk_div = RMT_CLK_DIV;
    tx_config.tx_config.loop_en = false;
    tx_config.tx_config.carrier_duty_percent = 50;
    tx_config.tx_config.carrier_freq_hz = 3000;
    tx_config.tx_config.carrier_level = 1;
    tx_config.tx_config.carrier_en = tx_config_CARRIER_EN;
    tx_config.tx_config.idle_level = 0;
    tx_config.tx_config.idle_output_en = true;
    tx_config.rmt_mode = 0;
    rmt_config(&tx_config);
    rmt_driver_install(tx_config.channel, 0, 0);

}

void init_hw_hcsr04_rx(void)
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

//***********************************************************
//***********************************************************

void task_hcsr04(void* p){
    //************************** Start measure distance ********************
    init_hw_hcsr04_rx();
    init_hw_hcsr04_tx();

    rmt_item32_t item;
    item.level0 = 1;
    item.duration0 = RMT_TICK_10_US;
    item.level1 = 0;
    item.duration1 = RMT_TICK_10_US; // for one pulse this doesn't matter

    size_t rx_size = 0;
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RX_CHANNEL, &rb);
    rmt_rx_start(RX_CHANNEL, 1);

    double distance = 0;

    for (;;)
    {
        rmt_write_items(TX_CHANNEL, &item, 1, true);
        rmt_wait_tx_done(TX_CHANNEL, portMAX_DELAY);

        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 1000);
        distance = 340.29 * ITEM_DURATION(item->duration0) / (1000 * 1000 * 2); // distance in meters = speed of sound (m/s) * duration of pulse for distance to object and back (2xdistance) in micro sec / 10E6 (conversion of micro sec to sec) / 2 (one way distance)
        printf("Distance is %f cm\n", distance * 100);                          // distance in centimeters
        if(!xQueueSend(qh, &distance, 500))
        {
            printf("Couldn't send distance within 500 ticks");
        }
        else
        {
            printf("Distance sent: %f\n", distance);
        }

        vRingbufferReturnItem(rb, (void *)item);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    //************************** End measure distance ******************** 
}


//***********************************************************
//***********************************************************
static void task_servo(void* p)
{
    servo_config_t servo_cfg = 
    {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_CH0_PIN,
            },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
                LEDC_CHANNEL_2,
                LEDC_CHANNEL_3,
                LEDC_CHANNEL_4,
                LEDC_CHANNEL_5,
                LEDC_CHANNEL_6,
                LEDC_CHANNEL_7,
            },
        },
        .channel_number = 8,
    } ;
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    
    float angle = 0.0f;
    while (1)
    {
        if(angle > 90.0f)
        {
            angle = 0.0f;
        }
        // Set angle to 100 degree
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);

        // Get current angle of servo
        iot_servo_read_angle(LEDC_LOW_SPEED_MODE, 0, &angle);

        //deinit servo
        iot_servo_deinit(LEDC_LOW_SPEED_MODE);

        angle +=45.0f;
        vTaskDelay(5000);
    }
    
    
}

//***********************************************************
//***********************************************************
void task_buzzer(void* p)
{
    double distance;
    if(xQueueReceive(qh, &distance, 500))
    {
        if(distance < THRESHOLD_DISTANCE)
        {
            //siwtch buzzer on
        }
    }
    printf("Start alarm");
}

//***********************************************************
//***********************************************************
void app_main(void)
{
    qh = xQueueCreate(1, sizeof(double));
    xTaskCreate(task_hcsr04, "HCSR04", 500, NULL, 1, NULL);
    xTaskCreate(task_servo, "Servo", 500, NULL, 1, NULL);
    xTaskCreate(task_buzzer, "Buzzer", 500, NULL, 1, NULL);
    vTaskStartScheduler();
    return -1;
}
