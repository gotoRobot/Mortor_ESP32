#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "rotary_encoder.h"

static void pwm_isr(void *arg);
static void current_thread(void *arg);
static void speed_thread(void *arg);

QueueHandle_t g_current_queue;
QueueHandle_t g_current_t_queue;
int32_t g_adc_offset;
rotary_encoder_t *g_encoder = NULL;

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_NUM_21));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11));
    for (uint8_t i = 0; i < 50; i++)
    {
        g_adc_offset += adc1_get_raw(ADC1_CHANNEL_7);
    }
    g_adc_offset /= 50;

    uint32_t pcnt_unit = 0;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 39, 36);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &g_encoder));
    ESP_ERROR_CHECK(g_encoder->set_glitch_filter(g_encoder, 1));
    ESP_ERROR_CHECK(g_encoder->start(g_encoder));

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 22));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 23));

    MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_b_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_a_shdw_full = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_b_shdw_full = 1;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 40000;
    pwm_config.cmpr_a = 50.0f;
    pwm_config.cmpr_b = 50.0f;
    pwm_config.counter_mode =  MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));

    g_current_queue = xQueueCreate(1, sizeof(int32_t));
    g_current_t_queue = xQueueCreate(1, sizeof(float));

    MCPWM0.int_ena.val = MCPWM_TIMER0_TEZ_INT_ENA;
    ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, pwm_isr, NULL, ESP_INTR_FLAG_IRAM, NULL));

    xTaskCreate(current_thread, "current_thread", 4095, NULL, 5, NULL);
    xTaskCreate(speed_thread, "speed_thread", 4095, NULL, 4, NULL);
}

static void IRAM_ATTR pwm_isr(void *arg)
{
    uint32_t mcpwm_intr_status;
    int32_t raw_val;
    mcpwm_intr_status = MCPWM0.int_st.val;
    BaseType_t high_task_awoken = pdFALSE;
    if (mcpwm_intr_status & MCPWM_TIMER0_TEZ_INT_ENA)
    {
        gpio_set_level(GPIO_NUM_21, 1);
        raw_val = adc1_get_raw(ADC1_CHANNEL_7);
        xQueueSendFromISR(g_current_queue, &raw_val, &high_task_awoken);
    }
    MCPWM0.int_clr.val = mcpwm_intr_status;
    portYIELD_FROM_ISR(high_task_awoken);

}

static void current_thread(void *arg)
{
    int32_t raw_val;
    float i_target;
    while (1)
    {
        if (xQueueReceive(g_current_queue, &raw_val, portMAX_DELAY) == pdTRUE)
        {
            gpio_set_level(GPIO_NUM_21, 0);
            xQueueReceive(g_current_t_queue, &i_target, 0);

            float i_current = (float)(raw_val - g_adc_offset) * 0.000634921f;	//raw_val to current(A);
            float i_error = i_target - i_current;

            static float e_sum = 0;
            static uint8_t windup = 0;
            if (!windup)
            {
                e_sum += i_error;
            }
            float u = i_error * 9.5425f + e_sum * 2.3762f;
            if (u > 4.99f)
            {
                u = 4.99f;
                windup = 1;
            }
            else if (u < -4.99f)
            {
                u = -4.99f;
                windup = 2;
            }
            else
            {
                windup = 0;
            }

            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);
        }
    }
}

static void Position_thread(void *arg)
{
    //initialize
    int32_t enc_pnt=0;
    int32_t p_current=0;
    int32_t p_error=0;
    float i_target=0;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    int32_t p_cnt;
    uint8_t windup=0;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    //以当前位置为起点，读取88个脉冲，重置encoder脉冲计数

    while (1)
    {
        while(p_current<single_step)
        {
            //读取 ，计算偏差， 设置力矩
        }

        //重置encoder计数
        

        int32_t enc_cnt = 0;
        static int32_t enc_cnt_p = 0;
        static float p_target = 0.0f;
        float p_current = (float)enc_cnt * 0.1428f;
        float p_error = p_target - p_current;
        static float p_e_sum = 0.0f;
        static uint8_t p_windup = 0;
        if (p_error > 2 * 3.14159f * 6)
            p_target -= 2 * 3.14159f * 12;
        else if (p_error < -2 * 3.14159f * 6)
            p_target += 2 * 3.14159f * 12;
        if (!p_windup)
        {
            p_e_sum += p_error;
        }
//        if (p_error > 2 * 3.14159f || p_error < -2 * 3.14159f)
            v_target = p_error * 8.7379f + p_e_sum * 0.013016f;
//        else
//        {
//            v_target = 0;
//            p_e_sum = 0;
//        }
        xQueueSend(g_current_t_queue, &i_target, portMAX_DELAY);

        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
    }
}
