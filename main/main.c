#define BASE_ON_ESP32
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

//宏定义常量
#define ROTARY_ENCODER_PINA 39
#define ROTARY_ENCODER_PINB 36
#define MCPWM0A_PIN 22
#define MCPWM0B_PIN 23
#define PID_P_I 9.5425f  
#define PID_I_I 2.3762f  //47523.4539552618*5e-5
#define PID_P_v 0.0049797f
#define PID_I_V 0.00044505f  //0.0445052390679643*1e-2
#define MAX_U 4.99f
#define MAX_I 0.13f
#define MAX_V 700.0f
#define MIN_V 70//只在速度足够大才控制电流
#define PARA_ENCODER 14.28f //(DELAY_HZ)100*2pi/44 11线编码器
#define DELAY_MS 10
#define SAMPLING_R 0.000634921f
#define PWM_HZ 40000

//FreeRtos相关线程和中断程序
static void PwmIsr(void *arg);
static void CurrentThread_(void *arg);
static void SpeedThread_(void *arg);
QueueHandle_t g_current_queue;//电流采样值的序列
QueueHandle_t g_current_t_queue;//目标电流值

//ENCODER相关配置
int32_t g_adc_offset;//采样偏差
rotary_encoder_t *g_encoder = NULL;//rotary encoder handle

#define ___EspAdc1ConfigWidth adc1_config_width
#define ___EspAdc1ConfigChannelAtten adc1_config_channel_atten
#define ___EspAdc1GetRaw adc1_get_raw
#define ___EspRotaryEncoderNewEc11 rotary_encoder_new_ec11
#define ___EspMcpwmGpioInit mcpwm_gpio_init
#define ___EspMcpwmInit mcpwm_init
#define ___EspMcpwmIsrRegister mcpwm_isr_register
#define ___EspMcpwmSetDuty mcpwm_set_duty

inline void PysbMotorInit(){
/*     ESP_ERROR_CHECK(gpio_reset_pin(GPIO_NUM_21));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT)); */
    #ifdef BASE_ON_ESP32
    //配置电流采样adc接口
    ESP_ERROR_CHECK(___EspAdc1ConfigWidth(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(___EspAdc1ConfigChannelAtten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11));
    ///测定基础电流,取50次取平均值足够了
    for (uint8_t i = 0; i < 50; i++)
    {
        g_adc_offset += ___EspAdc1GetRaw(ADC1_CHANNEL_7);
    }
    g_adc_offset /= 50;

    //配置电机编码器
    uint32_t pcnt_unit = 0;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, ROTARY_ENCODER_PINA, ROTARY_ENCODER_PINB);
    ESP_ERROR_CHECK(___EspRotaryEncoderNewEc11(&config, &g_encoder));
    ESP_ERROR_CHECK(g_encoder->set_glitch_filter(g_encoder, 1));
    ESP_ERROR_CHECK(g_encoder->start(g_encoder));

    ESP_ERROR_CHECK(___EspMcpwmGpioInit(MCPWM_UNIT_0, MCPWM0A, MCPWM0A_PIN));
    ESP_ERROR_CHECK(___EspMcpwmGpioInit(MCPWM_UNIT_0, MCPWM0B, MCPWM0B_PIN));

    //设置为PWM更新模式为延迟
    {
        MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 1;
        MCPWM0.operators[0].gen_stmp_cfg.gen_b_upmethod = 1;
        MCPWM0.operators[0].gen_stmp_cfg.gen_a_shdw_full = 1;
        MCPWM0.operators[0].gen_stmp_cfg.gen_b_shdw_full = 1;
    }

    mcpwm_config_t pwm_config;
    {
        pwm_config.frequency = PWM_HZ;
        pwm_config.cmpr_a = 50.0f;
        pwm_config.cmpr_b = 50.0f;
        pwm_config.counter_mode =  MCPWM_UP_DOWN_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_1; 
    }
    ESP_ERROR_CHECK(___EspMcpwmInit(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config)); 
    #endif   
}

void app_main(void)
{
    PysbMotorInit();

    g_current_queue = xQueueCreate(1, sizeof(int32_t));
    g_current_t_queue = xQueueCreate(1, sizeof(float));

    MCPWM0.int_ena.val = MCPWM_TIMER0_TEZ_INT_ENA;
    ESP_ERROR_CHECK(___EspMcpwmIsrRegister(MCPWM_UNIT_0, PwmIsr, NULL, ESP_INTR_FLAG_IRAM, NULL));

    xTaskCreate(CurrentThread_, "CurrentThread_", 4095, NULL, 5, NULL);
    xTaskCreate(SpeedThread_, "SpeedThread_", 4095, NULL, 4, NULL);
}
/**
 * @brief transport 
 * 
 * @param void
 */
static void IRAM_ATTR PwmIsr(void *arg)
{
    uint32_t mcpwm_intr_status;
    int32_t raw_val;
    mcpwm_intr_status = MCPWM0.int_st.val;
    BaseType_t high_task_awoken = pdFALSE;
    if (mcpwm_intr_status & MCPWM_TIMER0_TEZ_INT_ENA)
    {
        /* gpio_set_level(GPIO_NUM_21, 1); */
        raw_val = adc1_get_raw(ADC1_CHANNEL_7);
        xQueueSendFromISR(g_current_queue, &raw_val, &high_task_awoken);
    }
    MCPWM0.int_clr.val = mcpwm_intr_status;
    portYIELD_FROM_ISR(high_task_awoken);
}

/**
 * @brief 接收电流信号后改变占空比
 * 
 * @param arg 
 */
static void CurrentThread_(void *arg)
{
    int32_t raw_val;
    float i_target;
    while (1)
    {
        if (xQueueReceive(g_current_queue, &raw_val, portMAX_DELAY) == pdTRUE)
        {
            /* gpio_set_level(GPIO_NUM_21, 0); */
            xQueueReceive(g_current_t_queue, &i_target, 0);

            float i_current = (float)(raw_val - g_adc_offset) * SAMPLING_R;	//raw_val to current(A);
            float i_error = i_target - i_current;

            static float e_sum = 0;
            static uint8_t windup = 0;
            if (!windup)
            {
                e_sum += i_error;
            }
            float u = i_error * PID_P_I + e_sum * PID_I_I;
            if (u > MAX_U)
            {
                u = MAX_U;
                windup = 1;
            }
            else if (u < -MAX_U)
            {
                u = -MAX_U;
                windup = 2;
            }//方向B(逆时针)
            else
            {
                windup = 0;
            }

            ___EspMcpwmSetDuty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            ___EspMcpwmSetDuty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);
        }
    }
}

/**
 * @brief 测速，设定速度，计算并发送目标电流值
 * 
 * @param arg 
 */
static void SpeedThread_(void *arg)
{
    while (1)
    {
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();

        //设置初始状态
        int32_t enc_cnt = 0;
        static int32_t enc_cnt_p = 0;
        float v_current = 0;
        static float v_target = 0.0f;
        float i_target = 0.0f;

        enc_cnt = g_encoder->get_counter_value(g_encoder);
        v_current = (float)(enc_cnt - enc_cnt_p) * PARA_ENCODER; 
        enc_cnt_p = enc_cnt;
        float v_error = v_target - v_current;//计算速度误差

        static float e_sum = 0;
        static uint8_t windup = 0;
        if (!windup)
        {
            e_sum += v_error;
        }
        if (v_target > MIN_V || v_target < -MIN_V)//只在速度足够大才控制电流
            i_target = v_error * PID_P_v + e_sum * PID_I_V;
        else
        {
            i_target = 0;
            e_sum = 0;
        }

       if (i_target >= MAX_I)
        {
            i_target = MAX_I;
            if (v_error > 0)
                windup = 1;
            else
                windup = 0;
        } //方向B(逆时针)
        else if (i_target <= -MAX_I)
        {
            i_target = -MAX_I;
            if (v_error < 0)
                windup = 2;
            else
                windup = 0;
        }
        else
        {
            windup = 0;
        }

        //检测到旋钮被转动，并进行目标转速的设定
        if (v_error < -50.0f)
        {
            v_target += 40.0f;
//			v_target = v_current;
        }
        else if (v_error > 50.0f)
        {
            v_target -= 40.0f;
//			v_target = v_current;
        }

        if (v_target > MAX_V)
        {
            v_target = MAX_V;
        }
        else if (v_target < -MAX_V)
        {
            v_target = -MAX_V;
        }

        xQueueSend(g_current_t_queue, &i_target, portMAX_DELAY);//发送目标电流

        vTaskDelayUntil(&xLastWakeTime, DELAY_MS/ portTICK_PERIOD_MS);
    }
}