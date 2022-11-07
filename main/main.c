#include <stdio.h>
#include <stdlib.h>
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
#include <math.h>

/* static void position_isr(void *arg); */
static void pwm_isr(void *arg);
/* static void current_thread(void *arg); */
static void power_thread(void *arg);
static void position_thread(void *arg);

QueueHandle_t power_queue;
QueueHandle_t position_queue;
/* QueueHandle_t distence_queue; */
int32_t g_adc_offset;

struct position_handle{
    int8_t class_cnt;//记录当前档位
    int8_t degree_cnt;//当前档位分度值
    int16_t distence_cnt;//与目标位置的距离
    int16_t pos_target;//设定转动角度
    char motor_position_status;//0:无偏，1:正篇，2:负偏
};
    struct position_handle cube;
    struct position_handle impulse;

    /* basic encoder */
    rotary_encoder_t *encoder = NULL;
    int16_t enc_cnt_1;//目前的脉冲数
    int16_t enc_cnt_0;//上一个速度周期脉冲数
    char clockwise;//旋转方向

    char motor_speed_status=0;//0:静止，1:正转，2:反转，3:停止，4:最大正转，5:最大反转
    float v_current=0;
    float v_target=0;
    float v_error=0;
    float e_sum=0;
    #define PID_P_V 0.0049797f
    #define PID_I_V 0.00044505f  //0.0445052390679643*1e-2
    #define MAX_V 1000.0f
    #define MIN_V 50//只在速度足够大才控制电流r_handle

    int32_t g_adc_offset;
    float i_target;

        #define ONE_ROUND 2156 //输出轴转过一周的脉冲数
        #define ONE_CLASS 2156 //一个档位的脉冲数量
        #define HALF_CLASS 100
        #define MAX_I 0.13f
        #define DELAY_MS 10
        #define PARA_ENCODER 14.28f //(DELAY_HZ)100*2pi/44 11线编码器

inline void PysbMotorPositionSampling();
inline void PysbMotorSpeedSampling();
inline void PysbMotorPositionSet();
/* inline void PysbMotorSpeedSet(); */
void PysbMotorPositionControl();
void PysbMotorSpeedControl();

void app_main(void)
{
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11));
    for (uint8_t i = 0; i < 50; i++)
    {
        g_adc_offset += adc1_get_raw(ADC1_CHANNEL_7);
    }
    g_adc_offset /= 50;
    {
    uint32_t pcnt_unit = 0;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 39, 36);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));
    ESP_ERROR_CHECK(encoder->start(encoder));
    }
   
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 22));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 23));

    MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_b_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_a_shdw_full = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_b_shdw_full = 1;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 4000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode =  MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));

    power_queue = xQueueCreate(1, sizeof(int32_t));
    position_queue = xQueueCreate(1, sizeof(float));
    /* distence_queue = xQueueCreate(1, sizeof(int32_t)); */

    MCPWM0.int_ena.val = MCPWM_TIMER0_TEZ_INT_ENA;
    ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, pwm_isr, NULL, ESP_INTR_FLAG_IRAM, NULL));
/*     ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, position_isr, NULL, ESP_INTR_FLAG_IRAM, NULL)); */
   /*  xTaskCreate(current_thread, "current_thread", 4095, NULL, 5, NULL); */
   xTaskCreate(position_thread, "position_thread", 4095, NULL, 5, NULL);
   /* xTaskCreate(display_thread, "display_thread", 4095, NULL, 5, NULL); */
    xTaskCreate(power_thread, "power_thread", 4095, NULL, 5, NULL);    
    printf("g_adc_offset:%d\n",g_adc_offset);
    while(1){
        printf("cube.distence_cnt:%d\n",cube.distence_cnt);
        vTaskDelay(1000/portTICK_RATE_MS);
        /* gpio_set_levl(GPIO_OUTPUT_IO_0,cube.distence_cnt) */
    }
}

static void IRAM_ATTR pwm_isr(void *arg)
{
    uint32_t mcpwm_intr_status;
    int32_t raw_val;
    mcpwm_intr_status = MCPWM0.int_st.val;
    BaseType_t high_task_awoken = pdFALSE;
    if (mcpwm_intr_status & MCPWM_TIMER0_TEZ_INT_ENA)
    {
        raw_val = adc1_get_raw(ADC1_CHANNEL_7);
        xQueueSendFromISR(power_queue, &raw_val, &high_task_awoken);
    }
    MCPWM0.int_clr.val = mcpwm_intr_status;
    portYIELD_FROM_ISR(high_task_awoken);
}

/* 输入一个角度，返回一个目标脉冲数 */
int16_t position_set(float angle){
    cube.pos_target=angle*5.95277778+493;
    return cube.pos_target;
}

void position_thread(void *arg){
    cube.pos_target=position_set(180);
    while(motor_speed_status!=2){
        PysbMotorPositionSampling();

        PysbMotorPositionSet();

        PysbMotorPositionControl();

        xQueueSend(position_queue, &i_target, portMAX_DELAY);
    }
}

inline void PysbMotorInit(){

}

inline void PysbMotorSpeedSampling(){
        enc_cnt_1 = encoder->get_counter_value(encoder);
        v_current = (float)(enc_cnt_1 - enc_cnt_0) * PARA_ENCODER; 
        enc_cnt_1 = enc_cnt_0;
        switch ((int)v_current)
        {
        case 1:
            motor_speed_status=1;
            break;
        case 2:
            motor_speed_status=2;
            break;
        case 3:
            motor_speed_status=3;
            break;
        default:
            break;
        }
}

inline void PysbMotorPositionSampling(){
    enc_cnt_1=encoder->get_counter_value(encoder);
}

inline void PysbMotorPositionSet(){
    cube.distence_cnt=enc_cnt_1-cube.pos_target;
    if(enc_cnt_1==0){
        cube.motor_position_status=0;
    }
    else if(enc_cnt_1>0){
        cube.motor_position_status=1;
        cube.class_cnt=enc_cnt_1/ONE_CLASS;
    }//正转
    else{
        cube.motor_position_status=2;
        cube.class_cnt=enc_cnt_1/ONE_CLASS-1;
    }//enc_cnt_1<0，反转
    
    cube.degree_cnt=enc_cnt_1-cube.class_cnt*ONE_CLASS;
}

void PysbMotorPositionControl(){
    cube.distence_cnt=enc_cnt_1-cube.pos_target;
        if(cube.distence_cnt>0){
            i_target=pow(((float)cube.distence_cnt/(float)HALF_CLASS),3)*0.3f;
        }
        else if(cube.distence_cnt<0){
            i_target=pow(((float)cube.distence_cnt/(float)HALF_CLASS),3)*0.3f;
        }
        else{
            i_target=0;
        }
}

void PysbMotorSpeedControl(){
    v_error = v_target - v_current;
    if (v_error < -50.0f){
        v_target += 40.0f;
    }
    else if (v_error > 50.0f){
        v_target -= 40.0f;
    }//refresh v_target

    if (v_target > MAX_V){
        v_target = MAX_V;
    }
    else if (v_target < -MAX_V){
        v_target = -MAX_V;
    }

        if (!motor_speed_status)
        {
            e_sum += v_error;
        }
        if (v_target > MIN_V || v_target < -MIN_V)//we only control current if velocity is high enough
            i_target = v_error * PID_P_V + e_sum * PID_I_V;
        else
        {
            i_target = 0;
            e_sum = 0;
        }

        //设定最大电流，同时也检测v_error是否反向了
        if (i_target >= MAX_I){
            i_target = MAX_I;
        } //rotating B direction
        if (i_target <= -MAX_I) {
            i_target = -MAX_I;
        }
}


void power_thread(void *arg)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    float raw_val;
    float i_target;
    float u=0;//
    while (1) {
        if (xQueueReceive(power_queue, &raw_val, portMAX_DELAY) == pdTRUE)
        {
            xQueueReceive(position_queue,&i_target,portMAX_DELAY);
            float i_current = (float)(raw_val - g_adc_offset) * 0.000634921f;	//raw_val to current(A);
            float i_error = i_target - i_current;

            u = i_error *8;//P=9.5425f,I=2.3762
            if (u > 4.99f){
                u = 4.99f;
            }
            if (u < -4.99f){
                u = -4.99f;
            }

            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);

            vTaskDelayUntil(&xLastWakeTime, DELAY_MS/ portTICK_PERIOD_MS);
        }
    }
}