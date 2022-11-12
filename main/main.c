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

struct position_handle{
    int16_t angle;//设定转动角度
    char motor_position_status;//0:初始，1:正篇，2:负偏，4:无偏
    int16_t pos_error;//与目标位置的距离
    int16_t pos_target;
};
struct position_handle position_struct1;
struct position_handle position_struct2;
struct position_handle *position_struct;

struct class_handle{
    uint8_t num;
    char motor_position_status;//0:初始，1:正篇，2:负偏，4:无偏
    char motor_class_status;//0:初始，1:正篇，2:负偏，4:无偏
    int16_t class_range;
    int16_t half_class_range;
    int16_t class_cnt;//记录当前档位
    int16_t degree_cnt;//当前档位分度值
    int16_t enc_error;//与位置0的距离，用于划分当前的分区
};
struct class_handle class_struct1={3,0,0};
struct class_handle * class_struct;

    /* basic encoder */
    rotary_encoder_t *encoder = NULL;
    int16_t enc_cnt;//目前的脉冲数
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
    float i_current;
    float i_error;
    float i_offset;
    float u=0;

    #define ONE_ROUND 2156 //输出轴转过一周的脉冲数
    #define ONE_CLASS 2156 //一个档位的脉冲数量
    #define HALF_CLASS 100
    #define MAX_I 0.13f
    #define DELAY_MS 10
    #define PARA_ENCODER 14.28f //(DELAY_HZ)100*2pi/44 11线编码器

/* static void position_isr(void *arg); */
static void pwm_isr(void *arg);
/* static void current_thread(void *arg); */
static void power_thread(void *arg);
static void position_thread(struct position_handle *position_struct);//将encoder复位
static void class_thread(struct class_handle *class_struct);

void PysbMotorInitA();//初始化ESP32电机中断，GPIO，电流偏差等
void PysbMotorInit();
void PysbMotorInitB();//初始化数据结构体

inline void PysbMotorPositionSampling();
inline void PysbMotorSpeedSampling();
inline void PysbMotorPositionSet(struct position_handle * position_struct);
/* inline void PysbMotorSpeedSet(); */
void PysbMotorPositionControl(struct position_handle * position_struct);
inline void PysbMotorClassPositionSet(struct class_handle * class_struct);
void PysbMotorClassPositionControl(struct class_handle * class_struct);
void PysbMotorSpeedControl();

QueueHandle_t power_queue;
QueueHandle_t position_queue;
/* QueueHandle_t distence_queue; */
int32_t g_adc_offset; 

//demo
void restart();
void set_velocity(v1);//设置三档转速
void rotate();//转180和360
void stalls();//设置档位2，3，4
void ratchet_wheel();

inline void PysbMotorInit(){
    PysbMotorInitA();
    PysbMotorInitB();
}

inline void PysbMotorInitA(){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    for (uint8_t i = 0; i < 100; i++)
    {
        g_adc_offset += adc1_get_raw(ADC1_CHANNEL_7);
    }
    g_adc_offset /= 100;
    i_offset= g_adc_offset * 0.000634921f;
    {
    uint32_t pcnt_unit = 0;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 39, 36);
    rotary_encoder_new_ec11(&config, &encoder);
    encoder->set_glitch_filter(encoder, 1);
    encoder->start(encoder);
    }
   
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 22);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 23);

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
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    power_queue = xQueueCreate(1, sizeof(int32_t));
    position_queue = xQueueCreate(1, sizeof(float));

    MCPWM0.int_ena.val = MCPWM_TIMER0_TEZ_INT_ENA;
    mcpwm_isr_register(MCPWM_UNIT_0, pwm_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
}

inline void PysbMotorInitB(){
    struct position_handle position_struct1={
        .angle=180,
    };
}

void app_main(void)
{
    PysbMotorInit();

   /*  printf("\n =================================================================\n");
    printf(" |             Example of Motor Control                          |\n");
    printf(" |                                                               |\n");
    printf(" |  1. Try 'help', check all supported commands                  |\n");
    printf(" |  2. Try 'config' to set control period or pwm frequency       |\n");
    printf(" |  3. Try 'pid' to configure pid paremeters                     |\n");
    printf(" |  4. Try 'expt' to set expectation value and mode              |\n");
    printf(" |  5. Try 'motor' to start motor in several seconds or stop it  |\n");
    printf(" |                                                               |\n");
    printf(" =================================================================\n\n"); */
   /*  xTaskCreate(current_thread, "current_thread", 4095, NULL, 5, NULL); */
    
    /* position_struct=&position_struct1; */
    class_struct=&class_struct1;
    /* xTaskCreate(position_thread, "position_thread", 4095, position_struct, 5, NULL); */

    xTaskCreate(class_thread, "class_thread", 4095, class_struct, 5, NULL);

    xTaskCreate(power_thread, "power_thread", 4095, NULL, 5, NULL);
    printf("g_adc_offset:%d\n",g_adc_offset);
    while(1){
        printf("class.enc_error:%d\n",class_struct1.enc_error);
        printf("class.class_cnt:%d\n",class_struct1.class_cnt);
        printf("class.degree_cnt:%d\n",class_struct1.degree_cnt);
      
        printf("i_error:%f\n",i_error);

        printf("u:%f\n",u);
        vTaskDelay(1000/portTICK_RATE_MS);
        
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

/*--------------------------------------------------------------------------------------------------------*/
void position_thread(struct position_handle *position_struct){
    position_struct->pos_target=position_struct->angle*5.95277778f;
    position_struct->motor_position_status=0;
    while(1){
        PysbMotorPositionSampling();

        PysbMotorPositionSet(position_struct);

        PysbMotorPositionControl(position_struct);

        xQueueSend(position_queue, &i_target, portMAX_DELAY);
    }
}

void class_thread(struct class_handle *class_struct){
    (*class_struct).class_range=ONE_ROUND/(*class_struct).num;
    (*class_struct).half_class_range=(*class_struct).class_range/2;
    while(1){
        PysbMotorPositionSampling();
        PysbMotorClassPositionSet(class_struct);
        PysbMotorClassPositionControl(class_struct);
        xQueueSend(position_queue, &i_target, portMAX_DELAY);
    }
    
}

inline void PysbMotorPositionSampling(){
    enc_cnt=encoder->get_counter_value(encoder);
}

inline void PysbMotorPositionSet(struct position_handle * position_struct){
    (*position_struct).pos_error=enc_cnt-(*position_struct).pos_target;
    if((*position_struct).pos_error<=22&&(*position_struct).pos_error>-22){
        (*position_struct).motor_position_status=0;
    }
    else if((*position_struct).pos_error>22){
        (*position_struct).motor_position_status=1;
    }//正转
    else{
        (*position_struct).motor_position_status=2;
    }//enc_cnt<0，反转
}

//误差计算节点的切换，计算误差，设置档位和分度值
inline void PysbMotorClassPositionSet(struct class_handle * class_struct){

    /* (*class_struct).enc_error=(*class_struct).degree_cnt-(* class_struct).pos_target; */
    (*class_struct).enc_error=enc_cnt;
    if((* class_struct).enc_error==0){
        (*class_struct).motor_position_status=0;
    }
    else if((*class_struct).enc_error>0){
        (*class_struct).motor_position_status=1;
    }//pos正偏
    else{
        (*class_struct).motor_position_status=2;
    }//pos负偏

    int16_t _enc_error=(*class_struct).enc_error+(*class_struct).half_class_range;

    if((*class_struct).motor_position_status==4){
        (* class_struct).motor_class_status=4;
    }
    else if((*class_struct).motor_position_status==1){
        (* class_struct).motor_class_status=1;
        (* class_struct).class_cnt=_enc_error/(*class_struct).class_range;   
    }//正转
    else if((*class_struct).motor_position_status==2){
        (* class_struct).motor_class_status=2;
        (* class_struct).class_cnt=_enc_error/(*class_struct).class_range-1;
    }//enc_cnt<0，反转

    
    (* class_struct).degree_cnt=enc_cnt-(* class_struct).class_cnt*(*class_struct).class_range;
}

#define k1 0.0241f
#define k2 0.000000241f
void PysbMotorPositionControl(struct position_handle *position_struct){
    (*position_struct).pos_error=enc_cnt-(*position_struct).pos_target;
        if((*position_struct).pos_error>15&&(*position_struct).pos_error<=22){
            i_target=0;
        }
        else if((*position_struct).pos_error>0&&(*position_struct).pos_error<=15){
            i_target=0;;
        }
         else if((*position_struct).pos_error>22&&(*position_struct).pos_error<500){
            i_target=k2*(-(*position_struct).pos_error-22)+0.53;
        }
        else if((*position_struct).pos_error<15&&(*position_struct).pos_error>=-22){
            i_target=0;;
        }
        else if((*position_struct).pos_error<0&&(*position_struct).pos_error>=-15){
            i_target=0;
        }
        else if((*position_struct).pos_error<-22&&(*position_struct).pos_error>-500){
            i_target=k2*(-(*position_struct).pos_error+22)-0.53;
        }
        else if((*position_struct).pos_error==0){
            i_target=0;
        }
        else{
            i_target=(-((float)(*position_struct).pos_error/(float)HALF_CLASS))*1.5f;
        }
}

void PysbMotorClassPositionControl(struct class_handle * class_struct){

        if((* class_struct).degree_cnt>15&&(* class_struct).degree_cnt<=22){
            i_target=0;
        }
        else if((* class_struct).degree_cnt>0&&(* class_struct).degree_cnt<=15){
            i_target=0;;
        }
         else if((* class_struct).degree_cnt>22&&(* class_struct).degree_cnt<500){
            i_target=k2*(-(* class_struct).degree_cnt-22)+0.53;
        }
        else if((* class_struct).degree_cnt<15&&(* class_struct).degree_cnt>=-22){
            i_target=0;
        }
        else if((* class_struct).degree_cnt<0&&(* class_struct).degree_cnt>=-15){
            i_target=0;
        }
        else if((* class_struct).degree_cnt<-22&&(* class_struct).degree_cnt>-500){
            i_target=k2*(-(* class_struct).degree_cnt+22)-0.53;
        }
        else if((* class_struct).degree_cnt==0){
            i_target=0;
        }
        else{
            i_target=(-((float)(* class_struct).degree_cnt/(float)HALF_CLASS))*1.5f;
        }
}

/* ------------------------------------------------------------------------------------------------*/

inline void PysbMotorSpeedSampling(){
        enc_cnt = encoder->get_counter_value(encoder);
        v_current = (float)(enc_cnt - enc_cnt_0) * PARA_ENCODER; 
        enc_cnt = enc_cnt_0;
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
    //
    while (1) {
        if (xQueueReceive(power_queue, &raw_val, portMAX_DELAY) == pdTRUE)
        {
            xQueueReceive(position_queue,&i_target,portMAX_DELAY);
            i_current = (float)(raw_val) * 0.000634921f;	//raw_val to current(A);
            i_error = i_target - i_current;
            /* if(i_error<0&&i_error>-0.53f){i_error=-0.30;}
            if(i_error>0&&i_error<0.53){i_error=0.30;} */
            u =8*i_error;//P=9.5425f,I=2.3762
            if (u > 4.99f){
                u = 4.99f;
            }
            if (u < -4.99f){
                u = -4.99f;
            }//Attention! when doing position contorl Starting voltage of DC motor
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);

            vTaskDelayUntil(&xLastWakeTime, DELAY_MS/ portTICK_PERIOD_MS);
        }
    }
}