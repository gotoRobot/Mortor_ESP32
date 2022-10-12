#include <stdio.h>
#include <stdint.h>
#define _cplusplus

#include "pysb_global.h"

#include "pysb_uart.h"
#include "pysb_gpio.h"
#include "pysb_motor.h"

#include "pysb_status.h"
#include "pysb_lb_comm.h"
#include "pysb_set_table.h"
#include "pysb_system.h"
#include "pysb_isr.h"
#include "pysb_log.h"

#ifdef BASE_ON_ESP
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif 

xQueueHandle START_BUTTON_QUEUE = NULL;
/* QueueHandle_t CURRENT_SAMPLE_QUEUE;
QueueHandle_t CURRENT_TARGET_QUEUE; */
rotary_encoder_t *encoder_handle=NULL;////

static void PysbInit(void ) {
    START_BUTTON_QUEUE = xQueueCreate(10, sizeof(uint32_t));
    PysbGpioInit();
    PysbUartInit();
    PysbMotorInit();
    PysbIsrInit();
}

static void PysbLbComm(void *arg) {
    //initialize log
    static const char *LB_COMM_TASK_TAG = "LbCommTask";
    ___EspLogLevelSet(LB_COMM_TASK_TAG, ESP_LOG_INFO);

    
    uint8_t* rx_data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    cmd_buffer_t cmd_buffer = CreateCmdBuffer();
    cmds_queue_t cmds_queue = CreateCmdsQueue();
    extern lb_cmd_status_t LB_COMM_STATUS;
    InitTable();
    uint32_t rx_bytes = 0;
    while (1) {
        rx_bytes = ___EspUartReadBytes(UART_LB, rx_data, RX_BUF_SIZE, RX_WAIT_TIME);
        if (rx_bytes > 0) {
            PysbLbCommRxProcess(rx_data,rx_bytes,&cmd_buffer,&cmds_queue);
            ___EspLogInfo(LB_COMM_TASK_TAG, "Read %d bytes: '%s'", rx_bytes, rx_data);
            ___EspLogBufferHexdump(LB_COMM_TASK_TAG, rx_data, rx_bytes, ESP_LOG_INFO);
        } else {
            //detele the first cmd that is send by start block
            while(!CmdsIsEmpty(&cmds_queue)){
                lb_cmd_t cmd;
                GetCmd(&cmds_queue,&cmd);
                if(cmd.source_id == SB_ID){ //detele the cmd which is send by start block
                    DeleteCmd(&cmds_queue);
                }else{
                    break;
                }
            }
            switch(LB_COMM_STATUS.lb_comm_status){
                case START:{
                    ___EspLogInfo("Write Pin","set pin low");
                    ___EspGpioSetLevel(ACTIVATE_LB_PIN, 0); //activate logic block
                    LB_COMM_STATUS.lb_comm_status = ADDR_NOTICE;
                    break;
                }
                case ADDR_NOTICE:{
                    if(LB_COMM_STATUS.activate_dir == DIR_END){
                        LB_COMM_STATUS.lb_comm_status = FINISHED;
                        break;
                    }
                    if(PysbTableAddrNotice() != NORMAL){
                        ___EspLogError("ADDR_NOTICE","address notice unsuccessfully!");
                    }else{
                        LB_COMM_STATUS.lb_comm_status = ADDR_NOTICE_REQUEST;
                    }
                    break;
                }
                case ADDR_NOTICE_REQUEST:{
                    if(PysbTableAddrNoticeRequest() != NORMAL){
                        ___EspLogError("ADDR_NOTICE_REQUEST","address notice request unsuccessfully!");
                    }else{
                        LB_COMM_STATUS.lb_comm_status = ADDR_NOTICE_WAIT;
                        PysbStartLbCommRqIsr();
                    }
                    break;
                }
                case ADDR_NOTICE_WAIT:{
                    if(CmdsIsEmpty(&cmds_queue)){
                        break;
                    }
                    if(PysbTableAddrNoticeWait(&cmds_queue) != NORMAL){
                        ___EspLogError("ADDR_NOTICE_WAIT","address notice wait unsuccessfully!");
                    }else{
                        PysbStopLbCommRqIsr();
                        LB_COMM_STATUS.lb_comm_status = ADDR_NOTICE_WAIT_OVER;
                    }
                    break;
                }
                case ADDR_NOTICE_WAIT_OVER:{
                    //print
                    LB_COMM_STATUS.lb_comm_status = ACTIVATE;
                    break;
                }
                case ACTIVATE:{
                    if(LB_COMM_STATUS.activate_dir == DIR_END){
                        LB_COMM_STATUS.lb_comm_status = FINISHED;
                        break;
                    }
                    if(PysbTableActivate() != NORMAL){
                        ___EspLogError("ACTIVATE","activate unsuccessfully!");
                    }else{
                        LB_COMM_STATUS.lb_comm_status = ACTIVATE_REQUEST;
                    }
                    break;
                }
                case ACTIVATE_REQUEST:{
                    if(PysbTableActivateRequest() != NORMAL){
                        ___EspLogError("ACTIVATE_REQUEST","activate request unsuccessfully!");
                    }else{
                        LB_COMM_STATUS.lb_comm_status = ACTIVATE_WAIT;
                        PysbStartLbCommRqIsr();
                    }
                    break;
                }
                case ACTIVATE_WAIT:{
                    if(CmdsIsEmpty(&cmds_queue)){
                        break;
                    }
                    if(PysbTableActivateWait(&cmds_queue) != NORMAL){
                        ___EspLogError("ACTIVATE_WAIT","activate wait unsuccessfully!");
                    }else{
                        PysbStopLbCommRqIsr();
                        LB_COMM_STATUS.lb_comm_status = ACTIVATE_WAIT_OVER;
                    }
                    break;
                }
                case ACTIVATE_WAIT_OVER:{
                    if(LB_COMM_STATUS.activate == ENTER_ACTIVATE){
                        LB_COMM_STATUS.lb_comm_status = ADDR_NOTICE;
                    }else{//LB_COMM_STATUS.activate == ACTIVATE_RIGHT or ACTIVATE_DOWN
                        LB_COMM_STATUS.lb_comm_status = ACTIVATE;
                    }
                    break;
                }
                case FINISHED:{
                    ___EspLogInfo("FINISHED","nice!");
                    PrintTable();
                    PysbDelayS(5);
                    break;
                }
                default:{
                    if(!CmdsIsEmpty(&cmds_queue)){
                        lb_cmd_t cmd;
                        GetCmd(&cmds_queue,&cmd);
                        ___EspUartWriteBytes(UART_LB,cmd.cmd,cmd.cmd_length);
                        ___EspLogBufferHexdump("write byte", cmd.cmd, cmd.cmd_length, ESP_LOG_INFO);
                        DeleteCmd(&cmds_queue);
                    }
                }
            }
        }
    }
    free(rx_data); 
    DestoryCmdBuffer(&cmd_buffer);
}

void test(){
    lb_cmd_t cmd = CreateLbCmd();
    static const char *LB_COMM_TASK_TAG = "LbCommTask";
    ___EspLogLevelSet(LB_COMM_TASK_TAG, ESP_LOG_INFO);
    ___EspLogInfo(LB_COMM_TASK_TAG, "cmd: %d", cmd.opcode);
    while(true){
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    sb_status_t sb_status = INIT;
    // test();
    PysbInit();
    xTaskCreate(CurrentThread, "CurrentThread", 4095, NULL, 5, NULL);
    xTaskCreate(SpeedThread, "SpeedThread", 4095, NULL, 4, NULL);
    while(sb_status == INIT){
        uint8_t r = 0;
        xQueueReceive(START_BUTTON_QUEUE, &r, portMAX_DELAY);
        if(r == BUTTON){
            sb_status = START_UP;
        }
    }
    //remember give enough space for pysblbcomm task! 
    xTaskCreate(PysbLbComm,"PysbLbComm",24576*2,NULL,configMAX_PRIORITIES, NULL);
} 