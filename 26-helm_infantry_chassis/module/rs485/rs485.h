#ifndef __RS485_H
#define __RS485_H

#include "main.h"

#define FRAME_HEADER 0xA5
#define FRAME_TAILER 0x5A

typedef struct
{
    uint8_t frame_header;
    
    //yaw轴数据
    float yaw_position;

    uint8_t frame_tailer;
    uint16_t crc16;
} __attribute__((packed)) Tx_packed_t;

typedef struct
{
    uint8_t frame_header;
    
    /*底盘参数*/
    float chassis_vx;
    float chassis_vy;
    float chassis_vw;// bit:0 / 1

    /*云台参数*/
    float target_angle_yaw; // rad
    float INS_YAW;  // rad
    float INS_YAW_ACC; // rad/s ,角加速度
    float current_angle_neck;
    uint8_t neck_state;

    /*射击参数*/
    float shoot_frq; // Hz
    
    /*运动模式*/
    uint8_t chassis_mode;//3
    uint8_t gimbal_mode;//3
    uint8_t shoot_mode; //0：停止，1：拨打

    /* 使能标志位 */ 
    uint8_t enable_flag;
		
		//ui绘制
    uint8_t ui_refresh_key;

    uint8_t frame_tailer;
    uint16_t crc16;
} __attribute__((packed)) Rx_packed_t;

extern Tx_packed_t uart2_tx_message;
extern Rx_packed_t uart2_rx_message;

extern uint8_t uart2_receive_buffer[sizeof(Rx_packed_t) * 3];
extern uint8_t uart2_transmit_buffer[sizeof(Tx_packed_t)];

extern uint8_t ready_to_transmit;
extern uint8_t ready_to_receive;

extern uint8_t uart2_status;
extern uint32_t last_uart2_uwTick;
extern uint8_t uart2_current_byte;
extern uint16_t uart2_buffer_length;

void RS485_Init(UART_HandleTypeDef *uart_handle);
uint8_t RS485_Is_Online(void);
void uart2_send_data(Tx_packed_t *data_p);
void uart2_transmit_control(void);
void uart2_start_receive(void);
void uart2_error_recover(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void uart2_online_check(void);

#endif
