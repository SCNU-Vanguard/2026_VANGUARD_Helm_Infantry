/**
******************************************************************************
 * @file    procotol.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/******************************procotol处为联系各处的纽带，包含各种需要的头文件*****************************/

#include "procotol.h"
#include "robot_frame_init.h"

#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"

#include "rs485.h"



/* 485接收 */
void Serial_485_Receive_Control(void)
{
    /*底盘参数*/
    chassis_cmd -> vx = uart2_rx_message.chassis_vx;
    chassis_cmd -> vy = uart2_rx_message.chassis_vy;
    chassis_cmd -> vw = uart2_rx_message.chassis_vw;
    chassis_cmd -> chassis_mode = uart2_rx_message.chassis_mode;

            /*云台参数*/
    gimbal_cmd -> target_angle_yaw = uart2_rx_message.target_angle_yaw;
    gimbal_cmd -> current_angle_yaw = uart2_rx_message.INS_YAW;
    gimbal_cmd -> current_yaw_acc = uart2_rx_message.INS_YAW_ACC;
    gimbal_cmd -> gimbal_mode = uart2_rx_message.gimbal_mode;
  
    /*射击参数*/
    shoot_cmd -> shoot_frq = uart2_rx_message.shoot_frq;
    shoot_cmd -> mode = uart2_rx_message.shoot_mode ;
}

void Serial_485_Send_Control(void)
{
    // uart2_tx_message.shooter_heat_limit = referee_data->Game_Robot_state.shooter_barrel_heat_limit ;//枪口热量上限
    // uart2_tx_message.shooter_heat = referee_data->Power_Heat_Data.shooter_17mm_1_barrel_heat;//枪口热量
    //uart2_send_data(&uart2_tx_message);
    uart2_transmit_control( );
}

