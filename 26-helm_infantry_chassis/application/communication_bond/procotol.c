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
   chassis_cmd -> vx = USER_LIMIT_BORDER(uart2_rx_message.chassis_vx , 1.0f);
   chassis_cmd -> vy = USER_LIMIT_BORDER(uart2_rx_message.chassis_vy , 1.0f);
   chassis_cmd -> vw = USER_LIMIT_BORDER(uart2_rx_message.chassis_vw , 1.0f);
	  
    if(uart2_rx_message.chassis_mode < 3)
    {
        chassis_cmd -> chassis_mode = uart2_rx_message.chassis_mode;
    }
			

//    /*云台参数*/
   gimbal_cmd -> target_angle_yaw = USER_LIMIT_BORDER(uart2_rx_message.target_angle_yaw , 2 * PI) ;
   gimbal_cmd -> current_angle_yaw = USER_LIMIT_BORDER(uart2_rx_message.INS_YAW , 2 * PI);
   gimbal_cmd -> current_yaw_acc = uart2_rx_message.INS_YAW_ACC;
   gimbal_cmd -> gimbal_mode = uart2_rx_message.gimbal_mode;
   gimbal_cmd -> neck_state = uart2_rx_message.neck_state;
  
//    /*射击参数*/
   shoot_cmd -> shoot_frq = uart2_rx_message.shoot_frq;
   shoot_cmd -> auto_state = uart2_rx_message.shoot_mode ;

}

void Serial_485_Send_Control(void)
{
    // uart2_tx_message.shooter_heat_limit = referee_data->Game_Robot_state.shooter_barrel_heat_limit ;//枪口热量上限
    // uart2_tx_message.shooter_heat = referee_data->Power_Heat_Data.shooter_17mm_1_barrel_heat;//枪口热量
    //uart2_send_data(&uart2_tx_message);
    uart2_tx_message.yaw_position = gimbal_dm6006->receive_data.position;

    uart2_transmit_control( );
}

