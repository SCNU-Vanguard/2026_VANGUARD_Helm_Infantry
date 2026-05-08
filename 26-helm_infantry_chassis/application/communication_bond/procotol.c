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
#include "buzzer.h"

uint32_t offline_offline = 0;

/* 485接收 */
void Serial_485_Receive_Control(void)
{
   if (RS485_Is_Online() == 0)
   {

      offline_offline++ ;
      if(offline_offline > 1000) //1s没有通信了，认为485掉线了，重置命令
      {
         Buzzer_Play(Warming_sound, 0);
         offline_offline = 0;//防止溢出
      }
		 
      chassis_cmd -> vx = 0.0f;
      chassis_cmd -> vy = 0.0f;
      chassis_cmd -> vw = 0.0f;
      chassis_cmd -> vw_follow = 0.0f;
      chassis_cmd -> chassis_mode = CHASSIS_STOP;

      gimbal_cmd -> target_angle_yaw = 0.0f;
      gimbal_cmd -> current_yaw_acc = 0.0f;
      gimbal_cmd -> gimbal_mode = GIMBAL_MODE_STOP;

      shoot_cmd -> shoot_frq = 0.0f;
      shoot_cmd -> auto_state = SHOOT_STOP;
      return;
   }
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
   gimbal_cmd -> current_angle_neck = uart2_rx_message.current_angle_neck;
  
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

