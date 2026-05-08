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

/******************************procotol处为联系各处的纽带，包含各种需要的头文件*****************************/

#include "procotol.h"
#include "INS.h"
#include "shoot.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"

#include "vofa.h"
#include "bmi088.h"
#include "INS.h"

#include "rs485.h"
#include "buzzer.h"
#include "robot_frame_init.h"


/* 485接收 */
void Serial_485_Receive_Control(void)
{
    if (RS485_Is_Online() == 0)
   {
      //Buzzer_Play(Err_sound, 0);
      gimbal_cmd->yaw_position = uart2_rx_message.yaw_position;
      return;
   }
    gimbal_cmd->yaw_position = uart2_rx_message.yaw_position;

}

void Serial_485_Send_Control(void)
{

    /*底盘参数*/
    uart2_tx_message.chassis_vx = chassis_cmd -> vx ;
    uart2_tx_message.chassis_vy = chassis_cmd -> vy ;
    uart2_tx_message.chassis_vw = chassis_cmd -> vw ;
    uart2_tx_message.chassis_mode = chassis_cmd -> move_mode ;

    // /*云台参数*/
		if( gimbal_cmd -> target_angle_yaw - gimbal_cmd -> current_angle_yaw > PI )
		{
				gimbal_cmd -> target_angle_yaw -= 2 * PI;
		}
		else if( gimbal_cmd -> target_angle_yaw - gimbal_cmd -> current_angle_yaw < -PI )
		{
				gimbal_cmd -> target_angle_yaw += 2 * PI;
		}
	
    uart2_tx_message.target_angle_yaw = gimbal_cmd -> target_angle_yaw ;
    uart2_tx_message.INS_YAW = INS.Yaw ;
    uart2_tx_message.INS_YAW_ACC = INS.Gyro[2];
    uart2_tx_message.gimbal_mode = gimbal_cmd -> gimbal_mode ;
    uart2_tx_message.neck_state  = gimbal_cmd -> neck_state  ;
	uart2_tx_message.current_angle_neck = gimbal_cmd -> current_angle_neck;

    /*射击参数*/
    uart2_tx_message.shoot_frq = shoot_cmd -> shoot_frq ;
    uart2_tx_message.shoot_mode = shoot_cmd -> auto_state ;

    /*UI标志*/
    if(rc_data->key[0].z == 1 )
    {
        uart2_tx_message.ui_refresh_key = 1;
    }
    else
    {
        uart2_tx_message.ui_refresh_key = 0;
    }
    
    

    uart2_send_data(&uart2_tx_message);
    uart2_transmit_control( );
}
