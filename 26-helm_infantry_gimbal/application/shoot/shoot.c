/**
******************************************************************************
 * @file    shoot.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>
//#include "FreeRTOS.h"
//#include "task.h"
#include "cmsis_os2.h"

#include "shoot.h" 
#include "shoot_motor.h"

#include "robot_frame_config.h"
#include "robot_frame_init.h"

shoot_motor_instance_t *friction_motor[3];
static shoot_cmd_t shoot_cmd_storage;
shoot_cmd_t *shoot_cmd = &shoot_cmd_storage;//静态初始化，防止空指针

uint8_t fire_enable_flag = 0;//射击使能标志位
uint8_t Shoot_Mode_Change_Flag = 0;//射击模式切换标志位
uint8_t shoot_ready_flag = 0;//射击准备就绪标志位

uint32_t shoot_start_timer = 0;//用于计算摩擦轮开转时间


PID_t friction_angle_pid = {
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 480.0f,
    .output_limit = 10.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t friction_speed_pid = {
    .kp = 400.0f,
    .ki = 400.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f,
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t friction_motor_init = {
    .controller_param_init_config = {
        .angle_PID = &friction_angle_pid,
        .speed_PID = &friction_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = OTHER,//snail

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x011,
    },
};

void Shoot_Enable(void)
{
    for(int i = 0; i <3; i++)
    {
        Shoot_Motor_Enable(friction_motor[i]);
    }
}

void Shoot_Disable(void)
{
    for(int i = 0; i <3; i++)
    {
        Shoot_Motor_Stop(friction_motor[i]);
    }
}

void Shoot_Init(void)
{
    for(int i = 0;i < 3;i++)
	{
		friction_motor_init.can_init_config.tx_id = 0x01 + i;
        friction_motor_init.can_init_config.rx_id = 0x011 + i;
		friction_motor[i] = Shoot_Motor_Init(&friction_motor_init);
	}
}

void Shoot_Set_All_Friction(int16_t speed)
{
	for(int i = 0; i <3; i++)
    {
        Shoot_Motor_SetTar(friction_motor[i],speed);
    }
}

void Shoot_Observer( )
{
    ;
}

// 处理异常
void Shoot_Handle_Exception( )
{
    ;
}

// 设置射击模式
void Shoot_Set_Mode( )
{
    //模式切换
    if( (rc_data->rc.gear_shift == STOP_MODE && key_flag == 0) || keyboard_enable_flag == 0 )
    {
        shoot_cmd -> mode = SHOOT_MODE_STOP;
    }

    else if(rc_data->rc.gear_shift == REMOTE_MODE && key_flag == 0)
    {
        shoot_cmd -> mode = SHOOT_MODE_REMOTE;
    }

    else if(rc_data->rc.gear_shift == KEYBOARD_MODE || ROBOT_FRAME_KEYBOARD_ENABLED() )
    {
        shoot_cmd -> mode = SHOOT_MODE_KEYBOARD;//键鼠模式,默认使能
    }

    //扳机键，开启摩擦轮
    if( shoot_cmd -> mode == SHOOT_MODE_REMOTE )
    {
        shoot_cmd -> auto_state = rc_data->rc_rise_count[VT03_RC_RISE_TRIGGE_KEY] % 3;//模式切换,决定手打还是自瞄
    }
    else if( shoot_cmd -> mode == SHOOT_MODE_KEYBOARD )
    {
        //键鼠模式下，按住鼠标左键开火
        if( rc_data->mouse.press_l == 1 )
        {
            shoot_cmd -> auto_state = SHOOT_MANUAL;
        }
        else
        {
            shoot_cmd -> auto_state = SHOOT_STOP;
        }
    }

}
// 设置目标量
void Shoot_Reference( )
{
    ;
}

// 计算控制量
void Shoot_Console( )
{
    switch(shoot_cmd->mode)
    {
        case SHOOT_MODE_STOP:
                shoot_cmd -> shoot_v = 0;
                shoot_cmd -> shoot_frq = 0;
            Shoot_Disable();
        break;

        case SHOOT_MODE_REMOTE:
        {
            if(fire_enable_flag == 1 && shoot_cmd->auto_state != SHOOT_STOP)/////
            {
                Shoot_Enable();
                shoot_cmd -> shoot_v = SHOOT_V;
                Shoot_Set_All_Friction(shoot_cmd -> shoot_v);//设置目标值


                if(friction_motor[0] -> receive_flag == 0xA5 || friction_motor[1] -> receive_flag == 0xA5 || friction_motor[2] -> receive_flag == 0xA5)//摩擦轮开转后再给拨弹盘设置转速
                {
                    if(Shoot_Mode_Change_Flag == 1)
                    {
                    	Shoot_Mode_Change_Flag = 0;
                    	shoot_start_timer = osKernelGetTickCount();  // 记录当前时间
                    	shoot_ready_flag = 0;
                    }
                    if(osKernelGetTickCount() - shoot_start_timer >= 5000)// 判断是否已经等待满 3s
                    {
                    	shoot_ready_flag = 1;//3秒到，可以发射
                    }

                    if( shoot_ready_flag == 1 )
                    {
                        shoot_cmd -> shoot_frq = rc_data->rc.dial * REMOTE_SHOOT_SENSITIVITY;//Hz
                    }

                    // else if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_KEYBOARD && rc_data->mouse.press_l == 1 )//键鼠模式下，按住鼠标左键开火
                    // {
                    //     shoot_cmd -> shoot_frq = 10.0f;//Hz,键鼠模式暂定为10Hz
                    // }
                }

            }
            else
            {
                Shoot_Mode_Change_Flag = 1;
                shoot_cmd -> shoot_v = 0;
                shoot_cmd -> shoot_frq = 0;
                Shoot_Disable();
            }
            break;
        }

        case SHOOT_MODE_KEYBOARD:
        {
            if(fire_enable_flag == 1 )/////
            {
                Shoot_Enable();
							
							  //if( rc_data->key[0].g == 1 )
									
                shoot_cmd -> shoot_v = SHOOT_V;
                Shoot_Set_All_Friction(shoot_cmd -> shoot_v);//设置目标值


                if(friction_motor[0] -> receive_flag == 0xA5 || friction_motor[1] -> receive_flag == 0xA5 || friction_motor[2] -> receive_flag == 0xA5)//摩擦轮开转后再给拨弹盘设置转速
                {
                    if(Shoot_Mode_Change_Flag == 1)
                    {
                    	Shoot_Mode_Change_Flag = 0;
                    	shoot_start_timer = osKernelGetTickCount();  // 记录当前时间
                    	shoot_ready_flag = 0;
                    }
                    if(osKernelGetTickCount() - shoot_start_timer >= 5000)// 判断是否已经等待满 3s
                    {
                    	shoot_ready_flag = 1;//3秒到，可以发射
                    }

                    if( shoot_cmd -> auto_state == SHOOT_MANUAL && shoot_ready_flag == 1)//键鼠模式下，按住鼠标左键开火
                    {
                        shoot_cmd -> shoot_frq = 10.0f;//Hz,键鼠模式暂定为10Hz
                    }
                    else if( shoot_cmd -> auto_state == SHOOT_AUTO && shoot_ready_flag == 1)//自瞄模式
                    {
                        shoot_cmd -> shoot_frq = 10.0f;//Hz,自瞄模式暂定为10Hz
                    }
                    else
                    {
                        shoot_cmd -> shoot_frq = 0.0f;
                    }
                }

            }
            else
            {
                Shoot_Mode_Change_Flag = 1;
                shoot_cmd -> shoot_v = 0;
                shoot_cmd -> shoot_frq = 0;
                Shoot_Disable();
            }

            break;
        }   

        default:
        {
            shoot_cmd -> shoot_v = 0;
            shoot_cmd -> shoot_frq = 0;
            Shoot_Disable();
            break;
        }
            
        
    }

}

// 发送控制量
void Shoot_Send_Cmd( )
{
    Shoot_Motor_Send();
}

