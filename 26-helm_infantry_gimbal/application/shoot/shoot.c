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
#include "vesc_motor.h"

#include "robot_frame_config.h"
#include "robot_frame_init.h"

VESC_motor_instance_t *friction_motor[3];
static shoot_cmd_t shoot_cmd_storage;
shoot_cmd_t *shoot_cmd = &shoot_cmd_storage;//闈欐€佸垵濮嬪寲锛岄槻姝㈢┖鎸囬拡

uint8_t fire_enable_flag = 0;//拨弹标志位
uint8_t manual_shoot_flag = 0 ;


motor_init_config_t friction_motor_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = NULL,
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
        .outer_loop_type = OPEN_LOOP,
        .close_loop_type = OPEN_LOOP,

        .control_button = TORQUE_DIRECT_CONTROL,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = OTHER,//snail

    .can_init_config = {
        .can_handle = &hfdcan3,
        .tx_id = 0x01,
        .rx_id = 0x901,
    },
};

void Shoot_Enable(void)
{
    for(int i = 0; i <3; i++)
    {
        friction_motor[i]->motor_state_flag = MOTOR_ENABLE;
    }
}

void Shoot_Disable(void)
{
    for(int i = 0; i <3; i++)
    {
        friction_motor[i]->transmit_data.target_speed_rpm = 0;
        friction_motor[i]->motor_state_flag = MOTOR_DISABLE;
        Motor_ExCan_Crtl(friction_motor[i]);
    }
}

void Shoot_Init(void)
{
    for(int i = 0;i < 3;i++)
	{
        friction_motor_init.can_init_config.tx_id = 0x01 + i;
        friction_motor_init.can_init_config.rx_id = 0x901 + i;
		friction_motor[i] = VESC_Motor_Init(&friction_motor_init);
        friction_motor[i]->command_number = CAN_PACKET_SET_RPM;
	}
}

void Shoot_Set_All_Friction(int16_t speed)
{
	for(int i = 0; i <3; i++)
    {
        friction_motor[i]->transmit_data.target_speed_rpm = speed;
    }
}

static uint8_t Shoot_VESC_Is_Ready(void)
{
    for(int i = 0; i <3; i++)
    {
        if(friction_motor[i] == NULL || friction_motor[i]->receive_data.Speed_RPM <= 3000)
        {
            return 0U;
        }
    }

    return 1U;
}

void Shoot_Observer( )
{
    ;
}


void Shoot_Handle_Exception( )
{
    ;
}


void Shoot_Set_Mode( )
{
    //妯″紡鍒囨崲
    if( (rc_data->rc.gear_shift == STOP_MODE && key_flag == 0) || ( keyboard_enable_flag == 0 && key_flag == 1) )
    {
        shoot_cmd -> mode = SHOOT_MODE_STOP;
    }

    else if(rc_data->rc.gear_shift == REMOTE_MODE && key_flag == 0)
    {
        shoot_cmd -> mode = SHOOT_MODE_REMOTE;
    }

    else if(rc_data->rc.gear_shift == KEYBOARD_MODE || ROBOT_FRAME_KEYBOARD_ENABLED() )
    {
        shoot_cmd -> mode = SHOOT_MODE_KEYBOARD;
    }


    if( shoot_cmd -> mode == SHOOT_MODE_REMOTE )
    {
        if(fire_enable_flag == 1)
        {
            shoot_cmd -> auto_state = rc_data->rc_rise_count[VT03_RC_RISE_TRIGGE_KEY] % 3;
        }
        else 
        {
            shoot_cmd -> auto_state = 0 ;
        }
    }
    else if( shoot_cmd -> mode == SHOOT_MODE_KEYBOARD )
    {

        if( rc_data->mouse.press_l == 1 )
        {
            shoot_cmd -> auto_state = SHOOT_MANUAL;
        }
        else
        {
            shoot_cmd -> auto_state = SHOOT_STOP;
        }

        if( rc_data->key[0].r== 1 )
        {
            shoot_cmd -> friction_state = FRICTION_START;//开启摩擦轮
        }
        else if( rc_data->key[0].g == 1 )//关闭摩擦轮
        {
            shoot_cmd -> friction_state = FRICTION_STOP;
        }
    }

}
// 璁剧疆鐩爣閲?
void Shoot_Reference( )
{
    ;
}

uint8_t test_falg = 0;

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
                manual_shoot_flag = 1;//拨轮用于拨弹
                
                Shoot_Enable();
                shoot_cmd -> shoot_v = SHOOT_V;
                Shoot_Set_All_Friction(shoot_cmd -> shoot_v);//璁剧疆鐩爣鍊?
                if(Shoot_VESC_Is_Ready())
                {
                    shoot_cmd -> shoot_frq = rc_data->rc.dial * REMOTE_SHOOT_SENSITIVITY * 200.0f * 36 / 8;;//Hz
                }
                else
                {
                    shoot_cmd -> shoot_frq = 0.0f;
                }

            }
            else
            {
                manual_shoot_flag = 0;//拨轮允许小陀螺
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
				if(shoot_cmd->friction_state == FRICTION_START)
                {
                    shoot_cmd -> shoot_v = SHOOT_V;
                }
                if(shoot_cmd->friction_state == FRICTION_STOP)
                {
                    shoot_cmd -> shoot_v = 0;
                }

                Shoot_Set_All_Friction(shoot_cmd -> shoot_v);

                if(Shoot_VESC_Is_Ready())
                {
                    if( shoot_cmd -> auto_state == SHOOT_MANUAL )
                    {
                        test_falg = 1;
                        shoot_cmd -> shoot_frq = 3000.0f;
                    }
                    else if( shoot_cmd -> auto_state == SHOOT_AUTO )
                    {
                        shoot_cmd -> shoot_frq = 3000.0f;
                    }
                    else
                    {
                        shoot_cmd -> shoot_frq = 0.0f;
                    }
                }
                else
                {
                    shoot_cmd -> shoot_frq = 0.0f;
                }

            }
            else
            {
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


void Shoot_Send_Cmd( )
{
    for(int i = 0; i <3; i++)
    {
        Motor_ExCan_Crtl(friction_motor[i]);
    }
}

