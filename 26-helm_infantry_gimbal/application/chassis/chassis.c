/**
******************************************************************************
 * @file    chassis.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "chassis.h"
#include "shoot.h"

#include "DM_motor.h"
#include "DJI_motor.h"

#include "robot_frame_config.h"
#include "robot_frame_init.h"


static chassis_cmd_t chassis_cmd_storage;
chassis_cmd_t *chassis_cmd = &chassis_cmd_storage;//静态初始化，防止空指针

float key_w =0;
float key_s =0;

void Chassis_Set_Mode(void)
{
    //模式切换
    if( (rc_data->rc.gear_shift == STOP_MODE && key_flag == 0) || ( keyboard_enable_flag == 0 && key_flag == 1) )
    {
        chassis_cmd -> chassis_mode = CHASSIS_MODE_STOP;
    }

    else if(rc_data->rc.gear_shift == REMOTE_MODE && key_flag == 0)
    {
        chassis_cmd -> chassis_mode = CHASSIS_MODE_REMOTE;
    }

    else if(rc_data->rc.gear_shift == KEYBOARD_MODE || ROBOT_FRAME_KEYBOARD_ENABLED() )
    {
        chassis_cmd -> chassis_mode = CHASSIS_MODE_KEYBOARD;//键鼠模式,默认使能
    }

    switch(chassis_cmd -> chassis_mode)
    {
        case CHASSIS_MODE_STOP:
            chassis_cmd -> move_mode = CHASSIS_STOP;
            break;

        case CHASSIS_MODE_REMOTE:
            chassis_cmd -> move_mode = CHASSIS_FOLLOW_GIMBAL;
            break;

        case CHASSIS_MODE_KEYBOARD:
            chassis_cmd -> move_mode = CHASSIS_FOLLOW_GIMBAL;
            break;
        
        default:
            chassis_cmd -> move_mode = CHASSIS_STOP;
            break;
    }
}


void Chassis_Console(void)
{

    switch( chassis_cmd -> chassis_mode )
    {
        case CHASSIS_MODE_STOP:
        {
            
            chassis_cmd -> vx = 0.0f;
            chassis_cmd -> vy = 0.0f;
            chassis_cmd -> vw = 0.0f;

            break;
        }
            
        case CHASSIS_MODE_REMOTE:
        {

            chassis_cmd -> vx = rc_data->rc.rocker_l1 * REMOTE_CHASSIS_SENSITIVITY_VX;
            chassis_cmd -> vy = rc_data->rc.rocker_l_ * REMOTE_CHASSIS_SENSITIVITY_VY;
            if(manual_shoot_flag == 0)
            {
                chassis_cmd -> vw = rc_data->rc.dial * REMOTE_CHASSIS_SENSITIVITY_VW;
            }
            else
            {
                chassis_cmd -> vw = 0;
            }

            break;
        }
            

        case CHASSIS_MODE_KEYBOARD:
        {
            if( rc_data->key[0].w == 1)
            {
                key_w = 1;
            }
            else if(rc_data->key[0].w == 0 )
            {
                key_w = (key_w > 0) ? key_w - 0.01 : 0;
            }
            if(rc_data->key[0].s ==1 )
            {
                key_s = 1;
            }
            else if(rc_data->key[0].s == 0 )
            {
                key_s = (key_s > 0) ? key_s - 0.05 : 0;
                
            }
            chassis_cmd -> vx = -rc_data->key[0].a * KEYBOARD_CHASSIS_SENSITIVITY_VX + rc_data->key[0].d * KEYBOARD_CHASSIS_SENSITIVITY_VX;
            chassis_cmd -> vy = key_w * KEYBOARD_CHASSIS_SENSITIVITY_VY - key_s * KEYBOARD_CHASSIS_SENSITIVITY_VY;
            chassis_cmd -> vw = rc_data->key[0].e * KEYBOARD_CHASSIS_SENSITIVITY_VW- rc_data->key[0].shift * KEYBOARD_CHASSIS_SENSITIVITY_VW;
						 
            break;
        }
            
        
        default:
        {
            chassis_cmd -> vx = 0.0f;
            chassis_cmd -> vy = 0.0f;
            chassis_cmd -> vw = 0.0f;
            break;
        }
    }


}