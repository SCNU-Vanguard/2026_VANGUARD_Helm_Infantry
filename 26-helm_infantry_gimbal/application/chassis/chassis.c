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

#include "DM_motor.h"
#include "DJI_motor.h"

#include "robot_frame_config.h"
#include "robot_frame_init.h"

static chassis_cmd_t chassis_cmd_storage;
chassis_cmd_t *chassis_cmd = &chassis_cmd_storage;//静态初始化，防止空指针


void Chassis_Set_Mode(void)
{
    switch(rc_data->rc.gear_shift)
    {
        case STOP_MODE:
            chassis_cmd -> chassis_mode = CHASSIS_STOP;
            break;

        case REMOTE_MODE:
            chassis_cmd -> chassis_mode = CHASSIS_FOLLOW_GIMBAL;
            break;

        case KEYBOARD_MODE:
            chassis_cmd -> chassis_mode = CHASSIS_MOVE;
            break;
        
        default:
            chassis_cmd -> chassis_mode = CHASSIS_STOP;
            break;
    }
}


void Chassis_Console(void)
{

    switch( rc_data->rc.gear_shift )
    {
        case STOP_MODE:
        {
            
            chassis_cmd -> vx = 0.0f;
            chassis_cmd -> vy = 0.0f;
            chassis_cmd -> vw = 0.0f;

            break;
        }
            

        case REMOTE_MODE:
        {
            
            chassis_cmd -> vx = rc_data->rc.rocker_l1 * REMOTE_CHASSIS_SENSITIVITY_VX;
            chassis_cmd -> vy = rc_data->rc.rocker_l_ * REMOTE_CHASSIS_SENSITIVITY_VY;
            chassis_cmd -> vw = rc_data->rc.dial * REMOTE_CHASSIS_SENSITIVITY_VW;

            break;
        }
            

        case KEYBOARD_MODE:
        {
            
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