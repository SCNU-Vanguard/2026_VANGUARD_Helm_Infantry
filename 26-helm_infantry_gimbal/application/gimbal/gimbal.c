/**
******************************************************************************
 * @file    gimbal.c
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

#include "gimbal.h"
#include "robot_frame_init.h"
#include "robot_frame_config.h"

#include "DM_motor.h"
#include "INS.h"

DM_motor_instance_t *gimbal_pitch_neck;
DM_motor_instance_t *gimbal_pitch_head;

static gimbal_cmd_t gimbal_cmd_storage;
gimbal_cmd_t *gimbal_cmd = &gimbal_cmd_storage;//静态初始化，防止空指针

/*云台电机初始化*/
PID_t pitch_head_angle_pid = {
    .kp = 0,
    .ki = 0,
    .kd = 0,	
    .output_limit = 5.0f, 
    .integral_limit = 5.0f,
    .dead_band = 0.0f,
};

PID_t pitch_head_speed_pid = {
    .kp = 0,
    .ki = 0,
    .kd = 0,	
    .output_limit = 5.0f, 
    .integral_limit = 5.0f,
    .dead_band = 0.0f,
};

//脖子电机,位置速度模式
motor_init_config_t pitch_neck_init = {
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

        .control_button = POLYCYCLIC_LOOP_CONTROL,//电流闭环控制
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        
        .angle_feedback_source = MOTOR_FEED,//上板传入数据
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = DM4310,
    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },

};


//pitch头部电机,MIT模式
motor_init_config_t pitch_head_init = {
    .controller_param_init_config = {
        .angle_PID = &pitch_head_angle_pid,
        .speed_PID = &pitch_head_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = &INS.Pitch,//IMU
        .other_speed_feedback_ptr = &INS.Gyro[IMU_Z],//

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },

    .controller_setting_init_config = {
        .outer_loop_type = OPEN_LOOP,
        .close_loop_type = OPEN_LOOP,

        .control_button = POLYCYCLIC_LOOP_CONTROL,//电流闭环控制
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        
        .angle_feedback_source = OTHER_FEED,//上板传入数据
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = DM4310,
    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x02,
        .rx_id = 0x12,
    },

};

void Gimbal_Init(void)
{
	gimbal_pitch_neck = DM_Motor_Init(&pitch_neck_init);
    gimbal_pitch_head = DM_Motor_Init(&pitch_head_init);

    gimbal_pitch_neck->dm_mode = POS_MODE;
    gimbal_pitch_head->dm_mode = MIT_MODE;

}


//完全失能模式,云台没力
void Gimbal_Disable(void)
{
   DM_Motor_Disable(gimbal_pitch_neck);
   DM_Motor_Stop(gimbal_pitch_neck);

   DM_Motor_Disable(gimbal_pitch_head);
   DM_Motor_Stop(gimbal_pitch_head);
}


void Gimbal_Enable(void)
{
    DM_Motor_Enable(gimbal_pitch_neck);
    DM_Motor_Start(gimbal_pitch_neck);

    DM_Motor_Enable(gimbal_pitch_head);
    DM_Motor_Start(gimbal_pitch_head);
}


void Gimbal_Observer(void)
{
    ;
}

void Gimbal_Handle_Exception(void)
{
    DM_Motor_Error_Judge(gimbal_pitch_neck);
    DM_Motor_Error_Judge(gimbal_pitch_head);

}

void Gimbal_Set_Mode(void)
{
    //VT13遥控器控制
    if(rc_data->rc.gear_shift == STOP_MODE)
    {
        gimbal_cmd -> gimbal_mode = GIMBAL_MODE_STOP;
    }

    else if(rc_data->rc.gear_shift == REMOTE_MODE)
    {
        gimbal_cmd -> gimbal_mode = GIMBAL_MODE_REMOTE;
    }

    else if(rc_data->rc.gear_shift == KEYBOARD_MODE)
    {
        gimbal_cmd -> gimbal_mode = GIMBAL_MODE_KEYBOARD;//键鼠模式
    }

    //遥控器控制
    if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_REMOTE)
    {
        gimbal_cmd -> neck_state = rc_data->rc_rise_count[VT03_RC_RISE_AUTO_KEY_L] % 2;//自动按键左控制脖子状态
        gimbal_cmd -> auto_state = rc_data->rc_rise_count[VT03_RC_RISE_AUTO_KEY_R] % 2;//自动按键右控制自动模式状态

    }

}



void Gimbal_Reference(void)
{
    ;
}

void Gimbal_Console(void)
{
    gimbal_cmd -> current_angle_yaw = INS.Yaw;
    gimbal_cmd -> current_angle_head = INS.Pitch;

   

    //遥控器控制
    if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_REMOTE )
    {
        //脖子控制
        if(gimbal_cmd -> neck_state == NECK_LAY)
        {
            ;//缩头
        }

        else if(gimbal_cmd -> neck_state == NECK_STAND)
        {
            ;//抬头
        }

        
        if( gimbal_cmd -> neck_state == NECK_STAND && gimbal_cmd -> auto_state == GIMBAL_MANUAL )
        {
            gimbal_cmd -> target_angle_yaw += rc_data->rc.rocker_l_ * REMOTE_YAW_SENSITIVITY;
            gimbal_cmd -> target_angle_head += rc_data->rc.rocker_l1 * REMOTE_PITCH_SENSITIVITY;
        }
        else if( gimbal_cmd -> neck_state == NECK_LAY )
        {
            gimbal_cmd -> target_angle_yaw = INS.Yaw;
            gimbal_cmd -> target_angle_head = INS.Pitch;
        }
        else if( gimbal_cmd -> auto_state == GIMBAL_AUTO )
        {
            ;//角度由上位机决定
        }
        

        //限幅
        if( gimbal_cmd -> target_angle_yaw  > PI )
        {
            gimbal_cmd -> target_angle_yaw -= 2 * PI;
        }
        else if( gimbal_cmd -> target_angle_yaw  < -PI )
        {
            gimbal_cmd -> target_angle_yaw += 2 * PI;
        }

        //USER_LIMIT_MIN_MAX();//PITCH


        DM_Motor_SetTar(gimbal_pitch_head, gimbal_cmd -> target_angle_head);
        DM_Motor_SetTar(gimbal_pitch_neck, gimbal_cmd -> target_angle_neck);
    }

    else if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_KEYBOARD )
    {
        ;//键鼠控制,待添加
    }

    else
    {
        gimbal_cmd -> target_angle_yaw = INS.Yaw;
        gimbal_cmd -> target_angle_head = INS.Pitch;
    }

}

void Gimbal_Send_Cmd(void)
{
    //
    switch (gimbal_cmd -> gimbal_mode)
    {
        case GIMBAL_MODE_STOP:
            Gimbal_Disable();
            break;

        case GIMBAL_MODE_REMOTE:
            Gimbal_Enable();
            
            break;

        case GIMBAL_MODE_KEYBOARD:
            Gimbal_Enable();
            break;

        default:
            Gimbal_Disable();
            break;
    }

    //
    DM_Motor_Control(NULL);
}
