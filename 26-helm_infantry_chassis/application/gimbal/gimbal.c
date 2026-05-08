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

#include "gimbal.h"

#include "DM_motor.h"

static gimbal_cmd_t gimbal_cmd_storage;
gimbal_cmd_t *gimbal_cmd = &gimbal_cmd_storage;//静态初始化，防止空指针(任务堆栈)

DM_motor_instance_t *gimbal_dm6006;//云台电机Gimbal_DM6006_Init

//避免对 packed 成员取址导致未对齐指针告警
static float gimbal_current_angle_feedback = 0.0f;
static float gimbal_current_yaw_acc_feedback = 0.0f;

/*云台电机*/
// PID_t gimbal_dm6006_angle_pid = {
//     .kp = 7.9,
//     .ki = 0,
//     .kd = 0,	
//     .output_limit = 10.0f,//5.0f, 
//     .integral_limit = 5.0f,
//     .dead_band = 0.0f,
// };

// PID_t gimbal_dm6006_speed_pid = {
//     .kp = 3.95,
//     .ki = 0.021,
//     .kd = 0,	
//     .output_limit = 6.5f, 
//     .integral_limit = 5.0f,
//     .dead_band = 0.0f,
// };

PID_t gimbal_dm6006_angle_pid = {
    .kp = 6.0,
    .ki = 0,
    .kd = 5,	
    .output_limit = 20.0f,//5.0f, 
    .integral_limit = 5.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_dm6006_speed_pid = {
    .kp = 4.2,
    .ki = 0.018,
    .kd = 0,	
    .output_limit = 7.0f, 
    .integral_limit = 5.0f,
    .dead_band = 0.0f,
};

motor_init_config_t gimbal_dm6006_init = {
    
    .controller_param_init_config = {
        .angle_PID = &gimbal_dm6006_angle_pid,
        .speed_PID = &gimbal_dm6006_speed_pid,
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
        .close_loop_type = ANGLE_AND_SPEED_LOOP ,

        .control_button = POLYCYCLIC_LOOP_CONTROL,//电流闭环控制
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        // .angle_feedback_source = MOTOR_FEED,
        // .speed_feedback_source = MOTOR_FEED,
        
        .angle_feedback_source = OTHER_FEED,//上板传入数据
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = OTHER,

    .can_init_config = {
        .can_handle = &hfdcan3,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },

};

void Gimbal_DM6006_Init(void)
{
	
	//改用上板传下的IMU角度和角加速度作为反馈
    gimbal_dm6006_init.controller_param_init_config.other_angle_feedback_ptr = &gimbal_current_angle_feedback;
    gimbal_dm6006_init.controller_param_init_config.other_speed_feedback_ptr = &gimbal_current_yaw_acc_feedback;
	
    gimbal_dm6006 = DM_Motor_Init(&gimbal_dm6006_init);


    DM_Motor_Enable(gimbal_dm6006);//发送使能信号

    DM_Motor_Start(gimbal_dm6006);//允许发送
}


//完全失能模式,云台没力
void Gimbal_Disable(void)
{
    if( gimbal_dm6006->receive_data.state == 1 )
    {
        DM_Motor_Disable(gimbal_dm6006);
    }
    DM_Motor_Stop(gimbal_dm6006);
    gimbal_dm6006->motor_controller.speed_PID->output = 0;
    gimbal_dm6006->motor_controller.angle_PID->output = 0;
}


void Gimbal_Enable(void)
{
    if( gimbal_dm6006->receive_data.state == 0 )
    {
        DM_Motor_Enable(gimbal_dm6006);
    }
    DM_Motor_Start(gimbal_dm6006);
}

float pid_yaw_p = 0;
float pid_yaw_i = 0;
float pid_yaw_d = 0;
//外环
float pid_yaw_p_out = 1;
float pid_yaw_i_out = 0;
float pid_yaw_d_out = 0;

float yaw_tar =0;
float yaw_current = 0;

float torque = 0;
float g6006_i_out = 0;

//逆正顺负
void Gimbal_DM6006_Ctrl(void)
{

    // gimbal_dm6006->motor_controller.speed_PID->kp = pid_yaw_p ;
    // gimbal_dm6006->motor_controller.speed_PID->ki = pid_yaw_i;
    // gimbal_dm6006->motor_controller.speed_PID->kd = pid_yaw_d;

    // gimbal_dm6006->motor_controller.angle_PID->kp = pid_yaw_p_out;
    // gimbal_dm6006->motor_controller.angle_PID->ki = pid_yaw_i_out;
    // gimbal_dm6006->motor_controller.angle_PID->kd = pid_yaw_d_out;
    /* 野指针保护 */
    if( gimbal_dm6006 == NULL || gimbal_cmd == NULL )
    {
        return;
    }
    
    torque = gimbal_dm6006->receive_data.torque;
    yaw_current = gimbal_cmd->current_angle_yaw;
    float tar_yaw = gimbal_cmd -> target_angle_yaw;
    g6006_i_out = gimbal_dm6006->motor_controller.speed_PID->i_out;

		
    gimbal_current_angle_feedback = gimbal_cmd->current_angle_yaw;
    gimbal_current_yaw_acc_feedback = gimbal_cmd->current_yaw_acc;
    

    if( fabsf(gimbal_cmd->target_angle_yaw) > 2 * PI || fabsf(gimbal_current_angle_feedback) > 2 * PI )
    {
        return;
    }


    switch(gimbal_cmd->gimbal_mode)
    {

        case GIMBAL_MODE_STOP:
            Gimbal_Disable( );
            break;

        case GIMBAL_MODE_REMOTE:
            Gimbal_Enable( );
            DM_Motor_SetTar(gimbal_dm6006, tar_yaw);
            break;

        case GIMBAL_MODE_KEYBOARD:
            Gimbal_Enable( );
            DM_Motor_SetTar(gimbal_dm6006, tar_yaw);
            break;

        default:
            Gimbal_Disable( );
            break;
    }
		yaw_tar = tar_yaw;

    DM_Motor_Control(gimbal_dm6006);
}