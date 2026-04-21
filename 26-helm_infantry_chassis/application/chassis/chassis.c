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
#include "config.h"
#include "gimbal.h"

#include "DM_motor.h"
#include "DJI_motor.h"

/******************************底盘相关参数*****************************/
chassis_cmd_t chassis_cmd;

/******************************DJI电机底盘初始化实例*****************************/
DJI_motor_instance_t *chassis_drive[4];//4*驱动轮   ，
DJI_motor_instance_t *chassis_helm[2];//2*舵向电机


PID_t chassis_3508_speed_pid = {
    .kp = 28.005f,
    .ki = 0.26f,
    .kd = 2.8f,	//3.0f
    .output_limit = 15000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

PID_t chassis_gm6020_angle_pid = {
    .kp = 0,
    .ki = 0,
    .kd = 0,	
    .output_limit = 10.0f, 
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};

PID_t chassis_gm6020_speed_pid = {
    .kp = 0,
    .ki = 0,
    .kd = 0,	
    .output_limit = 15000.0f, //25000
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

/*底盘跟随电机*/
PID_t chassis_follow_gimbal_angle_pid = {
    .kp = 0,
    .ki = 0,
    .kd = 0,	
    .output_limit = 0.0f, 
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};



motor_init_config_t chassis_m3508_init = {
    
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .control_button = POLYCYCLIC_LOOP_CONTROL,//电流闭环控制
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },

};

motor_init_config_t chassis_gm6020_init = {
    
    .controller_param_init_config = {
        .angle_PID = &chassis_gm6020_angle_pid,
        .speed_PID = &chassis_gm6020_speed_pid,
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
        .close_loop_type = SPEED_LOOP,

        .control_button = POLYCYCLIC_LOOP_CONTROL,//电流闭环控制
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },

};

/******************************底盘初始化函数*****************************/
void Chassis_Init(void)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
			chassis_m3508_init.can_init_config.tx_id = i + 1;
			chassis_m3508_init.can_init_config.rx_id = i + 1;
			chassis_drive[i] = DJI_Motor_Init(&chassis_m3508_init);
			DJI_Motor_Enable(chassis_drive[i]);
			
    }

    for (uint8_t i = 0; i < 2; ++i)
    {
				chassis_gm6020_init.can_init_config.tx_id = i + 1;
				chassis_gm6020_init.can_init_config.rx_id = i + 1;
        chassis_helm[i] = DJI_Motor_Init(&chassis_gm6020_init);
        DJI_Motor_Enable(chassis_helm[i]);
    }

}

void Chassis_Enable(void)
{
    //允许发送
    for (uint8_t i = 0; i < 4; ++i)
    {
        DJI_Motor_Enable(chassis_drive[i]);
    }

    for (uint8_t i = 0; i < 2; ++i)
    {
        DJI_Motor_Enable(chassis_helm[i]);
    }
}

void Chassis_Disable(void)
{
    //禁止发送，且清空PID
    for (uint8_t i = 0; i < 4; ++i)
    {
        DJI_Motor_Disable(chassis_drive[i]);

        chassis_drive[i]->motor_controller.speed_PID->output = 0.0f;
    }

    for (uint8_t i = 0; i < 2; ++i)
    {
        DJI_Motor_Disable(chassis_helm[i]);

        chassis_helm[i]->motor_controller.angle_PID->output = 0.0f;
        chassis_helm[i]->motor_controller.speed_PID->output = 0.0f;
    }

}


/******************************两舵两全底盘解算*****************************/
//
//???
//             1(舵)
//          /        \
//         /          \
//        /            \          ^vy
//       4              2         |
//       \              /         ---> vx
//        \            /
//          \        /
//             3(舵)

//
//      1(舵)-----------2
//      |               |
//      |       ^vy     |
//      |       |       |
//      |       -->vx   |
//      |               |
//      4---------------3(舵)
//

/*
    * @brief 云台坐标系转底盘坐标系
    * @param cmd 
    * @param gimbal_current_angle 云台当前角度,RAD
    * @param chassis_forward_zero 底盘正向向前时云台电机零点, RAD
*/

static void Gimbal_To_Chassis_Frame(chassis_cmd_t *cmd, float gimbal_current_angle , float chassis_forward_zero)
{
    float angle_error = gimbal_current_angle - chassis_forward_zero;
    cmd->vx = cmd->vx * cosf(angle_error) - cmd->vy * sinf(angle_error);
    cmd->vy = cmd->vx * sinf(angle_error) + cmd->vy * cosf(angle_error);
}


static void Chassis_Calculate(chassis_cmd_t *cmd)
{
    float helm_zero[2] = {CHASSIS_GM6020_ZERO_1, CHASSIS_GM6020_ZERO_2};
    float helm_angle[2] = {-PI/4, 3*PI/4};//底盘小陀螺时舵向角度

    float wheel_speed[4];

    float current_angle;
    float target_angle;
    float speed_helm[2];//舵轮线速度
    float vx_total;//底盘总线速度
    float vy_total;

    //舵向角度处理
    for(uint8_t i = 0; i < 2; i++)
    {
        vx_total = cmd->vx - cmd->vw * CHASSIS_RADIUS * sinf( helm_angle[i] );
        vy_total = cmd->vy + cmd->vw * CHASSIS_RADIUS * cosf( helm_angle[i] );
        speed_helm[i] = sqrtf(vx_total * vx_total + vy_total * vy_total);//线速度 ???

        //角度处理
        target_angle = atan2f(vy_total, vx_total);
        current_angle = chassis_helm[i]->receive_data.ecd * ECD_2_RAD - helm_zero[i];
        //180优劣弧处理
        if(target_angle - current_angle > PI)
        {
            target_angle -= 2 * PI;
        }
        else if(target_angle - current_angle < -PI)
        {
            target_angle += 2 * PI;
        }

        //90度处理，速度反转
        // if(target_angle - current_angle > PI/2)
        // {
        //     target_angle  -= PI;
        // }
        // else if(target_angle - current_angle < -PI/2)
        // {
        //     target_angle +=  PI;
        // }
          
        DJI_Motor_Set_Ref(chassis_helm[i], target_angle); 
    }

    //舵轮
    wheel_speed[CHASSIS_DRIVE_HELM_1] = speed_helm[0]  / HELM_WHEEL_RADIUS * HELM_REDUCTION_RATIO * RAD_2_RPM;//RPM
    wheel_speed[CHASSIS_DRIVE_HELM_2] = speed_helm[1]  / HELM_WHEEL_RADIUS * HELM_REDUCTION_RATIO * RAD_2_RPM;//RPM
    //全向轮
    wheel_speed[CHASSIS_DRIVE_OMNI_1] = (cmd->vw  * CHASSIS_RADIUS - cmd->vx - cmd->vy ) / OMNI_WHEEL_RADIUS * OMNI_REDUCTION_RATIO * RAD_2_RPM;//RPM
    wheel_speed[CHASSIS_DRIVE_OMNI_2] = (cmd->vw  * CHASSIS_RADIUS + cmd->vy - cmd->vw ) / OMNI_WHEEL_RADIUS * OMNI_REDUCTION_RATIO * RAD_2_RPM;//RPM


    for(uint8_t i = 0; i < 4; ++i)
    {
         DJI_Motor_Set_Ref(chassis_drive[i], wheel_speed[i]);
    }
}

static void Chassis_Follow(chassis_cmd_t *cmd,float gimbal_current_angle , float chassis_forward_zero)
{
    /*抬头时限位保护*/
    float error_angle = gimbal_current_angle - chassis_forward_zero;
    if(error_angle > PI)
    {
        error_angle -= 2 * PI;
    }
    else if(error_angle < -PI)
    {
        error_angle += 2 * PI;
    }
    cmd->vw_follow = PID_Position(&chassis_follow_gimbal_angle_pid, error_angle , 0.0f);

    cmd->vw += cmd->vw_follow;//底盘跟随时叠加跟随角速度

}


/*********************************底盘模式控制*****************************/
void Chassis_Ctrl(void)
{

    switch (chassis_cmd.chassis_mode)
    {
        case CHASSIS_STOP:
            Chassis_Disable();
            chassis_cmd.vx = 0.0f;
            chassis_cmd.vy = 0.0f;
            chassis_cmd.vw = 0.0f;
            chassis_cmd.vw_follow = 0.0f;
            break;

        case CHASSIS_FOLLOW_GIMBAL:
            Chassis_Enable();

            Chassis_Follow(&chassis_cmd, gimbal_dm6006->receive_data.position, CHASSIS_FORWARD_ZERO);
            Gimbal_To_Chassis_Frame(&chassis_cmd, gimbal_dm6006->receive_data.position, CHASSIS_FORWARD_ZERO);
            Chassis_Calculate(&chassis_cmd);
            break;

        case CHASSIS_MOVE:
            Chassis_Enable();
            Gimbal_To_Chassis_Frame(&chassis_cmd, gimbal_dm6006->receive_data.position, CHASSIS_FORWARD_ZERO);
            Chassis_Calculate(&chassis_cmd);
            break;

        default:
            Chassis_Disable();
            break;
    }
}