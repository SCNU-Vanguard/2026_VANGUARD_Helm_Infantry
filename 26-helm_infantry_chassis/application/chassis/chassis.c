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
#include "gimbal.h"

#include "DM_motor.h"
#include "DJI_motor.h"

#include "robot_frame_config.h"
#include "pid.h"

/******************************底盘相关参数*****************************/
static chassis_cmd_t chassis_cmd_storage;
chassis_cmd_t *chassis_cmd = &chassis_cmd_storage;//静态初始化，防止空指针

/******************************DJI电机底盘初始化实例*****************************/
DJI_motor_instance_t *chassis_drive[4];//4*驱动轮   ，
DJI_motor_instance_t *chassis_helm[2];//2*舵向电机

float helm_zero[2] = {CHASSIS_GM6020_ZERO_1, CHASSIS_GM6020_ZERO_2};
float helm_angle[2] = {-PI/4, 3*PI/4};//底盘小陀螺时舵向角度


PID_t chassis_3508_speed_pid = {
    .kp = 23.0f,
    .ki = 0.081f,
    .kd = 0.0f,	
    .output_limit = 15000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

PID_t chassis_gm6020_angle_pid = {
    .kp = 25,
    .ki = 0,
    .kd = 0,	
    .output_limit = 100.0f, 
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};

// PID_t chassis_gm6020_speed_pid = {
//     .kp = 300,
//     .ki = 70,
//     .kd = 0,	
//     .output_limit = 20000.0f, 
//     .integral_limit = 1000.0f,
//     .dead_band = 0.0f,
// };

PID_t chassis_gm6020_speed_pid = {
    .kp = 200,
    .ki = 50,
    .kd = 0,	
    .output_limit = 20000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

/*底盘跟随*/
PID_t chassis_follow_gimbal_angle_pid = {
    .kp = 10,
    .ki = 0,
    .kd = 0,	
    .output_limit = 8.0f, 
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
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .control_button = POLYCYCLIC_LOOP_CONTROL,//电流闭环控制
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan1,
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

    //舵驱动
    chassis_drive[0]->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    //chassis_drive[2]->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;

    for (uint8_t i = 0; i < 2; ++i)
    {
        chassis_gm6020_init.can_init_config.tx_id = i + 1;
        chassis_gm6020_init.can_init_config.rx_id = i + 1;
        chassis_helm[i] = DJI_Motor_Init(&chassis_gm6020_init);

        chassis_helm[i]->motor_feedback = RAD;//设置舵轮反馈为弧度制，底盘解算时直接使用弧度值，无需再转换
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

float wheel_speed[4];
float current_angle[2];
float target_angle[2];
float temp_angle[2];
float speed_helm[2];//舵轮线速度

/*
    * @brief 底盘跟随
    * @param none
*/
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

    if( cmd->vw != 0.0f )
    {
        cmd->vw_follow = 0;
        chassis_follow_gimbal_angle_pid.output = 0;
        chassis_follow_gimbal_angle_pid.i_out = 0;
    }
}

/*
    * @brief 两舵轮两全解算
    * @param none
*/
static void Chassis_Calculate(chassis_cmd_t *cmd)
{
    float vx = cmd->vx;
    float vy = cmd->vy;
    float vw = cmd->vw + cmd->vw_follow;

    float vx_total = 0;//底盘总线速度
    float vy_total = 0;


    //////////////   云台坐标系转底盘坐标系   //////////////
    float angle_error = gimbal_dm6006->receive_data.position - CHASSIS_FORWARD_ZERO;    
    float vx_chassis = vx * cosf(angle_error) - vy * sinf(angle_error);
    float vy_chassis = vx * sinf(angle_error) + vy * cosf(angle_error);


    //////////////   底盘跟随   //////////////
    if( gimbal_cmd->neck_state == NECK_STAND )//抬头时底盘跟随
    {
        Chassis_Follow(cmd, gimbal_dm6006->receive_data.position, CHASSIS_FOLLOW_FORWARD_ZERO);
    }
    else if( gimbal_cmd->neck_state == NECK_LAY )//缩头时底盘不跟随
    {
        Chassis_Follow(cmd, gimbal_dm6006->receive_data.position, CHASSIS_GIMBAL_LAY_ZERO);
    }


    //////////////   舵轮解算   //////////////
    
        for(uint8_t i = 0; i < 2; i++)
        {
            vx_total = vx_chassis - vw * CHASSIS_RADIUS * sinf( helm_angle[i] );
            vy_total = vy_chassis + vw * CHASSIS_RADIUS * cosf( helm_angle[i] );
            speed_helm[i] = sqrtf(vx_total * vx_total + vy_total * vy_total);//线速度 ???
    
    
            //角度处理
            temp_angle[i] = - atan2f(vy_total, vx_total);//顺时针编码值为正
            current_angle[i] = chassis_helm[i]->receive_data.ecd * ECD_2_RAD - helm_zero[i];
            //180优劣弧处理
            float angle_diff = temp_angle[i] - current_angle[i];
            while (angle_diff > PI)
                    angle_diff -= 2 * PI;
            while (angle_diff < -PI)
                angle_diff += 2 * PI;
    
            //90度处理，速度反转
            if (angle_diff > PI / 2)
            {
                angle_diff -= PI;
                speed_helm[i] = -speed_helm[i];
            }
            else if (angle_diff < -PI / 2)
            {
                angle_diff += PI;
                speed_helm[i] = -speed_helm[i];
            }
						
//            if(vx != 0 || vy != 0 || cmd->vw != 0)
//            {
                target_angle[i] = chassis_helm[i]->receive_data.ecd * ECD_2_RAD + angle_diff; 
//            }

//            else////////    维持当前舵向    ///////////
//            {
//                target_angle[i] = chassis_helm[i]->receive_data.ecd * ECD_2_RAD ; 
//            }
        }
   
    
    //////////////   驱动轮解算   //////////////
    //舵轮
    wheel_speed[CHASSIS_DRIVE_HELM_1] = speed_helm[0]  / HELM_WHEEL_RADIUS * HELM_REDUCTION_RATIO * RAD_2_RPM ;//RPM
    wheel_speed[CHASSIS_DRIVE_HELM_2] = speed_helm[1]  / HELM_WHEEL_RADIUS * HELM_REDUCTION_RATIO * RAD_2_RPM ;//RPM
    //全向轮
    wheel_speed[CHASSIS_DRIVE_OMNI_1] = (vw  * CHASSIS_RADIUS + vx_chassis - vy_chassis ) / OMNI_WHEEL_RADIUS * OMNI_REDUCTION_RATIO * RAD_2_RPM;//RPM
    wheel_speed[CHASSIS_DRIVE_OMNI_2] = (vw  * CHASSIS_RADIUS + vy_chassis - vx_chassis ) / OMNI_WHEEL_RADIUS * OMNI_REDUCTION_RATIO * RAD_2_RPM;//RPM
    
    //3508驱动轮速度
    for(uint8_t i = 0; i < 4; ++i)
    {
        DJI_Motor_Set_Ref(chassis_drive[i], wheel_speed[i]);
    }

    //舵轮舵向
    for(uint8_t i = 0; i < 2; ++i)
    {
        DJI_Motor_Set_Ref(chassis_helm[i], target_angle[i]);
    }

}



/*********************************底盘模式控制*****************************/
void Chassis_Ctrl(void)
{

    switch (chassis_cmd->chassis_mode)
    {
        case CHASSIS_STOP:
            Chassis_Disable();
            chassis_cmd->vx = 0.0f;
            chassis_cmd->vy = 0.0f;
            chassis_cmd->vw = 0.0f;
            chassis_cmd->vw_follow = 0.0f;
            break;

        case CHASSIS_FOLLOW_GIMBAL:
            Chassis_Enable();
            Chassis_Calculate(chassis_cmd);
            break;

        case CHASSIS_MOVE:
            Chassis_Enable();
            Chassis_Calculate(chassis_cmd);
            break;

        default:
            Chassis_Disable();
            break;
    }

    DJI_Motor_Control(NULL);
}
