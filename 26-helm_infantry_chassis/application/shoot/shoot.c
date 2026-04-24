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

#include "shoot.h" 

#include "robot_frame_config.h"
#include "robot_frame_init.h"

static shoot_cmd_t shoot_cmd_storage;
shoot_cmd_t *shoot_cmd = &shoot_cmd_storage;//静态初始化，防止空指针


// PID_t friction_angle_pid = {
//     .kp = 12.0f,
//     .ki = 0.0f,
//     .kd = 480.0f,
//     .output_limit = 10.0f,
//     .integral_limit = 0.0f,
//     .dead_band = 0.0f,
// };

// PID_t friction_speed_pid = {
//     .kp = 400.0f,
//     .ki = 400.0f,
//     .kd = 0.0f,
//     .output_limit = 25000.0f,
//     .integral_limit = 25000.0f,
//     .dead_band = 0.0f,
// };

// motor_init_config_t friction_motor_init = {
//     .controller_param_init_config = {
//         .angle_PID = &friction_angle_pid,
//         .speed_PID = &friction_speed_pid,
//         .current_PID = NULL,
//         .torque_PID = NULL,

//         .other_angle_feedback_ptr = NULL,
//         .other_speed_feedback_ptr = NULL,

//         .angle_feedforward_ptr = NULL,
//         .speed_feedforward_ptr = NULL,
//         .current_feedforward_ptr = NULL,
//         .torque_feedforward_ptr = NULL,

//         .pid_ref = 0.0f,
//     },
//     .controller_setting_init_config = {
//         .outer_loop_type = ANGLE_LOOP,
//         .close_loop_type = ANGLE_AND_SPEED_LOOP,

//         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
//         .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

//         .angle_feedback_source = MOTOR_FEED,
//         .speed_feedback_source = MOTOR_FEED,

//         .feedforward_flag = FEEDFORWARD_NONE,
//     },

//     .motor_type = OTHER,//snail

//     .can_init_config = {
//         .can_handle = &hfdcan2,
//         .tx_id = 0x01,
//         .rx_id = 0x011,
//     },
// };

// void Shoot_Enable(void)
// {
//     for(int i = 0; i <3; i++)
//     {
//         Shoot_Motor_Enable(friction_motor[i]);
//     }

// }

// void Shoot_Disable(void)
// {
//     for(int i = 0; i <3; i++)
//     {
//         Shoot_Motor_Stop(friction_motor[i]);
//     }
// }

// void Shoot_Init(void)
// {
//     for(int i = 0;i < 3;i++)
// 	{
// 		friction_motor_init.can_init_config.tx_id = 0x01 + i;
//         friction_motor_init.can_init_config.rx_id = 0x011 + i;
// 		friction_motor[i] = Shoot_Motor_Init(&friction_motor_init);
// 	}
// }

// void Shoot_Set_All_Friction(int16_t speed)
// {
// 	for(int i = 0; i <3; i++)
//     {
//         Shoot_Motor_SetTar(friction_motor[i],speed);
//     }
// }

// void Shoot_Observer( )
// {
//     ;
// }

// // 处理异常
// void Shoot_Handle_Exception( )
// {
//     ;
// }

// // 设置射击模式
// void Shoot_Set_Mode( )
// {
//     //扳机键，开启摩擦轮
//     shoot_cmd -> mode = rc_data->rc_rise_count[VT03_RC_RISE_TRIGGE_KEY] % 3;//模式切换

// }
// // 设置目标量
// void Shoot_Reference( )
// {
//     ;
// }

// // 计算控制量
// void Shoot_Console( )
// {
//     switch(shoot_cmd->mode)
//     {
//         case SHOOT_MODE_STOP:
// 						shoot_cmd -> shoot_v = 0;
// 						shoot_cmd -> shoot_frq = 0;
//             Shoot_Disable();
//         break;

//         case SHOOT_MODE_MANUAL:
//         {
//             Shoot_Enable();

//             shoot_cmd -> shoot_v = SHOOT_V;
//             Shoot_Set_All_Friction(shoot_cmd -> shoot_v);

//             if(friction_motor[0] -> receive_flag == 0xA5 || friction_motor[1] -> receive_flag == 0xA5 || friction_motor[2] -> receive_flag == 0xA5)//摩擦轮开转后再给拨弹盘设置转速
//             {
//                 // if(Shoot_Mode_Change_Flag == 1)
//                 // {
//                 // 	Shoot_Mode_Change_Flag = 0;
//                 // 	shoot_start_timer = osKernelGetTickCount();  // 记录当前时间
//                 // 	shoot_ready_flag = 0;
//                 // }
//                 // if(osKernelGetTickCount() - shoot_start_timer >= 5000)// 判断是否已经等待满 3s
//                 // {
//                 // 	shoot_ready_flag = 1;//3秒到，可以发射
//                 // }

//                 shoot_cmd -> shoot_frq = rc_data->rc.dial * REMOTE_SHOT_SENSITIVITY;//Hz
//             }

//         }
//         break;

//         case SHOOT_MODE_AUTO:
//         {
//             Shoot_Enable();
//             shoot_cmd -> shoot_v = SHOOT_V;
//             shoot_cmd -> shoot_frq = 10.0f;//Hz

//             break;
//         }   

//         default:
//         {

//             shoot_cmd -> shoot_v = 0;
//             shoot_cmd -> shoot_frq = 0;
//             Shoot_Disable();
//             break;
//         }
            
        
//     }

// }

// // 发送控制量
// void Shoot_Send_Cmd( )
// {
//     Shoot_Motor_Send();
// }