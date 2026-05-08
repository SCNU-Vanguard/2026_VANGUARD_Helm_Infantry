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
#include "shoot.h"
#include "robot_frame_init.h"
#include "robot_frame_config.h"

#include "Serial.h"

#include "remote_vt03.h"
#include "DM_motor.h"
#include "INS.h"

#include "DJI_motor.h"
#include "DM_motor.h"

DM_motor_instance_t *gimbal_pitch_neck;
DJI_motor_instance_t *gimbal_pitch_head;

static gimbal_cmd_t gimbal_cmd_storage;
gimbal_cmd_t *gimbal_cmd = &gimbal_cmd_storage;//静态初始化，防止空指针


/*云台电机初始化*/
PID_t pitch_head_angle_pid = {
    .kp = 7.5,
    .ki = 0,
    .kd = 0,	
    .output_limit = 10000.0f, 
    .integral_limit = 5.0f,
    .dead_band = 0.0f,
};

PID_t pitch_head_speed_pid = {
    .kp = 5000,
    .ki = 16,
    .kd = 30,	
    .output_limit = 20000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

/*云台跟随底盘*/
// PID_t gimbal_follow_chassis_angle_pid = {
//     .kp = 0.1,
//     .ki = 0,
//     .kd = 0,	
//     .output_limit = 2.0f,//注意-PI - PI 
//     .integral_limit = 0.0f,
//     .dead_band = 0.0f,
// };

//pitch头部电机,6020
motor_init_config_t pitch_head_init = {
    .controller_param_init_config = {
        .angle_PID = &pitch_head_angle_pid,
        .speed_PID = &pitch_head_speed_pid,
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

        
        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
			
    },


    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },
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
        .can_handle = &hfdcan2,
        .tx_id = 0x02,
        .rx_id = 0x12,
    },

};



void Gimbal_Init(void)
{
    pitch_head_init.controller_param_init_config.other_angle_feedback_ptr = &INS.Pitch;//使用imu的yaw角度作为yaw电机的角度反馈
    pitch_head_init.controller_param_init_config.other_speed_feedback_ptr = &INS.Gyro[0];//使用imu的z轴陀螺仪作为yaw电机的速度反馈,注意方向

    gimbal_pitch_head = DJI_Motor_Init(&pitch_head_init);

    gimbal_pitch_neck = DM_Motor_Init(&pitch_neck_init);

    //设置控制模式
    gimbal_pitch_neck->dm_mode = POS_MODE;
    //设置反馈量类型
    gimbal_pitch_neck->motor_feedback = DM_MOTOR_ABSOLUTE;

}


//完全失能模式,云台没力
void Gimbal_Disable(void)
{
    if( gimbal_pitch_neck->receive_data.state == 1 )
    {
        DM_Motor_Disable(gimbal_pitch_neck);
    }
    DM_Motor_Stop(gimbal_pitch_neck);

    DJI_Motor_Disable(gimbal_pitch_head);
		
    gimbal_pitch_head->motor_controller.angle_PID->output = 0;
    gimbal_pitch_head->motor_controller.speed_PID->output = 0;
		
}


void Gimbal_Enable(void)
{
    if( gimbal_pitch_neck->receive_data.state == 0 )
    {
        DM_Motor_Enable(gimbal_pitch_neck);
    }
    DM_Motor_Start(gimbal_pitch_neck);

    DJI_Motor_Enable(gimbal_pitch_head);	
}


void Gimbal_Observer(void)
{
    ;
}

void Gimbal_Handle_Exception(void)
{
    DM_Motor_Error_Judge(gimbal_pitch_neck);
    DJI_Motor_Error_Judge(gimbal_pitch_head);

}

uint32_t pause_count = 0;

//模式选择
void Gimbal_Set_Mode(void)
{
	
    //模式切换
    if( (rc_data->rc.gear_shift == STOP_MODE && key_flag == 0) || (keyboard_enable_flag == 0 && key_flag == 1) )
    {
        gimbal_cmd -> gimbal_mode = GIMBAL_MODE_STOP;
    }

    else if(rc_data->rc.gear_shift == REMOTE_MODE && key_flag == 0)
    {
        gimbal_cmd -> gimbal_mode = GIMBAL_MODE_REMOTE;
    }

    else if(rc_data->rc.gear_shift == KEYBOARD_MODE || ROBOT_FRAME_KEYBOARD_ENABLED() )
    {
        gimbal_cmd -> gimbal_mode = GIMBAL_MODE_KEYBOARD;//键鼠模式,默认使能
    }

    //VT13遥控器控制
    if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_REMOTE)
    {
        gimbal_cmd -> neck_state = rc_data->rc_rise_count[VT03_RC_RISE_AUTO_KEY_L] % 2;//FN键控制脖子状态
        gimbal_cmd -> auto_state = rc_data->rc_rise_count[VT03_RC_RISE_AUTO_KEY_R] % 2;//拍照键 -- 开启自瞄

        if(  rc_data->rc.pause == 1 )
        {
            pause_count++;
            if(pause_count > 1000)
            {
                pause_count = 0;
                key_flag = 0;
                keyboard_enable_flag = 0;
            }
        }

    }

    else if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_KEYBOARD )
    {
        //自瞄模式
        if( rc_data->mouse.press_r == 1 )
        {
            gimbal_cmd -> auto_state = GIMBAL_AUTO;
        }
        else if( rc_data->mouse.press_r == 0 )
        {
            gimbal_cmd -> auto_state = GIMBAL_MANUAL;
        }

        gimbal_cmd -> neck_state = GET_KEY_COUNT(VT03_KEY_PRESS , KEY_F) % 2;//按键F控制脖子状态

        gimbal_cmd -> gimbal_climb_flag = GET_KEY_COUNT(VT03_KEY_PRESS_WITH_CTRL , KEY_F) % 2;//ctrl + G 

    }

}



void Gimbal_Reference(void)
{
    ;
}

float target_pitch = 0 ;
float current_pitch = 0;

float pid_pitch_p = 0;
float pid_pitch_i = 0;
float pid_pitch_d = 0;
//外环
float pid_pitch_p_out = 1;
float pid_pitch_i_out = 0;
float pid_pitch_d_out = 0;

float i_6020 = 0;

void Gimbal_Console(void)
{
    
		//test
//		gimbal_pitch_head->motor_controller.speed_PID->kp = pid_pitch_p ;
//		gimbal_pitch_head->motor_controller.speed_PID->ki = pid_pitch_i;
//		gimbal_pitch_head->motor_controller.speed_PID->kd = pid_pitch_d;

//		gimbal_pitch_head->motor_controller.angle_PID->kp = pid_pitch_p_out;
//		gimbal_pitch_head->motor_controller.angle_PID->ki = pid_pitch_i_out;
//		gimbal_pitch_head->motor_controller.angle_PID->kd = pid_pitch_d_out;
	
		
        gimbal_cmd -> current_angle_neck = gimbal_pitch_neck->receive_data.position;
		gimbal_cmd -> current_angle_yaw = INS.Yaw;
		gimbal_cmd -> current_angle_head = INS.Pitch;

    if( vs_aim_packet_from_nuc.yaw == 0 || vs_aim_packet_from_nuc.pitch == 0)//为0时即不传数据
    {
        gimbal_cmd -> auto_yaw =  INS.Yaw;
        gimbal_cmd -> auto_pitch_head = INS.Pitch;
    }
    else 
    {
        gimbal_cmd->auto_yaw = ( ( fabsf(vs_aim_packet_from_nuc.yaw) < PI ) == 1) ? vs_aim_packet_from_nuc.yaw : INS.Yaw;
        gimbal_cmd->auto_pitch_head = ( ( fabsf(vs_aim_packet_from_nuc.pitch) < 0.5f * PI ) == 1) ? vs_aim_packet_from_nuc.pitch : INS.Pitch;

    }


    //遥控器模式选择
    if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_REMOTE )
    {
        Gimbal_Enable();
        
        //脖子控制
        if(gimbal_cmd -> neck_state == NECK_LAY)
        {
            fire_enable_flag = 0;//允许拨弹
            //neck
            if( fabs(gimbal_cmd -> yaw_position - CHASSIS_GIMBAL_LAY_ZERO) < 0.5f  )
            {
                gimbal_cmd -> target_angle_neck = 1.1f;//缩头
                gimbal_pitch_neck -> transmit_data.velocity_des = 1.5f;
            }
            //head
            gimbal_cmd -> target_angle_yaw = INS.Yaw;
            gimbal_cmd -> target_angle_head = 0.03f;
        }

        else if(gimbal_cmd -> neck_state == NECK_STAND)
        {
            fire_enable_flag = 1;//允许拨弹

            gimbal_cmd -> target_angle_neck = GIMBAL_NECK_STAND;//抬头
            gimbal_pitch_neck -> transmit_data.velocity_des = 1.5f;
        }

        
       if( gimbal_cmd -> neck_state == NECK_STAND && gimbal_cmd -> auto_state == GIMBAL_MANUAL )
       {
				 
           gimbal_cmd -> target_angle_yaw += - rc_data->rc.rocker_r_ * REMOTE_YAW_SENSITIVITY;
           gimbal_cmd -> target_angle_head += - rc_data->rc.rocker_r1 * REMOTE_PITCH_SENSITIVITY;
       }
       else if( gimbal_cmd -> neck_state == NECK_STAND && gimbal_cmd -> auto_state == GIMBAL_AUTO )
       {
           gimbal_cmd -> target_angle_yaw = gimbal_cmd -> auto_yaw;
           gimbal_cmd -> target_angle_head = gimbal_cmd -> auto_pitch_head;
     }   

        //限幅
        if(gimbal_cmd -> target_angle_head < - 0.5 )
        {
            gimbal_cmd -> target_angle_head = 0.5;
        }
        else if(gimbal_cmd -> target_angle_head > 0.18 )
        {
            gimbal_cmd -> target_angle_head = 0.18;
        }
			
        DJI_Motor_Set_Ref(gimbal_pitch_head, gimbal_cmd -> target_angle_head);
        DM_Motor_SetTar(gimbal_pitch_neck, gimbal_cmd -> target_angle_neck);

        target_pitch = gimbal_cmd -> target_angle_head ;
        current_pitch = INS.Pitch;   

    }

    else if( gimbal_cmd -> gimbal_mode == GIMBAL_MODE_KEYBOARD )
    {
        //键鼠控制,待添加
        //脖子控制
        if( gimbal_cmd -> gimbal_climb_flag == 0)//不过隧道或爬坡 可加入底盘imu检测
        {
            Gimbal_Enable();
            // gimbal_cmd->gimbal_follow = 0;
            // gimbal_follow_chassis_angle_pid.output = 0;
            // gimbal_follow_chassis_angle_pid.i_out = 0;

            //脖子控制
            if(gimbal_cmd -> neck_state == NECK_LAY)
            {
                fire_enable_flag = 0;//允许拨弹
                //neck
                if( fabs(gimbal_cmd -> yaw_position - CHASSIS_GIMBAL_LAY_ZERO) < 0.5f  )
                {
                    gimbal_cmd -> target_angle_neck = 1.1f;//缩头
                    gimbal_pitch_neck -> transmit_data.velocity_des = 1.5f;
                }
                //head
                gimbal_cmd -> target_angle_yaw = INS.Yaw;
                gimbal_cmd -> target_angle_head = 0.03f;
            }

            else if(gimbal_cmd -> neck_state == NECK_STAND)
            {
                fire_enable_flag = 1;//允许拨弹

                gimbal_cmd -> target_angle_neck = GIMBAL_NECK_STAND;//抬头
                gimbal_pitch_neck -> transmit_data.velocity_des = 1.5f;
            }

            if( gimbal_cmd -> neck_state == NECK_STAND && gimbal_cmd -> auto_state == GIMBAL_MANUAL )
            {
								
                gimbal_cmd -> target_angle_yaw += - rc_data->mouse.x * KEYBOARD_YAW_SENSITIVITY;
                gimbal_cmd -> target_angle_head += - rc_data->mouse.y * KEYBOARD_PITCH_SENSITIVITY;
            }
            else if( gimbal_cmd -> neck_state == NECK_STAND && gimbal_cmd -> auto_state == GIMBAL_AUTO )
            {
                gimbal_cmd -> target_angle_yaw = gimbal_cmd -> auto_yaw;
                gimbal_cmd -> target_angle_head = gimbal_cmd -> auto_pitch_head;
            }   

            //限幅
            if(gimbal_cmd -> target_angle_head < - 0.5 )
            {
                gimbal_cmd -> target_angle_head = 0.5;
            }
            else if(gimbal_cmd -> target_angle_head > 0.18 )
            {
                gimbal_cmd -> target_angle_head = 0.18;
            }
                
            DJI_Motor_Set_Ref(gimbal_pitch_head, gimbal_cmd -> target_angle_head);
            DM_Motor_SetTar(gimbal_pitch_neck, gimbal_cmd -> target_angle_neck);

        }
        else if( gimbal_cmd -> gimbal_climb_flag == 1 )
        {
            fire_enable_flag = 0;//不允许拨弹和开摩擦轮
            gimbal_cmd -> target_angle_neck = 1.1f;//缩头
            gimbal_pitch_neck -> transmit_data.velocity_des = 1.5f;
            DM_Motor_SetTar(gimbal_pitch_neck, gimbal_cmd -> target_angle_neck);
					
            //yaw处理,向左负，向右正
			// 			float error_angle = gimbal_cmd -> yaw_position - CHASSIS_GIMBAL_LAY_ZERO;
            // if(error_angle > PI)
            // {
            //     error_angle -= 2 * PI;
            // }
            // else if(error_angle < -PI)
            // {
            //     error_angle += 2 * PI;
            // }

            // gimbal_cmd->gimbal_follow = PID_Position(&gimbal_follow_chassis_angle_pid, error_angle , 0.0f);
            // gimbal_cmd -> target_angle_yaw -= gimbal_cmd->gimbal_follow;						

            DJI_Motor_Disable(gimbal_pitch_head);
            gimbal_pitch_head->motor_controller.angle_PID->output = 0;
            gimbal_pitch_head->motor_controller.speed_PID->output = 0;

        }
        
        
    }

    else
    {
        gimbal_cmd -> target_angle_yaw = INS.Yaw;
        gimbal_cmd -> target_angle_head = INS.Pitch;
        //gimbal_cmd -> target_angle_neck = GIMBAL_NECK_ZERO;

        Gimbal_Disable();
    }

    i_6020 = gimbal_pitch_head->receive_data.real_current;

}

void Gimbal_Send_Cmd(void)
{
    DM_Motor_Control(NULL);
    DJI_Motor_Control(NULL);
}
