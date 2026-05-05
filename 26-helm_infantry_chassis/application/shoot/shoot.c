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

 #include "pid.h"
 #include "drv_motor.h"
 #include "DJI_motor.h"

 static shoot_cmd_t shoot_cmd_storage;
 shoot_cmd_t *shoot_cmd = &shoot_cmd_storage;//静态初始化，防止空指针

 DJI_motor_instance_t * shoot_2006_motor;


 PID_t shoot_2006_speed_pid = {
     .kp = 10.0f,
     .ki = 1.0f,
     .kd = 0.1f,
     .output_limit = 9000.0f,
     .integral_limit = 9000.0f,
     .dead_band = 0.0f,
 };

 motor_init_config_t shoot_2006_init = {
     .controller_param_init_config = {
         .angle_PID = NULL,
         .speed_PID = &shoot_2006_speed_pid,
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
         .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // Keep the 2006 trigger motor init unchanged.
         .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

         .angle_feedback_source = MOTOR_FEED,
         .speed_feedback_source = MOTOR_FEED,

         .feedforward_flag = FEEDFORWARD_NONE,
     },

     .motor_type = M2006,

     .can_init_config = {
         .can_handle = &hfdcan2,
         .tx_id = 0x07,
         .rx_id = 0x07,
     },
 };

void Shoot_Enable(void)
{
    if (shoot_2006_motor != NULL)
    {
        DJI_Motor_Enable(shoot_2006_motor);
    }
}

void Shoot_Disable(void)
{
    if (shoot_2006_motor != NULL)
    {
        DJI_Motor_Disable(shoot_2006_motor);

        if (shoot_2006_motor->motor_controller.speed_PID != NULL)
        {
            shoot_2006_motor->motor_controller.speed_PID->output = 0;
        }
    }
}

void Shoot_Init(void)
{
    shoot_2006_motor = DJI_Motor_Init(&shoot_2006_init);
}

// 处理异常
void Shoot_Handle_Exception( )
{
    ;
}


void Shoot_Set_Mode( )
{
    ;
}

// 设置目标量
void Shoot_Reference( )
{
    ;
}



// 计算控制量
void Shoot_Console( )
{
	
		float shoot_target = 0.0f;
		shoot_target = shoot_cmd -> shoot_frq;
		if(shoot_target < 0 )
		{
			shoot_target = -shoot_target;
		}
			
	
    if (shoot_2006_motor == NULL)
    {
        return;
    }

    switch(shoot_cmd->auto_state)
    {
        case SHOOT_STOP:
            Shoot_Disable();
        break;

        case SHOOT_MANUAL:
        {
            Shoot_Enable();
            DJI_Motor_Set_Ref(shoot_2006_motor, shoot_target);
        }
        break;

        case SHOOT_AUTO:
        {
            Shoot_Enable();
            DJI_Motor_Set_Ref(shoot_2006_motor, shoot_target);
            break;
        }   

        default:
        {
            DJI_Motor_Set_Ref(shoot_2006_motor, 0);
            Shoot_Disable();
            break;
        }
           
       
    }

}

// 发送控制量
void Shoot_Send_Cmd( )
{
    if (shoot_2006_motor != NULL)
    {
        DJI_Motor_Control(shoot_2006_motor);
    }
}