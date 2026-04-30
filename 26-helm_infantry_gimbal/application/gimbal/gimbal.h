/**
* @file gimbal.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__


/* 控制模式选择 */
typedef enum{

	GIMBAL_MODE_STOP = 0,//完全失能
	GIMBAL_MODE_REMOTE = 1,
	GIMBAL_MODE_KEYBOARD = 2,

}gimbal_mode_e;

typedef enum
{
	NECK_LAY = 0,//缩头
	NECK_STAND = 1,//抬头
}gimbal_neck_e;

typedef enum
{
	GIMBAL_MANUAL = 0,
	GIMBAL_AUTO = 1,
	//GIMBAL_AUTO_SHOT = 2,
}gimbal_auto_e;


typedef struct
{
	/* data */

}__attribute__((packed)) gimbal_behaviour_t;

typedef struct
{
	/* mode */
	gimbal_mode_e gimbal_mode; //control_mode
	gimbal_neck_e neck_state; //脖子状态
	gimbal_auto_e auto_state; //自动模式状态

	uint8_t gimbal_climb_flag;//云台使能标志位 , 只用于过隧道
	/* data */
	float target_angle_head;//rad
	float current_angle_head;

	float target_angle_neck;//rad

	float target_angle_yaw;//rad
	float current_angle_yaw;//上位机传下的IMU角度 rad
	float current_yaw_acc;//上位机传下的IMU角加速度 rad/s

	float auto_yaw;//上位机传入
	float auto_pitch_head;

}__attribute__((packed)) gimbal_cmd_t;

extern gimbal_cmd_t * gimbal_cmd;

void Gimbal_Init(void);

#endif /* __GIMBAL_H__ */
