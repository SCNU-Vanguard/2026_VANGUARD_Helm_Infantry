/**
* @file chassis.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>

#include "robot_frame_config.h"



//驱动轮电调ID
#define CHASSIS_DRIVE_HELM_1 0 //舵轮1
#define CHASSIS_DRIVE_HELM_2 2 //舵轮2

#define CHASSIS_DRIVE_OMNI_1 1 //全向轮1
#define CHASSIS_DRIVE_OMNI_2 3 //全向轮2

typedef enum
{
	CHASSIS_STOP = 0,
	CHASSIS_FOLLOW_GIMBAL = 1,//底盘跟随模式
	CHASSIS_MOVE = 2,
} chassis_mode_e;

typedef struct
{
	/* data */
}__attribute__((packed)) chassis_behaviour_t;

typedef struct
{
	float vx;//线速度
	float vy;
	float vw;//底盘角速度，rad/s,顺时针为正

	float vw_follow;//底盘跟随时加速度

	chassis_mode_e chassis_mode;

}__attribute__((packed)) chassis_cmd_t;

extern void Chassis_Init(void);
void Chassis_Enable(void);	
void Chassis_Disable(void);
void Chassis_Ctrl(void);

#endif /* __CHASSIS_H__ */
