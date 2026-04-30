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

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "balance_chassis.h"
#endif


/* 控制模式选择 */
typedef enum{

	CHASSIS_MODE_STOP = 0,//完全失能
	CHASSIS_MODE_REMOTE = 1,
	CHASSIS_MODE_KEYBOARD = 2,
}chassis_mode_e;

/* 是否跟随 */
typedef enum
{
	CHASSIS_STOP = 0,
	CHASSIS_FOLLOW_GIMBAL = 1,//底盘跟随模式
	CHASSIS_MOVE = 2,
}chassis_move_mode_e;

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

	chassis_move_mode_e move_mode;//用于传下底盘
	chassis_mode_e chassis_mode;//control_mode

}__attribute__((packed)) chassis_cmd_t;

extern chassis_cmd_t *chassis_cmd;

#endif /* __CHASSIS_H__ */
