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

#include <stdint.h>
#include "DM_motor.h"

typedef enum{

	GIMBAL_MODE_STOP = 0,//完全使能
	GIMBAL_MODE_MOVE = 1,
	GIMBAL_MODE_AUTO = 2,

}gimbal_mode_e;


typedef struct
{
	/* data */

}__attribute__((packed)) gimbal_behaviour_t;

typedef struct
{
	/* data */
	float target_angle;//rad
	uint8_t gimbal_mode; //mode



}__attribute__((packed)) gimbal_cmd_t;

/*函数*/
void Gimbal_DM6006_Init(void);
void Gimbal_DM6006_Ctrl(void);

/*变量*/
extern DM_motor_instance_t *gimbal_dm6006;//云台电机
extern gimbal_cmd_t *gimbal_cmd;//云台相关控制指令

#endif /* __GIMBAL_H__ */
