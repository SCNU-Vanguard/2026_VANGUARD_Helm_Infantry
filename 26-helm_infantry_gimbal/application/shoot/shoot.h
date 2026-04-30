/**
* @file shoot.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include <stdint.h>


/* 控制模式选择 */
typedef enum{

	SHOOT_MODE_STOP = 0,//完全失能
	SHOOT_MODE_REMOTE = 1,
	SHOOT_MODE_KEYBOARD = 2,
}shoot_mode_e;

/* 火控选择 */
typedef enum
{
	SHOOT_STOP = 0,
	SHOOT_MANUAL = 1,
	SHOOT_AUTO = 2,
}shoot_auto_e;

typedef struct
{
	/* data */
}__attribute__((packed)) shoot_behaviour_t;

typedef struct
{
	/* mode */
	shoot_mode_e mode;
	shoot_auto_e auto_state;

	/* data */
	float shoot_frq;//Hz 
	float shoot_v;//rpm

}__attribute__((packed)) shoot_cmd_t;


void Shoot_Enable(void);
void Shoot_Disable(void);
void Shoot_Init(void);
void Shoot_Set_All_Friction(int16_t speed);

extern shoot_cmd_t *shoot_cmd;
extern uint8_t fire_enable_flag;//允许拨弹标志位

#endif /* __SHOOT_H__ */
