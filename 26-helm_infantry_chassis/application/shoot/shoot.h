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


typedef enum{
	SHOOT_MODE_STOP = 0,
	SHOOT_MODE_MANUAL = 1,
	SHOOT_MODE_AUTO = 2,

}shoot_mode_e;

typedef struct
{
	/* data */
}__attribute__((packed)) shoot_behaviour_t;

typedef struct
{
	/* mode */
	shoot_mode_e mode;

	/* data */
	float shoot_frq;//Hz 
	float shoot_v;//rpm

}__attribute__((packed)) shoot_cmd_t;


// void Shoot_Enable(void);
// void Shoot_Disable(void);
// void Shoot_Init(void);
// void Shoot_Set_All_Friction(int16_t speed);

extern shoot_cmd_t *shoot_cmd;

#endif /* __SHOOT_H__ */
