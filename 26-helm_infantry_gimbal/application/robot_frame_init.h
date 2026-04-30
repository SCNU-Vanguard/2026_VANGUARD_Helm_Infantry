/**
* @file robot_frame_init.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_INIT_H__
#define __ROBOT_FRAME_INIT_H__

#include "remote_vt03.h"

extern void Robot_Frame_Init(void);
extern void Robot_Frame_Update_Keyboard_Mode(void);

extern VT03_ctrl_t *rc_data;
extern uint8_t key_flag;
extern uint8_t keyboard_enable_flag;

#define ROBOT_FRAME_KEYBOARD_UPDATE() Robot_Frame_Update_Keyboard_Mode()
#define ROBOT_FRAME_KEYBOARD_ENABLED() ( ((key_flag) != 0U) && ((keyboard_enable_flag) != 0U) )

#define GET_KEY_COUNT(key_flag, key_index) (rc_data->key_count[key_flag][(key_index)])
#endif /* __ROBOT_FRAME_INIT_H__ */
