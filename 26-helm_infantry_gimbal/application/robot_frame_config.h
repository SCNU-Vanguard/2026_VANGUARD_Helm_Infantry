/**
* @file robot_frame_config.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_CONFIG_H__
#define __ROBOT_FRAME_CONFIG_H__

////////////////////////////////// 宏定义函数 /////////////////////////////////

#define USER_LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))


////////////////////////////////// 宏定义函数 /////////////////////////////////


////////////////////////////////// 自定义参数 /////////////////////////////////

#define RC_NOW 0
#define RC_LAST 1

//注意，根据rc_data.gear_shift定义
#define KEYBOARD_MODE 0
#define REMOTE_MODE 1
#define STOP_MODE 2 //整体使能

/*底盘相关参数*/
#define CHASSIS_GM6020_ZERO_1  0.0f //rad  定义软件零点,底盘轮子正向前
#define CHASSIS_GM6020_ZERO_2  0.0f //

#define CHASSIS_FORWARD_ZERO   0.0f //底盘正向向前时云台电机零点 rad

#define CHASSIS_RADIUS         0.0f //底盘半径 m
#define OMNI_WHEEL_RADIUS      0.0f //全向轮半径 m
#define HELM_WHEEL_RADIUS      0.0f //舵轮半径 m

#define OMNI_REDUCTION_RATIO   0.0f //全向轮减速比
#define HELM_REDUCTION_RATIO    19.20321f //舵轮减速比 3591.0f/187.0f


/*射击相关参数*/
#define SHOOT_V 8000  //射击速度 rpm
#define REMOTE_SHOT_SENSITIVITY 0.02f //射频


/* 限幅值 */
#define CHASSIS_MAX_VX         0.0f //底盘最大线速度 m/s
#define CHASSIS_MAX_VY         0.0f
#define CHASSIS_MAX_VW         0.0f //底盘最大角速度 rad/s
#define CHASSIS_MAX_VW_FOLLOW  0.0f //底盘跟



/* 灵敏度 */
#define REMOTE_CHASSIS_SENSITIVITY_VX 0.001f //底盘线速度灵敏度
#define REMOTE_CHASSIS_SENSITIVITY_VY 0.001f
#define REMOTE_CHASSIS_SENSITIVITY_VW 1.0f 

#define REMOTE_YAW_SENSITIVITY         0.001f //云台yaw轴灵敏度
#define REMOTE_PITCH_SENSITIVITY       0.001f //云台pitch轴灵敏度

#define REMOTE_SHOOT_SENSITIVITY       0.001f //射击频率灵敏度

#define KEYBOARD_CHASSIS_SENSITIVITY_VX 0.0f //底盘线速度灵敏度
#define KEYBOARD_CHASSIS_SENSITIVITY_VY 0.0f
#define KEYBOARD_CHASSIS_SENSITIVITY_VW 0.0f

#define KEYBOARD_YAW_SENSITIVITY         0.001f //云台yaw轴灵敏度
#define KEYBOARD_PITCH_SENSITIVITY       0.001f //云台pitch轴灵敏度


////////////////////////////////// 自定义参数 /////////////////////////////////


////////////////////////////////// 单位转换  /////////////////////////////////
#define RAD_2_RPM  9.549296586f // rad/s to rpm 60.0f/(2*PI)
#define ECD_2_RAD  7.6699e-4f   // 2pi / 8192.0f,编码器值转弧度


#define RAD_2_DEGREE 57.2957795f    //rad/s to °/s 180/pi
#define DEGREE_2_RAD 0.01745329252f //°/s to rad/s pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       //rpm to °/s ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f //rpm to rad/s ×2pi/60sec

#define BMI088_Frame 1

////////////////////////////////// 单位转换  /////////////////////////////////

#endif /* __ROBOT_FRAME_CONFIG_H__ */
