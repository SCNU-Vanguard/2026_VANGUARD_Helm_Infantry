#ifndef __CONFIG_H__
#define __CONFIG_H__


/*底盘相关参数*/
#define CHASSIS_GM6020_ZERO_1  0.0f //rad  定义软件零点,底盘轮子正向前
#define CHASSIS_GM6020_ZERO_2  0.0f //

#define CHASSIS_FORWARD_ZERO   0.0f //底盘正向向前时云台电机零点 rad

#define CHASSIS_RADIUS         0.0f //底盘半径 m
#define OMNI_WHEEL_RADIUS      0.0f //全向轮半径 m
#define HELM_WHEEL_RADIUS      0.0f //舵轮半径 m

#define OMNI_REDUCTION_RATIO   0.0f //全向轮减速比
#define HELM_REDUCTION_RATIO    19.20321f //舵轮减速比 3591.0f/187.0f


/* 限幅值 */
#define CHASSIS_MAX_VX         0.0f //底盘最大线速度 m/s
#define CHASSIS_MAX_VY         0.0f
#define CHASSIS_MAX_VW         0.0f //底盘最大角速度 rad/s
#define CHASSIS_MAX_VW_FOLLOW  0.0f //底盘跟


#endif /* __CONFIG_H__ */
