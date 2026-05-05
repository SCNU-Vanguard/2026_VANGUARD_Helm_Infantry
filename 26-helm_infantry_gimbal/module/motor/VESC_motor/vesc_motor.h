#ifndef _VESE_MOTOR_H__
#define _VESE_MOTOR_H__

/*bsp.h*/
#include "bsp_can.h"
#include "bsp_dwt.h"

/*module.h*/
#include "drv_motor.h"
#include "defense_center.h"


#define VESC_MOTOR_CNT 4//根据实际电机数量修改

/*          VESC Extended Frame command numbers     */
/*29 bit ID的第二个字节开始*/
#define CAN_PACKET_SET_DUTY  0   
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM 3
#define CAN_PACKET_SET_POS 4
#define CAN_PACKET_FILL_RX_BUFFER 5
#define CAN_PACKET_FILL_RX_BUFFER_LONG 6
#define CAN_PACKET_PROCESS_RX_BUFFER 7
#define CAN_PACKET_PROCESS_SHORT_BUFFER 8
#define CAN_PACKET_STATUS 9
#define CAN_PACKET_SET_CURRENT_REL 10
#define CAN_PACKET_SET_CURRENT_BRAKE_REL 11
#define CAN_PACKET_SET_CURRENT_HANDBRAKE 12
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_REL 13
#define CAN_PACKET_STATUS_2 14
#define CAN_PACKET_STATUS_3 15
#define CAN_PACKET_STATUS_4 16
#define CAN_PACKET_PING 17
#define CAN_PACKET_PONG 18
#define CAN_PACKET_DETECT_APPLY_ALL_FOC 19
#define CAN_PACKET_DETECT_APPLY_ALL_FOC_RES 20
#define CAN_PACKET_CONF_CURRENT_UMITS 21
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS 22
#define CAN_PACKET_CONF_CURRENT_UMITS_IN 23
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN 24
#define CAN_PACKET_CONF_FOC_ERPMS 25
#define CAN_PACKET_CONF_STORE_FOC_ERPMS 26
#define CAN_PACKET_STATUS_5 27


typedef struct 
{
    int16_t Duty_Cycle; // 占空比
    int16_t TotalCurrent; // 电流
    int32_t Speed_RPM; // 转速

    int16_t temperature; // 温度 ℃
    int16_t PID_Pos; // 位置,unit?
} VESC_motor_callback_t;

typedef struct 
{
    /* send_data */
    int32_t target_speed_rpm;


    /* 暂未使用 */
    float target_duty; // 目标占空比, -1 ~ 1, 对应 -100% ~ 100%
    int32_t target_current; // 目标电流, mA, -200000 ~ 200000
    int32_t target_brake_current; // 目标制动电流, mA, -200000 ~ 200000

    int32_t target_position; //单位？
    float relative_current; //相对电流, -1 ~ 1, 对应 -100% ~ 100%
    float relative_brake_current; //相对制动电流, -1 ~ 1, 对应 -100% ~ 100%
    int32_t max_current_limit; //*1000
    int32_t min_current_limit; //*1000
    int32_t max_input_current_limit; //*1000
    int32_t min_input_current_limit; //*1000

}VESC_motor_fillmessage_t;


typedef struct
{
    motor_working_type_e motor_state_flag; //电机状态标志，用于控制电机使能或失能
    
	motor_controller_t motor_controller;    // 电机控制器
    motor_control_setting_t motor_settings; // 电机设置
    motor_model_e motor_type; // 电机类型

    CAN_instance_t *motor_can_instance; // 电机CAN实例 , 里面的rx_id决定读取哪些信息，框架限制，暂时只能读取一种

    motor_error_e error_code;//电机错误状态

    VESC_motor_callback_t receive_data; // 电机反馈数据
    VESC_motor_fillmessage_t transmit_data; // 电机设定数据

    uint8_t command_number; //当前发送的命令编号,用于发送函数判断发送数据类型
    uint8_t receive_number; //当前接收的命令编号
    
    int16_t receive_flag;   

    supervisor_t *supervisor;

    uint32_t feed_cnt;
    float dt;

    // 发送编号位置设置
    uint8_t message_num;
} VESC_motor_instance_t;


VESC_motor_instance_t *VESC_Motor_Init(motor_init_config_t *config);
void Motor_ExCan_Crtl(VESC_motor_instance_t *motor);


#endif /* _VESE_MOTOR_H__ */
