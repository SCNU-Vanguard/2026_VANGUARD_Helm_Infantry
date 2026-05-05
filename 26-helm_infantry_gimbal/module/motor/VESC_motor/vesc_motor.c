
#include "vesc_motor.h"


static uint8_t idx;
static VESC_motor_instance_t *vesc_motor_instances[VESC_MOTOR_CNT];


/* ---------------------------------- 相关函数 --------------------------*/
void VESC_Motor_Enable(VESC_motor_instance_t *instance)
{
    instance->motor_state_flag = MOTOR_ENABLE;
}

void VESC_Motor_Disable(VESC_motor_instance_t *instance)
{
    instance->motor_state_flag = MOTOR_DISABLE;
}

//电机离线回调函数
static void VESC_Motor_Lost_Callback(void *instance)
{
    VESC_motor_instance_t *motor = (VESC_motor_instance_t *) instance;
    motor->error_code |= MOTOR_LOST_ERROR;
    motor->receive_flag = 0;

    VESC_Motor_Disable(motor);
}

/*发送函数*/
/**
 ************************************************************************
 * @brief:      	Motor_ExCan_Crtl: 电机扩展CAN发送函数
 * @param[in]:   motor: 		指向VESC_motor_instance_t结构的指针，用于指定目标电机
 * @param[in]:   cmd_number: 命令编号，用于指定发送数据的类型（如设置占空比、设置转速等） , 目前只是用RPM,其他控制命令具体查看手册
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送速度控制命令
 ************************************************************************
 **/
void Motor_ExCan_Crtl(VESC_motor_instance_t *motor)
{
    /*Data Length :4 */
    motor->motor_can_instance->tx_header.DataLength = FDCAN_DLC_BYTES_4;//默认发送4字节，个别发送8字节

    switch(motor->command_number)
    {
        
        case CAN_PACKET_SET_DUTY://设置PWM占空比 -1 to 1( -100% ~ 100% ) , Duty Cyde* 100 000
            break;

        case CAN_PACKET_SET_CURRENT://设置控制电流(单位mA , -200 000 ~ 200 000) Current * 1000
            break;

        case CAN_PACKET_SET_CURRENT_BRAKE://设置制动电流(单位mA , -200 000 ~ 200 000)，Current * 1000 , 限制最大值?
            break;

        case CAN_PACKET_SET_RPM://设置目标转速(单位rpm , ) , RPM * 100
            motor->motor_can_instance->tx_header.Identifier = (uint32_t) ( CAN_PACKET_SET_RPM << 8) | motor->motor_can_instance->tx_id;//根据电机实例的CAN配置设置发送ID
            motor->motor_can_instance->tx_header.DataLength = FDCAN_DLC_BYTES_4;

            int32_t send_speed_rpm = motor->transmit_data.target_speed_rpm * 7;//发送的为磁对转速，14磁极7磁对
            //最大实际转速不超10000rpm
            if(send_speed_rpm > 70000)
            {
                send_speed_rpm = 70000;
            }
            else if(send_speed_rpm < -70000)
            {
                send_speed_rpm = -70000;
            }

            motor->motor_can_instance->tx_buff[3] =  send_speed_rpm & 0xFF;//小端发送
            motor->motor_can_instance->tx_buff[2] = (send_speed_rpm >> 8 ) & 0xFF;
            motor->motor_can_instance->tx_buff[1] = (send_speed_rpm >> 16) & 0xFF;
            motor->motor_can_instance->tx_buff[0] = (send_speed_rpm >> 24) & 0xFF;

            CAN_Transmit(motor->motor_can_instance, 1);//发送数据,超时时间为5ms
            break;

        case CAN_PACKET_SET_POS ://设置目标位置(单位？) , Position * 100
            break;

        case CAN_PACKET_SET_CURRENT_REL://设置相对电流 , Relative Current * 100 000
            break;

        case CAN_PACKET_SET_CURRENT_BRAKE_REL://设置相对制动电流 , Relative Brake Current * 100 000
            break;

        case CAN_PACKET_CONF_CURRENT_UMITS ://设置电流限制 , Max Current Limit * 1000 , Min Current Limit * 1000
            break;

        case CAN_PACKET_CONF_STORE_CURRENT_LIMITS ://存储当前电流限制到Flash
            break;

        case CAN_PACKET_CONF_CURRENT_UMITS_IN : //设置FOC控制的ERPMS参数 , FOC ERPMS * 1000
            break;

        case CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN : //存储当前FOC ERPMS参数到E2PROM
            break;

        default:
            break;
    }
}


static void VESC_Motor_RxCallback(CAN_instance_t *motor_can)//电机接收回调函数
{
    uint8_t *rxbuff                   = motor_can->rx_buff;
	VESC_motor_instance_t *motor        = (VESC_motor_instance_t *) motor_can->id;//将can实例中保存的电机地址 转换成 电机实例的指针

    motor->receive_number = motor_can->rx_id >> 8;//接收编号,根据ID的高位判断命令类型

    // if(motor_can -> rx_header.IdType != FDCAN_EXTENDED_ID || motor_can->rx_len != 8)
    // {
    //     return;//非扩展帧,暂不处理
    // }

    switch(motor->receive_number) //根据接收ID的高位判断命令类型
    {
        case CAN_PACKET_STATUS://接收电机状态数据
            motor->receive_data.Duty_Cycle = ( (int16_t)(rxbuff[6] | (rxbuff[7] << 8)) ) / 1000 ;
            motor->receive_data.TotalCurrent = ( (int16_t)(rxbuff[4] | (rxbuff[5] << 8)) ) / 10 ;
            motor->receive_data.Speed_RPM =  (int32_t)(  rxbuff[3] | (rxbuff[2] << 8) | (rxbuff[1] << 16) |(rxbuff[0] << 24) ) ;
                                                         
            motor->receive_flag = 1;
            //motor->error_code &= ~MOTOR_LOST_ERROR; //清除离线错误状态
            //Supervisor_Reset(motor->supervisor);//重置守护线程计数器
            motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
            break;

        case CAN_PACKET_STATUS_2://接收电机状态数据2 , 包含温度和位置等 ，F01
            motor->receive_data.temperature = ( (int16_t)(rxbuff[2] | (rxbuff[3] << 8)) ) / 10 ;
            motor->receive_data.PID_Pos = ( (int16_t)(rxbuff[6] | (rxbuff[7] << 8)) ) / 50 ;
            //Supervisor_Reset(motor->supervisor);//重置守护线程计数器
            motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
            break;

        default:
            break;
    }
}

VESC_motor_instance_t *VESC_Motor_Init(motor_init_config_t *config)
{
    VESC_motor_instance_t *instance = (VESC_motor_instance_t *) malloc(sizeof(VESC_motor_instance_t));
    memset(instance, 0, sizeof(VESC_motor_instance_t));

    instance->motor_type     = config->motor_type;
    instance->motor_settings = config->controller_setting_init_config;
    instance->motor_controller.current_PID = PID_Init(config->controller_param_init_config.current_PID);
    instance->motor_controller.speed_PID   = PID_Init(config->controller_param_init_config.speed_PID);
    instance->motor_controller.angle_PID   = PID_Init(config->controller_param_init_config.angle_PID);
    instance->motor_controller.torque_PID  = PID_Init(config->controller_param_init_config.torque_PID);

    //电机控制闭环时的非电机本身反馈数据指针
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    //电机控制闭环时的前馈控制器或前馈控制量指针
    instance->motor_controller.torque_feedforward_ptr = config->controller_param_init_config.torque_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr  = config->controller_param_init_config.speed_feedforward_ptr;


    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = VESC_Motor_RxCallback; // set callback
    config->can_init_config.id                  = instance;              // set id,eq to address(it is IdTypentity)
    instance->motor_can_instance                = CAN_Register(&config->can_init_config);

    //////////////////////////////////////          扩展帧配置          ////////////////////////////////////////////////////////////////
    instance->motor_can_instance->tx_header.IdType = FDCAN_EXTENDED_ID;//VESC电调通信使用扩展ID
    instance->motor_can_instance->tx_header.DataLength = FDCAN_DLC_BYTES_4;



    // 注册守护线程
    supervisor_init_config_t supervisor_config = {
        .handler_callback = VESC_Motor_Lost_Callback,
        .owner_id = instance,
        .reload_count = 20, // 20ms未收到数据则丢失
    };
    instance->supervisor = Supervisor_Register(&supervisor_config);
    instance->error_code = MOTOR_ERROR_NONE;

    DWT_GetDeltaT(&instance->feed_cnt);//获取当前时间戳

    VESC_Motor_Disable(instance);
    vesc_motor_instances[idx++] = instance;
    return instance;

}

