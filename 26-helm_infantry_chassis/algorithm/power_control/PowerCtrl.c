
#include "PowerCtrl.h"
#include "chassis.h"
#include "referee_task.h"

float P_cal[4] = {0}; // 电机功率模型计算出的功率
float P_total = 0;   // 测量总功率
float P_total_temp = 0; // 临时总功率

float P_cmd[4] = {0}; // 以最大功率上线分配所得的功率
float chassis_max_power = 50; // 底盘最大功率 , 5~8W 的误差 

float I_temp[4] = {0};

float I_test[4];
float P_test;

float speed_rad;
float current;

//与大P分配有关的参数---------
float error[4] = {0};
float error_sum = 0.0f;
float weight[4] = {0};
float k_coe = 0.0f;
float E_lower = 5.0f;   // 误差下阈值（需要调）
float E_upper = 30.0f;  // 误差上阈值（需要调）
//--------------------------

// power_control_mode_e Power_Control_Mode = SUPERCAP_CONTROL;

static float power_limit_offset_table[10] = {0, 0, 0, 0, 0}; // 功率限制偏置表,由于软件功控在不同功率水平下的限制效果不同，故打表以让软件功控尽量贴合底盘最大功率

static void Power_Decision(void)
{
	if(referee_outer_info->RobotPerformance.robot_level < 1 || referee_outer_info->RobotPerformance.robot_level > 10) // 不在正常等级认为机器人没有连接裁判系统，无法获取功率限制等信息，直接返回
	{
		return;
	}
	// if((referee_outer_info->BuffMusk.remaining_energy>>6)&1U)	//剩余能量不足1%时，底盘功率为35w
	// {
	// 	chassis_max_power = 35.0f ;//- 5.0f;后续偏置待调
	// }
	if((chassis_super_capacitor->receive_data.statusCode&0x03) == 0 && chassis_super_capacitor->LostFlag == false) //超电正常
	{
		if(chassis_super_capacitor->receive_data.capEnergy>38)  //低于10%可能会出现充不进电；故大于15%时才使用较大功率 255*0.15=38.25
		{
			chassis_max_power = referee_outer_info->RobotPerformance.chassis_power_limit + power_limit_offset_table[referee_outer_info->RobotPerformance.robot_level-1]
								+ 25.0f; //比功率限制大25W，2000j/25W=80s，满功率状态下预估可以使用一分半左右
		}
		else
		{
			chassis_max_power = referee_outer_info->RobotPerformance.chassis_power_limit + power_limit_offset_table[referee_outer_info->RobotPerformance.robot_level-1]
								- 10.0f; //比功率限制小10W，2000j/10W=200s，满功率状态下预估三分钟左右充满超电
		}
	}
	else //超电异常
	{
		chassis_max_power = referee_outer_info->RobotPerformance.chassis_power_limit + power_limit_offset_table[referee_outer_info->RobotPerformance.robot_level-1];
	}
}

void chassis_power_control()
{
	//Power_Decision();
   
	float P_cal_helm[2] = {0};
	float P_cmd_helm[2] = {0};
	float I_temp_helm[2] = {0};
	float helm_weight = 0.0f;

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float delta = 0.0f;
    float E_k = 0.0f;

//功率模型
    for (uint8_t i = 0; i < 4; i++)
    {
		if (chassis_drive[i] == NULL) 
		{
			// 记录或跳过
			continue;
		}
		speed_rad = chassis_drive[i]->receive_data.speed;        // motor_speed ---> rad
		current = chassis_drive[i] ->transmit_data.current;

		P_cal[i] =  K0 * current *speed_rad +
					K1 * speed_rad * speed_rad +  
					K2 * current * current + 
					constant;//静态功耗

		if(P_cal[i] < 0) // 负功率不计入（过渡性）
			continue;

		P_total_temp += P_cal[i];//输出总功率
	}

	for (uint8_t i = 0; i < 2; i++)
	{
		if (chassis_helm[i] == NULL)
		{
			continue;
		}
		speed_rad = chassis_helm[i]->receive_data.speed;
		current = chassis_helm[i]->transmit_data.current;

		P_cal_helm[i] = K0_6020 * current *speed_rad +
						K1_6020 * speed_rad * speed_rad +
						K2_6020 * current * current +
						constant;
		if(P_cal_helm[i] < 0)
			continue;

		P_total_temp += P_cal_helm[i];
	}

	P_total = P_total_temp;//用于观察计算出来的底盘总功率
	P_total_temp = 0;//防止累加


    if(P_total > chassis_max_power)//总功率超过
    {
		if(P_total > chassis_max_power)
		{

			for(uint8_t i = 0; i < 4; i++)
			{
				weight[i] = (chassis_max_power ) / P_total;
			}

			// 重新分配功率

			for(uint8_t i = 0; i < 4; i++)
			{
				P_cmd[i] = P_cal[i] * weight[i];
			}
			// helm_weight = (chassis_max_power * 0.5) / P_total;
			// for(uint8_t i = 0; i < 2; i++)
			// {
			// 	P_cmd_helm[i] = P_cal_helm[i] * helm_weight;
			// }

			// 根据 P_cmd 反算电流

			for(uint8_t i = 0; i < 4; i++)
			{
				speed_rad = chassis_drive[i]->receive_data.speed; 
				current = chassis_drive[i] ->transmit_data.current;

				float a = K2;
				float b = K0 * speed_rad;
				float c = K1 * speed_rad * speed_rad + constant - P_cmd[i];

				float delta = b*b - 4*a*c;

				if(delta < 0)
					continue;
				
				if(current > 0)
					I_temp[i] = (-b + sqrtf(delta)) / (2*a);
				else
					I_temp[i] = (-b - sqrtf(delta)) / (2*a);

				// 电流限幅
				if(I_temp[i] > 16000.0f)
					I_temp[i] = 16000.0f;
				if(I_temp[i] < -16000.0f)
					I_temp[i] = -16000.0f;

				chassis_drive[i]->transmit_data.current = I_temp[i];
			}

			// for(uint8_t i = 0; i < 2; i++)
			// {
			// 	if (chassis_helm[i] == NULL)
			// 	{
			// 		continue;
			// 	}

			// 	speed_rad = chassis_helm[i]->receive_data.speed; 
			// 	current = chassis_helm[i] ->transmit_data.current;

			// 	float a = K2_6020;
			// 	float b = K0_6020 * speed_rad;
			// 	float c = K1_6020 * speed_rad * speed_rad + constant - P_cmd_helm[i];

			// 	float delta = b*b - 4*a*c;

			// 	if(delta < 0)
			// 		continue;
				
			// 	if(current > 0)
			// 		I_temp_helm[i] = (-b + sqrtf(delta)) / (2*a);
			// 	else
			// 		I_temp_helm[i] = (-b - sqrtf(delta)) / (2*a);

			// 	if(I_temp_helm[i] > 20000.0f)
			// 		I_temp_helm[i] = 20000.0f;
			// 	if(I_temp_helm[i] < -20000.0f)
			// 		I_temp_helm[i] = -20000.0f;

			// 	chassis_helm[i]->transmit_data.current = I_temp_helm[i];
			// }
		}
    }
	
	I_test[0] = (float)abs(chassis_drive[0]->transmit_data.current);
	I_test[1] = (float)abs(chassis_drive[1]->transmit_data.current);
	I_test[2] = (float)abs(chassis_drive[2]->transmit_data.current);
	I_test[3] = (float)abs(chassis_drive[3]->transmit_data.current);
	
////////////////////////////////////////////////////////////////////////////////////////////调试使用，后面可以注释这一段
	for(uint8_t i = 0;i < 4;i++)
	{
		speed_rad = chassis_drive[i]->receive_data.speed; //motor_list[i]->data.speed ---> rpm
		current = chassis_drive[i] -> transmit_data.current;

        P_cal[i] =  K0 * current *speed_rad +
					K1 * speed_rad * speed_rad +  
					K2 * current * current + 
					constant;//静态功耗

        if(P_cal[i] < 0) // 负功率不计入（过渡性）
            continue;

        P_total_temp += P_cal[i];//输出总功率
	}

	// for(uint8_t i = 0;i < 2;i++)
	// {
	// 	if (chassis_helm[i] == NULL)
	// 	{
	// 		continue;
	// 	}
	// 	speed_rad = chassis_helm[i]->receive_data.speed;
	// 	current = chassis_helm[i] -> transmit_data.current;

    //     P_cal_helm[i] =  K0_6020 * current *speed_rad +
	// 					K1_6020 * speed_rad * speed_rad +
	// 					K2_6020 * current * current + 
	// 					constant;

    //     if(P_cal_helm[i] < 0)
    //         continue;

    //     P_total_temp += P_cal_helm[i];
	// }
	
	P_test = P_total_temp;//用于观察功率控制后的底盘总功率
	P_total_temp = 0;//防止累加
////////////////////////////////////////////////////////////////////////////////////////////
}

// void Power_Control()
// {
// 	if(Power_Control_Mode == SUPERCAP_CONTROL && SuperCap_Data->StatusCode == 3)//错误，不可恢复
// 	{
// 		Power_Control_Mode = CODE_CONTROL;
// 		SuperCap_SystemRestart(POWER_LIMIT, ENERGY_BUFFER);//重启超电，尝试一次，不可持续重启
// 	}
	
// 	if(SuperCap_Data->StatusCode == 2 || SuperCap_Data->StatusCode == 1)//错误，可恢复
// 	{
// 		Power_Control_Mode = CODE_CONTROL;
// 		SuperCap_ClearError(POWER_LIMIT, ENERGY_BUFFER);//清除错误
// 	}
	
// 	if(SuperCap_Data->StatusCode == 0)
// 	{
// 		if(SuperCap_Data->CapEnergy < 64 && Power_Control_Mode == SUPERCAP_CONTROL)
// 		{
// 			Power_Control_Mode = CODE_CONTROL;
// 		}
// 		else if(SuperCap_Data->CapEnergy > 128 && Power_Control_Mode == CODE_CONTROL)
// 		{
// 			Power_Control_Mode = SUPERCAP_CONTROL;
// 		}
// 	}
	
// 	if(Power_Control_Mode == CODE_CONTROL)
// 	{
// 		if(chassis_cmd.mode == SPIN)
// 		{
// 			chassis_max_power = 55;
// 		}
// 		else
// 		{
// 			chassis_max_power = 40;
// 		}
// 		if(SuperCap_Forced_Use_Flag)
// 		{
// 			chassis_max_power = 55;
// 			if(SuperCap_Data->CapEnergy < 10)
// 			{
// 				SuperCap_Forced_Use_Flag = 0;
// 			}
// 		}
		
// //		chassis_power_control();//软件功控
// 	}
// }
