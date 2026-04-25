/*
 * VPC.c
 * author: miracle-cloud
 * Created on: 2025��10��31��
 */

#include "VPC.h"
#include "Serial.h"
#include "INS.h"
#include "gimbal.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

#include "usbd_cdc_if.h"

extern Frame_header_t header;

uint8_t frame_buf[1024];

//缓冲区整体左移，类似队列操作，更新cdc_rx_len
static void Consume_CDC_Cache(uint16_t consume_len)
{
  taskENTER_CRITICAL();
	
  if (consume_len >= cdc_rx_len)
  {
      cdc_rx_len = 0;
  }
	else
	{
    memmove(cdc_rx_cache, &cdc_rx_cache[consume_len], cdc_rx_len - consume_len);
    cdc_rx_len -= consume_len;
  }
  taskEXIT_CRITICAL();
}

void VPC_Init(void)
{
  #if(Tongji == 1) 
	{
    VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
    VS_Receive_Packet_Init(&vs_aim_packet_from_nuc);

	}
	#elif(TianJin == 1)
	{
    header.sof = 0xA6; // 帧头
    header.crc8 = 0 ;// CRC8校验，暂时不计算

    auto_send_data.state = 0; //默认状�?1?7
    auto_send_data.autoaim = 1;
    auto_send_data.enemy_color = 0; //0 --- 红色
	}
	#endif
}

void VPC_UpdatePackets(void)
{
  #if(Tongji == 1) 
	{
    vs_aim_packet_to_nuc.head[0] = 'S';
    vs_aim_packet_to_nuc.head[1] = 'P';
    vs_aim_packet_to_nuc.mode = 1;
    vs_aim_packet_to_nuc.q[0] = INS.q[0];
    vs_aim_packet_to_nuc.q[1] = INS.q[1];
    vs_aim_packet_to_nuc.q[2] = INS.q[2];
    vs_aim_packet_to_nuc.q[3] = INS.q[3];
    vs_aim_packet_to_nuc.yaw = INS.Yaw;
    vs_aim_packet_to_nuc.yaw_vel = 0; 

    vs_aim_packet_to_nuc.pitch = INS.Pitch;
    vs_aim_packet_to_nuc.pitch_vel = 0;
    vs_aim_packet_to_nuc.bullet_speed = 22.9f; 
    vs_aim_packet_to_nuc.bullet_count = 0;  

	}
	#elif(TianJin == 1)
	{
    auto_send_data.curr_yaw = INS.Yaw;
		auto_send_data.curr_pitch = INS.Pitch;
		auto_send_data.curr_roll = INS.Roll;

	}
	#endif
}


void Choose_VPC_Type(void)
{
  taskENTER_CRITICAL();
	
  uint16_t cache_len = cdc_rx_len;
  if (cache_len == 0)
  {
  	taskEXIT_CRITICAL();
  	return;
  }
  
  memcpy(frame_buf, cdc_rx_cache, cache_len);
  
  taskEXIT_CRITICAL();
  
  ////////////////////////////  检测合法帧头？   ///////////////////////////////////
  uint16_t valid_head_idx = 0;
  while (valid_head_idx < cache_len - 1) 
  {
      if( (frame_buf[valid_head_idx] == 'S' && frame_buf[valid_head_idx+1] == 'P') )
      {
          break;
      }
      valid_head_idx++;
  }

  if (valid_head_idx > 0)
  {
      Consume_CDC_Cache(valid_head_idx);
      return; 
  }

  ////////////////////////////  判断帧头后对数据解包   ////////////////////////////////////////
  if (frame_buf[0] == 'S' && frame_buf[1] == 'P')
  {
    uint16_t frame_len = sizeof(vs_receive_packet_t);
      
    if (cache_len >= frame_len)
    {
      VS_UnPack_Data_ROS2(frame_buf, &vs_aim_packet_from_nuc, frame_len);
      Consume_CDC_Cache(frame_len);
    }
  }
}

