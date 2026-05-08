#include "rs485.h"
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include "gimbal.h"
#include "defense_center.h"
#include "CRC.h"

#define FRAME_HEADER 0xA5 // 帧头
#define FRAME_TAILER 0x5A // 帧尾

Tx_packed_t uart2_tx_message; // 发送数据结构体
Rx_packed_t uart2_rx_message; // 接收数据结构�?

uint8_t uart2_receive_buffer[sizeof(Rx_packed_t) * 3]; // 接收缓冲�?
uint8_t uart2_transmit_buffer[sizeof(Tx_packed_t)];    // 发送缓冲区

uint8_t ready_to_transmit = 0; // 发送就绪标志位
uint8_t ready_to_receive = 1;  // 接收就绪标志�?

uint8_t uart2_status = 0;       // 状态标志位
uint32_t last_uart2_uwTick = 0; // 上次发送时间戳

uint16_t uart2_buffer_length = 0;
uint8_t uart2_current_byte = 0;

float cnttttt = 0;
uint8_t rs485_status = 0;
uint32_t rs485_cnt ;

static UART_HandleTypeDef *rs485_uart_handle;
static supervisor_t *rs485_supervisor_instance;
static uint8_t rs485_online_flag = 0;

static void RS485_Set_Offline_Command(void)
{
    memset(&uart2_rx_message, 0, sizeof(uart2_rx_message));
    uart2_rx_message.frame_header = FRAME_HEADER;
    uart2_rx_message.frame_tailer = FRAME_TAILER;
}

static void RS485_Lost_Callback(void *id)
{
    rs485_online_flag = 0;
    memset(uart2_receive_buffer, 0, sizeof(uart2_receive_buffer));
    uart2_buffer_length = 0;
    RS485_Set_Offline_Command();

    if (rs485_uart_handle != NULL)
    {
        HAL_UART_Receive_IT(rs485_uart_handle, &uart2_current_byte, 1);
    }
}

void RS485_Init(UART_HandleTypeDef *uart_handle)
{
    rs485_uart_handle = uart_handle;
    RS485_Set_Offline_Command();

    supervisor_init_config_t supervisor_conf = {
        .reload_count = 20,
        .handler_callback = RS485_Lost_Callback,
        .owner_id = NULL,
    };
    rs485_supervisor_instance = Supervisor_Register(&supervisor_conf);

    HAL_UART_Receive_IT(rs485_uart_handle, &uart2_current_byte, 1);
}

uint8_t RS485_Is_Online(void)
{
    if (rs485_supervisor_instance == NULL)
    {
        return 0;
    }

    return (rs485_online_flag && rs485_supervisor_instance->online_flag);
}

static void uart2_append_byte(uint8_t byte)
{
    if (uart2_buffer_length >= sizeof(uart2_receive_buffer))
    {
        memmove(uart2_receive_buffer, uart2_receive_buffer + 1, sizeof(uart2_receive_buffer) - 1);
        uart2_buffer_length = sizeof(uart2_receive_buffer) - 1;
    }
    uart2_receive_buffer[uart2_buffer_length++] = byte;
}

static uint8_t uart2_extract_frame(Rx_packed_t *frame)
{
    while (uart2_buffer_length >= sizeof(Rx_packed_t))
    {
        if (uart2_receive_buffer[0] != FRAME_HEADER)
        {
            memmove(uart2_receive_buffer, uart2_receive_buffer + 1, uart2_buffer_length - 1);
            uart2_buffer_length--;
            continue;
        }

        if (uart2_receive_buffer[sizeof(Rx_packed_t) - 3] != FRAME_TAILER)
        {
            memmove(uart2_receive_buffer, uart2_receive_buffer + 1, uart2_buffer_length - 1);
            uart2_buffer_length--;
            continue;
        }

        if (Verify_CRC16_Check_Sum(uart2_receive_buffer, sizeof(Rx_packed_t)) == 0)
        {
            memmove(uart2_receive_buffer, uart2_receive_buffer + 1, uart2_buffer_length - 1);
            uart2_buffer_length--;
            continue;
        }
        rs485_cnt++;
        memcpy(frame, uart2_receive_buffer, sizeof(Rx_packed_t));
        memmove(uart2_receive_buffer, uart2_receive_buffer + sizeof(Rx_packed_t), uart2_buffer_length - sizeof(Rx_packed_t));
        uart2_buffer_length -= sizeof(Rx_packed_t);
        return 1;
    }

    return 0;
}

// 记得启用接收中断HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);

void uart2_send_data(Tx_packed_t *data_p) // 发送数据函�?
{
    data_p->frame_header = FRAME_HEADER;
    data_p->frame_tailer = FRAME_TAILER;
    Append_CRC16_Check_Sum((uint8_t *)data_p, sizeof(Tx_packed_t));
    memcpy(uart2_transmit_buffer, data_p, sizeof(Tx_packed_t));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit_IT(&huart2, uart2_transmit_buffer, sizeof(Tx_packed_t));
}

void uart2_transmit_control(void) // 发送控制函数，发送时直接调用这个函数即可
{
    if (uart2_status == ready_to_transmit)
    {
        uart2_send_data(&uart2_tx_message);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 接收回调函数
{
    if (huart == &huart2)
    {
        uart2_append_byte(uart2_current_byte);

        if (uart2_extract_frame(&uart2_rx_message))
        {
            if (rs485_supervisor_instance != NULL)
            {
                Supervisor_Reload(rs485_supervisor_instance);
                rs485_online_flag = 1;
            }
            last_uart2_uwTick = uwTick;
            uart2_status = ready_to_transmit;
            uart2_transmit_control();
            cnttttt++;
        }
        HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
    }
}

void uart2_online_check(void)
{
    if (uwTick - last_uart2_uwTick > 5)
    {
        last_uart2_uwTick = uwTick;
        uart2_status = ready_to_transmit;
        uart2_transmit_control();
        if (uwTick - last_uart2_uwTick > 50)
        {
            //uart2_rx_message.gimbal_angle_yaw_motor2imu = 0;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) // 发送回调函�?
{
    if (huart == &huart2)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        uart2_status = ready_to_receive;
    }
}

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) // 错误回调函数
// {
//     memset(uart2_receive_buffer, 0, sizeof(uart2_receive_buffer));
//     uart2_buffer_length = 0;
//     memset(&uart2_rx_message, 0, sizeof(Rx_packed_t));
//     HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
// }
