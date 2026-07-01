#include "bsp_uart.h"
#include "usart.h"

static uint16_t remote_last_pos = 0;   // ?????IDLE???DMAд???

void BSP_USART_Init(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    (void)huart5.Instance->RDR;

    HAL_UART_Receive_DMA(&huart5, REMOTE_BUF, REMOTE_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}

/**
 * @brief  ??USART5?ж?????????????IDLE????
 */
void REMOTE_IdleHandler(void)
{
    uint16_t cur_pos = REMOTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
    if (cur_pos != remote_last_pos) {
        REMOTE_ExtractFrame(REMOTE_BUF, REMOTE_BUF_SIZE,
                            remote_last_pos, cur_pos);
        remote_last_pos = cur_pos;
    }
}

static void REMOTE_ExtractFrame(const uint8_t *buf, uint16_t size,
                                uint16_t start, uint16_t end)
{
    static uint8_t  frame[9];
    static uint8_t  sync_state = 0;   // 0:空闲搜帧头, 1:等待第二同步字节, 2:接收帧数据
    static uint8_t  frame_idx = 0;
    static uint8_t  head_byte = 0;    // 保存当前识别到的帧头 0x5B/0x5C/0x5D

    uint16_t i = start;
    while (i != end) {
        uint8_t byte = buf[i];
        switch (sync_state) {
            case 0:  // 空闲状态：搜寻帧头 0x5B / 0x5C / 0x5D
                if (byte == 0x5B || byte == 0x5C || byte == 0x5D) {
                    head_byte = byte;
                    frame[0] = byte;
                    sync_state = 1;
                }
                break;

            case 1:  // 已收到第一个帧头，等待第二个相同同步字节（双字节帧头）
                if (byte == head_byte) {
                    // 匹配双帧头：5B5B / 5C5C / 5D5D，进入数据接收
                    frame[1] = byte;
                    frame_idx = 2;
                    sync_state = 2;
                } else if (byte == 0x5B || byte == 0x5C || byte == 0x5D) {
                    // 收到新帧头，刷新缓存，停留在状态1继续等第二个同步字节
                    head_byte = byte;
                    frame[0] = byte;
                } else {
                    // 字节不匹配，重置同步状态重新搜帧头
                    sync_state = 0;
                }
                break;

            case 2:  // 接收剩余7字节，整帧共9字节
                frame[frame_idx++] = byte;
                if (frame_idx == 9) {
                    // 根据帧头匹配对应帧尾
                    uint8_t expected_footer;
                    switch (head_byte)
                    {
                        case 0x5B: expected_footer = 0x2B; break;
                        case 0x5C: expected_footer = 0x2C; break;
                        case 0x5D: expected_footer = 0x2D; break;
                        default:   expected_footer = 0x00; break;
                    }

                    if (frame[8] == expected_footer) {
                        // 帧尾校验通过，解析数据
                        REMOTE_ParseData(frame, &RCctrl);
                    }
                    // 一帧接收完成，重置状态机
                    sync_state = 0;
                    frame_idx = 0;
                }
                break;
        }
        i = (i + 1) % size;
    }
}






