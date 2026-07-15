#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"

uint16_t remote_last_pos = 0;  /* non-static: stm32h7xx_it.c 恢复 DMA 时需复位 */

void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_TC;
    huart->RxXferSize = DataLength;
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx, (uint32_t)SrcAddress, (uint32_t)DstAddress, (uint32_t)SecondMemAddress, DataLength);
}

void BSP_USART_Init(void)
{
    USART_RxDMA_MultiBufferStart(&huart5, (uint32_t *)&(huart5.Instance->RDR), (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], 36);

    /* --- USART10 遥控器接收: 强制初始化 DMA 环形接收 --- */
    __HAL_UART_CLEAR_OREFLAG(&huart10);
    (void)huart10.Instance->RDR;

    /* 强制重置 RxState，确保 HAL_UART_Receive_DMA 不会被 BUSY 拦截 */
    huart10.RxState = HAL_UART_STATE_READY;

    HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(&huart10, REMOTE_BUF, REMOTE_BUF_SIZE);
    (void)ret;  /* 如果此处不是 HAL_OK，断点观察 ret 值 */

    /* 兜底：如果 HAL 没能使能 DMA（EN=0），手动使能 */
    {
        DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)huart10.hdmarx->Instance;
        if ((dma->CR & DMA_SxCR_EN) == 0U) {
            /* 重载 NDTR，使能 DMA 流 */
            dma->NDTR = REMOTE_BUF_SIZE;
            dma->CR |= DMA_SxCR_EN;
        }
    }

    /* 确保 CR3.DMAR = 1（UART→DMA 请求通路） */
    huart10.Instance->CR3 |= USART_CR3_DMAR;

    __HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);
}

void REMOTE_IdleHandler(void)
{
    uint16_t cur_pos = (REMOTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart10.hdmarx))
                       % REMOTE_BUF_SIZE;
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
    static uint8_t  sync_state = 0;
    static uint8_t  frame_idx = 0;
    static uint8_t  head_byte = 0;

    uint16_t i = start;
    while (i != end) {
        uint8_t byte = buf[i];
        switch (sync_state) {
            case 0:
                if (byte == 0x5B || byte == 0x5C || byte == 0x5D) {
                    head_byte = byte;
                    frame[0] = byte;
                    sync_state = 1;
                }
                break;

            case 1:
                if (byte == head_byte) {
                    frame[1] = byte;
                    frame_idx = 2;
                    sync_state = 2;
                } else if (byte == 0x5B || byte == 0x5C || byte == 0x5D) {
                    head_byte = byte;
                    frame[0] = byte;
                } else {
                    sync_state = 0;
                }
                break;

            case 2:
                frame[frame_idx++] = byte;
                if (frame_idx == 9) {
                    uint8_t expected_footer;
                    switch (head_byte)
                    {
                        case 0x5B: expected_footer = 0x2B; break;
                        case 0x5C: expected_footer = 0x2C; break;
                        case 0x5D: expected_footer = 0x2D; break;
                        default:   expected_footer = 0x00; break;
                    }

                    if (frame[8] == expected_footer) {
                        REMOTE_ParseData(frame, &RCctrl);
                    }
                    sync_state = 0;
                    frame_idx = 0;
                }
                break;
        }
        i = (i + 1) % size;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart5){
         if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
      {
                __HAL_DMA_DISABLE(huart->hdmarx);
                ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
          __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);

          if(Size == 0x000B && SBUS_MultiRx_Buf[0][0] == 0x0F && SBUS_MultiRx_Buf[0][24] == 0x00)
          {
           // SBUS_TO_RC(SBUS_MultiRx_Buf[0], &SBUS_RC);
          }
      }
      else
      {
                __HAL_DMA_DISABLE(huart->hdmarx);
            ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
          __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);

          if(Size == 0x000B && SBUS_MultiRx_Buf[1][0] == 0x0F && SBUS_MultiRx_Buf[1][24] == 0x00)
          {
           // SBUS_TO_RC(SBUS_MultiRx_Buf[1], &SBUS_RC);
          }
      }

      huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
      huart->RxEventType = HAL_UART_RXEVENT_TC;
      __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
      SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
      __HAL_DMA_ENABLE(huart->hdmarx);
  }
}