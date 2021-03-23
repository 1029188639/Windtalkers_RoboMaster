#include "vision.h"
#include <stdio.h>
#include <string.h>

#include "bsp_vision.h"
#include "dma.h"
#include "usart.h"
#include "gimbal_task.h"
//#include "caninfo_task.h"
#include "remote_control.h"
#include "CRC8_CRC16.h"
#include "remote_control.h"

#define VS_huart	huart1
#define VS_UART		USART1
#define VS_rx_dma	hdma_usart1_rx
#define VS_tx_dma	hdma_usart1_tx

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

#define VS_FRAME_LENGTH 20
#define VS_BUF_NUM      64

portTickType visionTime;
/*
Ϊ�Ӿ����ݿ��ٿռ�
*/
visionStruct visionData={0};
visionStruct visionTempData={0};
RootAngleType rootAngle={0};
followStruct followData={0};
uint8_t visionReceiveFlag = 0;
uint8_t press_r = 0;

uint8_t vision_rx_buf[2][VS_FRAME_LENGTH];
uint8_t errcnt;

//����Ԥ��ģʽ
uint8_t Normal_anticipation[3] = {0x00,0xBB,0x01};
//С����Ԥ��ģʽ
uint8_t gyro_anticipation[3] = {0x00,0xBB,0x10};

const RC_ctrl_t *visionRC;

void Vision_Task(void const *pvParameters)
{
	vision_usart_init();
	visionRC = get_remote_control_point();
	while(1)
	{
		//�л�����Ԥ��ģʽ
		if(visionRC->key.v & KEY_PRESSED_OFFSET_C)
		{
			HAL_UART_Transmit(&huart1,Normal_anticipation,3,1);
		}
		//�л�����Ԥ��ģʽ
		if(visionRC->key.v & KEY_PRESSED_OFFSET_V)
		{
			HAL_UART_Transmit(&huart1,gyro_anticipation,3,1);
		}
		
		//����Ҽ������Ӿ�
		if(visionRC->mouse.press_r)
		{
			press_r = 1;
		}
		else
		{
			press_r = 0;
		}
		osDelay(1);
	}
}

void vision_usart_init(void)
{
	vision_init(vision_rx_buf[0],vision_rx_buf[1],VS_BUF_NUM );
}

void vision_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
		SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //ʹ�ܿ����ж�
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
		
}

static short int flag =0xaa00;
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

       
		__HAL_DMA_DISABLE(&hdma_usart1_rx);

		this_time_rx_len = VS_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

		hdma_usart1_rx.Instance->NDTR = VS_BUF_NUM;

		DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

		__HAL_DMA_ENABLE(&hdma_usart1_rx);
		memcpy(&visionTempData,vision_rx_buf[0],VS_FRAME_LENGTH);
			if(visionTempData.head == flag && visionTempData.PitchData < 12.0f && visionTempData.PitchData > -12.0f && visionTempData.YawData >-12.0f && visionTempData.YawData <12.0f )
			{
					memcpy(&visionData,&visionTempData,VS_FRAME_LENGTH);
					visionReceiveFlag = 1;
			}
    }
}



