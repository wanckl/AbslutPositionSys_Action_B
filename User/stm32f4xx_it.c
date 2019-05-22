/**
  ******************************************************************************
  * @file    CAN/LoopBack/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "led.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup CAN_LoopBack
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t ret;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

#ifdef USE_CAN1
/**
  * @brief  This function handles CAN1 RX0 request.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	RxMessage.StdId = 0x00;
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = 0;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	RxMessage.Data[0] = 0x00;
	RxMessage.Data[1] = 0x00;

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	if((RxMessage.ExtId == 0x1234) && (RxMessage.IDE == CAN_ID_EXT)
	 && (RxMessage.DLC == 2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8) == 0xDECA))
	{
		ret = 1; 
	}
	else
	{
		ret = 0; 
	}
}
#endif  /* USE_CAN1 */

#ifdef USE_CAN2
/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  RxMessage.StdId = 0x00;
  RxMessage.ExtId = 0x00;
  RxMessage.IDE = 0;
  RxMessage.DLC = 0;
  RxMessage.FMI = 0;
  RxMessage.Data[0] = 0x00;
  RxMessage.Data[1] = 0x00;

  CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

  if((RxMessage.ExtId == 0x1234) && (RxMessage.IDE == CAN_ID_EXT)
     && (RxMessage.DLC == 2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8) == 0xDECA))
  {
    ret = 1; 
  }
  else
  {
    ret = 0; 
  }
}
#endif  /* USE_CAN2 */

/**TIM2 interrupt Handler for time base source
  * @}
  */ 
uint8_t imu_get_flag = 0, velocity_flag = 0;
void TIM2_IRQHandler(void)
{
	static uint8_t timer_10ms;
	static uint16_t timer_res;
	uint16_t res = 0;
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //400 us溢出中断
	{
		timer_res ++;
		timer_10ms ++;
		if(timer_10ms >= 25)
		{
			timer_10ms = 0;
			imu_get_flag = 1;
		}
		if(timer_res >= 1250)	//0.5s timer_res base
		{
			timer_res = 0;
			LED0 = ~LED0;
//			velocity_flag = 1;
			res = mpu_temp_pid(60);
			res <<= 1;
			TIM_SetCompare3(TIM3,0);	//Hate Res at Bottom
			TIM_SetCompare4(TIM3,0);	//Top Layer
		}
		ssi_data_process();
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}
/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
