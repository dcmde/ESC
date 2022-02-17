#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void) {
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void) {
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern volatile uint32_t time;
volatile uint32_t time = 0;
void SysTick_Handler(void) {
    if(time>0){
        --time;
    }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

volatile uint8_t state = 0;
extern volatile double A;
volatile double A = 0;
volatile float theta = 0, theta_2 = 0, offset = 0;
extern uint16_t data_adc[12];

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta = TIM4->CNT / 800.0 * 3.1416 * 7;
        offset = ((float) data_adc[3] - 2048) / 2048.0 * 3.1416;
        //double A = ((float) data_adc[3] - 2048) / 2048.0;
        TIM1->CCR1 = (uint16_t) 500 * (1 + A * sinf(theta + offset));
        TIM1->CCR2 = (uint16_t) 500 * (1 + A * sinf(theta + offset - 2 * 3.1416 / 3.0));
        TIM1->CCR3 = (uint16_t) 500 * (1 + A * sinf(theta + offset + 2 * 3.1416 / 3.0));
    }
}

volatile int32_t encoder_num_turn = 0;

void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        if (TIM4->CR1 & TIM_CR1_DIR) {
            ++encoder_num_turn;
        } else {
            --encoder_num_turn;
        }
    }
}
