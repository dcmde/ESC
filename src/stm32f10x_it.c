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
volatile uint32_t time = 0;

void SysTick_Handler(void) {
    if (time > 0) {
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
volatile float theta = 0, offset = 0;
volatile uint32_t theta_cur_pts = 0, theta_prev_pts = 0;
extern uint16_t data_adc[12];
extern volatile uint8_t data_uart[5];
volatile uint32_t speed = 0;
volatile uint32_t encoder_num_turn = 0;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta = TIM4->CNT / 800.0 * 3.1416 * 7;
        offset = ((float) data_adc[3] - 2048) / 2048.0 * 3.1416;
        theta_prev_pts = theta_cur_pts;
        theta_cur_pts = TIM4->CNT;
        if (TIM4->CR1 & TIM_CR1_DIR) {
            speed = 1599 - theta_cur_pts + theta_prev_pts + encoder_num_turn * 1599;
        } else {
            speed = theta_cur_pts - theta_prev_pts + encoder_num_turn * 1599;
        }
        encoder_num_turn = 0;
        data_uart[0] = speed >> 24 & 0xFF;
        data_uart[1] = speed >> 16 & 0xFF;
        data_uart[2] = speed >> 8 & 0xFF;
        data_uart[3] = speed & 0xFF;
        //double A = ((float) data_adc[3] - 2048) / 2048.0;
        TIM1->CCR1 = (uint16_t) 500 * (1 + A * sinf(theta + offset));
        TIM1->CCR2 = (uint16_t) 500 * (1 + A * sinf(theta + offset - 2 * 3.1416 / 3.0));
        TIM1->CCR3 = (uint16_t) 500 * (1 + A * sinf(theta + offset + 2 * 3.1416 / 3.0));
    }
}

volatile uint16_t time_counter = 1;
void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (!(--time_counter)) {
            // Toggle PC13
            GPIOC->ODR ^= 0x2000;
            // Increment command value by 0.05 between -.3 and .3
            A += 0.05;
            if (A > 0.3) {
                A = -.3;
            }
            time_counter = 1000;
        }
    }
}

void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        ++encoder_num_turn;
//        if (TIM4->CR1 & TIM_CR1_DIR) {
//            ++encoder_num_turn;
//        } else {
//            --encoder_num_turn;
//        }
    }
}
