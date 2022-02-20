#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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
extern char uart_array[UART_ARRAY_LEN];
volatile int32_t speed = 0;
volatile uint32_t encoder_num_turn = 0;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta = TIM4->CNT / 800.0 * 3.1416 * 7;
        offset = ((float) data_adc[3] - 2048) / 2048.0 * 3.1416;
//        uart_array[0] = speed >> 24 & 0xFF;
//        uart_array[1] = speed >> 16 & 0xFF;
//        uart_array[2] = speed >> 8 & 0xFF;
//        uart_array[3] = speed & 0xFF;
        //double A = ((float) data_adc[3] - 2048) / 2048.0;
        TIM1->CCR1 = (uint16_t) 500 * (1 + A * sinf(theta + offset));
        TIM1->CCR2 = (uint16_t) 500 * (1 + A * sinf(theta + offset - 2 * 3.1416 / 3.0));
        TIM1->CCR3 = (uint16_t) 500 * (1 + A * sinf(theta + offset + 2 * 3.1416 / 3.0));
    }
}

volatile uint16_t time_counter = 1;
volatile int32_t speed_for, speed_rev;

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        if (!(--time_counter)) {
            // Toggle PC13
            GPIOC->ODR ^= 0x2000;
            if (A > 0) {
                A = -.2;
            } else {
                A = .2;
            }

            // Reset max speeds
            speed_for = 0;
            speed_rev = 0;
            // Set counter to trigger à 1Hz
            time_counter = 1000;
        }

        theta_prev_pts = theta_cur_pts;
        theta_cur_pts = TIM4->CNT;

        speed = theta_cur_pts - theta_prev_pts;

        if (TIM4->CR1 & TIM_CR1_DIR) {
            if (theta_prev_pts < theta_cur_pts) {
                speed = -theta_cur_pts + theta_prev_pts + 1599;
            } else {
                speed = theta_cur_pts - theta_prev_pts;
            }
        } else {
            if (theta_prev_pts > theta_cur_pts) {
                speed = theta_cur_pts - theta_prev_pts + 1599;
            } else {
                speed = theta_cur_pts - theta_prev_pts;
            }
        }


        // Transmit data through UART
        uint8_t n = sprintf(uart_array, "%i %i\n", (int) speed, (int) theta_cur_pts);
        encoder_num_turn = 0;
        DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
        DMA1_Channel7->CNDTR = n;
        DMA1_Channel7->CCR |= DMA_CCR7_EN;
    }
}

void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        // TIM4 IT_Update Disabled
//        ++encoder_num_turn;
//        if (TIM4->CR1 & TIM_CR1_DIR) {
//            encoder_num_turn = 1;
//        } else {
//            encoder_num_turn = 2;
//        }
    }
}
