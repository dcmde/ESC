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

volatile double u = 0;
volatile float theta = 0, offset = 0;
volatile uint16_t theta_cur_pts = 0, theta_prev_pts = 0;
extern uint16_t data_adc[12];
extern char uart_array[UART_ARRAY_LEN];
volatile int16_t speed = 0;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta = TIM4->CNT / 800.0 * 3.1416 * 7;
        offset = ((float) data_adc[3] - 2048) / 2048.0 * 3.1416;
        TIM1->CCR1 = (uint16_t) 500 * (1 + u * sinf(theta + offset));
        TIM1->CCR2 = (uint16_t) 500 * (1 + u * sinf(theta + offset - 2 * 3.1416 / 3.0));
        TIM1->CCR3 = (uint16_t) 500 * (1 + u * sinf(theta + offset + 2 * 3.1416 / 3.0));
    }
}

volatile float f = 1;
volatile uint32_t time_1kHz = 0;
uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};
uint16_t cpt_1kHz = 0;
uint8_t index_freq = 0;

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        GPIOC->BSRR = 0x2000;

        ++time_1kHz;
        if (cpt_1kHz++ == 1000) {
            cpt_1kHz = 0;
            f = f_array[index_freq];
            if (++index_freq == 10) {
                index_freq = 0;
            }
        }

        u = .3 * sin(2 * M_PI * f / 1000.0 * time_1kHz);

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

        // Header
        uart_array[0] = 0x5A;
        uart_array[1] = 0xA5;
        // Time
        uart_array[2] = (time_1kHz >> 8) & 0xFF;
        uart_array[3] = time_1kHz & 0xFF;
        // Freq
        uart_array[4] = f_array[index_freq];
        // theta
        uart_array[5] = (theta_cur_pts >> 8) & 0xFF;
        uart_array[6] = theta_cur_pts & 0xFF;
        // speed
        uart_array[7] = (speed >> 8) & 0xFF;
        uart_array[8] = speed & 0xFF;

        DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
        DMA1_Channel7->CNDTR = UART_ARRAY_LEN;
        DMA1_Channel7->CCR |= DMA_CCR7_EN;

        GPIOC->BRR = 0x2000;
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
