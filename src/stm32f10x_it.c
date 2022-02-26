#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "global_variables.h"

float theta = 0, f = 1;

uint32_t time3_1kHz = 0;
uint16_t theta_cur_pts = 0, theta_prev_pts = 0;
uint16_t cpt3_1kHz = 0;
int16_t speed = 0;
uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};
uint8_t index_freq = 0;

void SysTick_Handler(void) {
    if (timeS_1kHz > 0) {
        --timeS_1kHz;
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta = (TIM4->CNT + theta_offset) / 800.0 * 3.1416 * 7;
        TIM1->CCR1 = (uint16_t) 500 * (1 + u * sinf(theta));
        TIM1->CCR2 = (uint16_t) 500 * (1 + u * sinf(theta - 2 * 3.1416 / 3.0));
        TIM1->CCR3 = (uint16_t) 500 * (1 + u * sinf(theta + 2 * 3.1416 / 3.0));
    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        GPIOC->BSRR = 0x2000;

        ++time3_1kHz;
        if (cpt3_1kHz++ == 1000) {
            cpt3_1kHz = 0;
            f = f_array[index_freq];
            if (++index_freq == 10) {
                index_freq = 0;
            }
        }

        u = .3 * sin(2 * M_PI * f / 1000.0 * time3_1kHz);

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
        uart_array[2] = (time3_1kHz >> 8) & 0xFF;
        uart_array[3] = time3_1kHz & 0xFF;
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
    }
}

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
}

void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
}

void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
}

void SVC_Handler(void) {
}

void DebugMon_Handler(void) {
}

void PendSV_Handler(void) {
}