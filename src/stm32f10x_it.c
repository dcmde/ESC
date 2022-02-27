#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "global.h"

float theta = 0, f = 1;

uint32_t time3_1kHz = 0, cpt3_1kHz = 0;
uint16_t theta_cur_pts = 0;

void SysTick_Handler(void) {
    if (timeS_1kHz > 0) {
        --timeS_1kHz;
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta = (float) (TIM4->CNT + theta_offset) / 800.f * 3.1416f * 7.f;
        TIM1->CCR1 = (uint16_t) PWM_MAX_AMPLITUDE * (1 + u * sinf(theta));
        TIM1->CCR2 = (uint16_t) PWM_MAX_AMPLITUDE * (1 + u * sinf(theta - 2 * 3.1416f / 3.f));
        TIM1->CCR3 = (uint16_t) PWM_MAX_AMPLITUDE * (1 + u * sinf(theta + 2 * 3.1416f / 3.f));
    }
}

void TIM3_IRQHandler(void) {
    static int16_t speed_fil = 0;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        GPIOC->BSRR = 0x2000; // PC13 ON

        ++time3_1kHz;

        theta_cur_pts = TIM4->CNT;

        speed_cur = get_speed(theta_cur_pts, TIM4->CR1 & TIM_CR1_DIR);

        speed_fil = (speed_cur * 6 + speed_fil * 2) / 8;

        // very 500ms
        if (++cpt3_1kHz == 500) {
            cpt3_1kHz = 0;
            theta_offset = loop_tim3(speed_fil);
        }

        // Header
        uart_array[0] = 0x5A;
        uart_array[1] = 0xA5;
        // Time
        uart_array[2] = (time3_1kHz >> 8) & 0xFF;
        uart_array[3] = time3_1kHz & 0xFF;
        // theta
        uart_array[4] = (theta_cur_pts >> 8) & 0xFF;
        uart_array[5] = theta_cur_pts & 0xFF;
        // speed_cur
        uart_array[6] = (speed_cur >> 8) & 0xFF;
        uart_array[7] = speed_cur & 0xFF;
        // speed_fil
        uart_array[8] = (speed_fil >> 8) & 0xFF;
        uart_array[9] = speed_fil & 0xFF;
        // offset theta
        uart_array[10] = (theta_offset >> 8) & 0xFF;
        uart_array[11] = theta_offset & 0xFF;

        DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
        DMA1_Channel7->CNDTR = UART_ARRAY_LEN;
        DMA1_Channel7->CCR |= DMA_CCR7_EN;

        GPIOC->BRR = 0x2000; // PC13 OFF
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