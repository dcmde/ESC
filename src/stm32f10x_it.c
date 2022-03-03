#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "global.h"

float theta = 0;

uint32_t time3_1kHz = 0, cpt3_1kHz = 0;
uint16_t theta_cur_pts = 0;
uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};
uint8_t i = 0;

void SysTick_Handler(void) {
    static int16_t speed_fil = 0;

    if (timeS_1kHz > 0) {
        --timeS_1kHz;
    }
    GPIOC->BSRR = 0x2000; // PC13 ON

    ++time3_1kHz;

    theta_cur_pts = TIM4->CNT;

    speed_cur = get_speed(theta_cur_pts);

    speed_fil = (speed_cur * 6 + speed_fil * 2) / 8;

    u = 1.f * sinf(2 * M_PI * f / 1000. * time3_1kHz);

    if (++cpt3_1kHz == 1000) {
        if (++i == 10) {
            i = 0;
        }
        cpt3_1kHz = 0;
        f = f_array[i];
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
    // freq sin
    uart_array[10] = f_array[i] & 0xFF;

    DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
    DMA1_Channel7->CNDTR = UART_ARRAY_LEN;
    DMA1_Channel7->CCR |= DMA_CCR7_EN;

    GPIOC->BRR = 0x2000; // PC13 OFF
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        int16_t TIM_ON_1, TIM_ON_2, TIM_ON_3;

        theta = (float) (TIM4->CNT + theta_offset) / 800.f * 3.1416f * 7.f;

        // alpha = Ton/Tperiod
        // alpha = 1/2 + 1/2.(u/Vcc) => u_phase = u/2
        // here we have
        // alpha = 1/2 + (u/Vcc) => u_phase = u
        TIM_ON_1 = (int16_t) TIM_PERIOD * (.5f + u * sinf(theta) / 10.f);
        TIM_ON_2 = (int16_t) TIM_PERIOD * (.5f + u * sinf(theta - 2 * 3.1416f / 3.f) / 10.f);
        TIM_ON_3 = (int16_t) TIM_PERIOD * (.5f + u * sinf(theta + 2 * 3.1416f / 3.f) / 10.f);

        TIM_ON_1 = TIM_ON_1 < 0 ? 0 : TIM_ON_1;
        TIM_ON_2 = TIM_ON_2 < 0 ? 0 : TIM_ON_2;
        TIM_ON_3 = TIM_ON_3 < 0 ? 0 : TIM_ON_3;

        TIM_ON_1 = TIM_ON_1 > TIM_PERIOD_CLAMP ? TIM_PERIOD_CLAMP : TIM_ON_1;
        TIM_ON_2 = TIM_ON_2 > TIM_PERIOD_CLAMP ? TIM_PERIOD_CLAMP : TIM_ON_2;
        TIM_ON_3 = TIM_ON_3 > TIM_PERIOD_CLAMP ? TIM_PERIOD_CLAMP : TIM_ON_3;

        TIM1->CCR1 = TIM_ON_1;
        TIM1->CCR2 = TIM_ON_2;
        TIM1->CCR3 = TIM_ON_3;
    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
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