#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "global.h"

float theta = 0.f, u = 0.f;
uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};
uint8_t i = 0;

volatile motor_control_struct_t motorControlStruct;

void SysTick_Handler(void) {

    if (timeS_1kHz > 0) {
        --timeS_1kHz;
    }
    GPIOC->BSRR = 0x2000; // PC13 ON

    motorControlStruct.theta = TIM4->CNT;
    //motorControlStruct.time_xxHz = ;
    motorControlStruct.speed = get_speed(motorControlStruct.theta);
    motorControlStruct.speed_filt = (speed_cur * 6 + motorControlStruct.speed_filt * 2) / 8;

    loop_run(&motorControlStruct);

    GPIOC->BRR = 0x2000; // PC13 OFF
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        int16_t TIM_ON_1, TIM_ON_2, TIM_ON_3;

        theta = (float) (TIM4->CNT + motorControlStruct.theta_offset) / 800.f * 3.1416f * 7.f;

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