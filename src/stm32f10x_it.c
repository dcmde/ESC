#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#define ACTIVATE_R1 TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE)
#define ACTIVATE_R2 TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE)
#define ACTIVATE_R3 TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define DEACTIVATE_R1 TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE)
#define DEACTIVATE_R2 TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE)
#define DEACTIVATE_R3 TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE)

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
void SysTick_Handler(void) {
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

volatile uint8_t state = 0;
volatile float theta = 0;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        theta += 0.05;
        GPIOC->ODR ^= 0x2000;
        double A = 0.09;
        TIM1->CCR1 = (uint16_t)500*(1 + A*sinf(theta));
        TIM1->CCR2 = (uint16_t)500*(1 + A*sinf(theta + 2*3.1416/3.0));
        TIM1->CCR3 = (uint16_t)500*(1 + A*sinf(theta - 2*3.1416/3.0));
    }
}
/*
void TIM2_IRQHandler(void) {
    int16_t alpha_1, alpha_2, alpha_3;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        GPIOC->ODR ^= 0x2000;

        switch (state) {
            case 0:
                state = 1;

                DEACTIVATE_R2;
                ACTIVATE_R3;

                TIM1->CCR1 = 800;
                TIM1->CCR3 = 200;
                break;
            case 1:
                state = 2;

                DEACTIVATE_R1;
                ACTIVATE_R2;

                TIM1->CCR2 = 800;
                TIM1->CCR3 = 200;

                break;
            case 2:
                state = 3;

                ACTIVATE_R1;
                DEACTIVATE_R3;

                TIM1->CCR1 = 200;
                TIM1->CCR2 = 800;

                break;
            case 3:
                state = 4;

                DEACTIVATE_R2;
                ACTIVATE_R3;

                TIM1->CCR1 = 200;
                TIM1->CCR3 = 800;

                break;
            case 4:
                state = 5;

                ACTIVATE_R2;
                DEACTIVATE_R1;

                TIM1->CCR2 = 200;
                TIM1->CCR3 = 800;

                break;
            case 5:
                state = 0;

                ACTIVATE_R1;
                DEACTIVATE_R3;

                TIM1->CCR1 = 800;
                TIM1->CCR2 = 200;

                break;
        }
    }
}
*/