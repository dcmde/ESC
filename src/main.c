#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_dma.h>
#include "misc.h"

extern volatile int32_t encoder_num_turn;

int main() {
    uint8_t data[3] = {'a', 'n', '\n'};
    GPIO_InitTypeDef gpioInitTypeDef;
    TIM_TimeBaseInitTypeDef timInitStruct;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    NVIC_InitTypeDef nvicInitTypeDef;
    USART_InitTypeDef usartInitTypeDef;
    DMA_InitTypeDef dmaInitTypeDef;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB,
                           ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |
                           RCC_APB1Periph_TIM4 |
                           RCC_APB1Periph_USART2,
                           ENABLE);

    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_13;
    gpioInitTypeDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpioInitTypeDef);

    timInitStruct.TIM_RepetitionCounter = 0;
    timInitStruct.TIM_Prescaler = 36;
    timInitStruct.TIM_Period = 1000;
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1, &timInitStruct);

    nvicInitTypeDef.NVIC_IRQChannelSubPriority = 6;
    nvicInitTypeDef.NVIC_IRQChannel = TIM2_IRQn;
    nvicInitTypeDef.NVIC_IRQChannelPreemptionPriority = 3;
    nvicInitTypeDef.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInitTypeDef);

    timInitStruct.TIM_Prescaler = 100;
    timInitStruct.TIM_Period = 1000;
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &timInitStruct);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM2, ENABLE);

    nvicInitTypeDef.NVIC_IRQChannelSubPriority = 0;
    nvicInitTypeDef.NVIC_IRQChannel = TIM4_IRQn;
    nvicInitTypeDef.NVIC_IRQChannelPreemptionPriority = 2;
    nvicInitTypeDef.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInitTypeDef);

    TIM4->ARR = 1600;
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    TIM_OCInitTypeDef ocInitTypeDef;
    ocInitTypeDef.TIM_OCIdleState = TIM_OCIdleState_Set;
    ocInitTypeDef.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    ocInitTypeDef.TIM_OCMode = TIM_OCMode_PWM1;
    ocInitTypeDef.TIM_OCPolarity = TIM_OCPolarity_High;
    ocInitTypeDef.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    ocInitTypeDef.TIM_OutputState = TIM_OutputState_Enable;
    ocInitTypeDef.TIM_OutputNState = TIM_OutputNState_Enable;
    ocInitTypeDef.TIM_Pulse = 500;

    TIM_OC1Init(TIM1, &ocInitTypeDef);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    ocInitTypeDef.TIM_Pulse = 500;
    TIM_OC2Init(TIM1, &ocInitTypeDef);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    ocInitTypeDef.TIM_Pulse = 500;
    TIM_OC3Init(TIM1, &ocInitTypeDef);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 0xFF;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    //TIM_Cmd(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // Init UART
    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_2;
    gpioInitTypeDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioInitTypeDef);

    dmaInitTypeDef.DMA_BufferSize = 3;
    dmaInitTypeDef.DMA_DIR = DMA_DIR_PeripheralDST;
    dmaInitTypeDef.DMA_M2M = DMA_M2M_Disable;
    dmaInitTypeDef.DMA_MemoryBaseAddr = (uint32_t) data;
    dmaInitTypeDef.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitTypeDef.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitTypeDef.DMA_Mode = DMA_Mode_Normal;
    dmaInitTypeDef.DMA_PeripheralBaseAddr = (uint32_t) 0x40004404;
    dmaInitTypeDef.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitTypeDef.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitTypeDef.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel7, &dmaInitTypeDef);
    DMA_Cmd(DMA1_Channel7, ENABLE);

    usartInitTypeDef.USART_WordLength = USART_WordLength_8b;
    usartInitTypeDef.USART_StopBits = USART_StopBits_1;
    usartInitTypeDef.USART_Parity = USART_Parity_No;
    usartInitTypeDef.USART_Mode = USART_Mode_Tx;
    usartInitTypeDef.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInitTypeDef.USART_BaudRate = 115200;
    USART_Init(USART2, &usartInitTypeDef);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART2, ENABLE);

    while (1) {
        DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
        DMA1_Channel7->CNDTR = 3;
        DMA1_Channel7->CCR |= DMA_CCR7_EN;
    }
}