#include <stm32f10x.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_dma.h>
#include "misc.h"

#define uart_data_size 5
extern volatile int32_t encoder_num_turn;
volatile uint8_t data_uart[uart_data_size] = {'1', '2', '3', '4', '\n'};
uint16_t data_adc[12] = {0};
extern volatile uint32_t time;
extern volatile double A;

void Gpio_init();

void ADC_init();

void UART_init();

void PWM_Bridge_init();

void Encoder_init();

void Time_init();

int main() {
    // SysTick_Handler every 1 ms
    SysTick_Config(72000);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_ADC1 |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC,
                           ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |
                           RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4 |
                           RCC_APB1Periph_USART2,
                           ENABLE);

    Gpio_init();

    ADC_init();

    PWM_Bridge_init();

    time = 50;
    while (time);

    UART_init();

    Time_init();

    Encoder_init();

    while (1) {
        // Activate DMA to transfer DMA values
        DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
        DMA1_Channel7->CNDTR = uart_data_size;
        DMA1_Channel7->CCR |= DMA_CCR7_EN;

    }

}

void Gpio_init() {
    GPIO_InitTypeDef gpioInitTypeDef;

    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_13;
    gpioInitTypeDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOA, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpioInitTypeDef);

    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpioInitTypeDef);
}

void ADC_init() {
    ADC_InitTypeDef adcInitTypeDef;
    DMA_InitTypeDef dmaInitTypeDef;

    dmaInitTypeDef.DMA_BufferSize = 12;
    dmaInitTypeDef.DMA_DIR = DMA_DIR_PeripheralSRC;
    dmaInitTypeDef.DMA_M2M = DMA_M2M_Disable;
    dmaInitTypeDef.DMA_MemoryBaseAddr = (uint32_t) data_adc;
    dmaInitTypeDef.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1->DR);
    dmaInitTypeDef.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dmaInitTypeDef.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitTypeDef.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitTypeDef.DMA_Mode = DMA_Mode_Circular;
    dmaInitTypeDef.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dmaInitTypeDef.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel1, &dmaInitTypeDef);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    adcInitTypeDef.ADC_ContinuousConvMode = ENABLE;
    adcInitTypeDef.ADC_ScanConvMode = ENABLE;
    adcInitTypeDef.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adcInitTypeDef.ADC_DataAlign = ADC_DataAlign_Right;
    adcInitTypeDef.ADC_Mode = ADC_Mode_Independent;
    adcInitTypeDef.ADC_NbrOfChannel = 4;

    ADC_Init(ADC1, &adcInitTypeDef);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_28Cycles5);

    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);

    while (ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void UART_init() {
    USART_InitTypeDef usartInitTypeDef;
    DMA_InitTypeDef dmaInitTypeDef;
    GPIO_InitTypeDef gpioInitTypeDef;

    gpioInitTypeDef.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInitTypeDef.GPIO_Pin = GPIO_Pin_2;
    gpioInitTypeDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioInitTypeDef);

    dmaInitTypeDef.DMA_BufferSize = uart_data_size;
    dmaInitTypeDef.DMA_DIR = DMA_DIR_PeripheralDST;
    dmaInitTypeDef.DMA_M2M = DMA_M2M_Disable;
    dmaInitTypeDef.DMA_MemoryBaseAddr = (uint32_t) data_uart;
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
}

void PWM_Bridge_init() {
    TIM_OCInitTypeDef ocInitTypeDef;
    TIM_TimeBaseInitTypeDef timInitStruct;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    timInitStruct.TIM_RepetitionCounter = 0;
    timInitStruct.TIM_Prescaler = 0;
    timInitStruct.TIM_Period = 1000;
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1, &timInitStruct);

    ocInitTypeDef.TIM_OCIdleState = TIM_OCIdleState_Set;
    ocInitTypeDef.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    ocInitTypeDef.TIM_OCMode = TIM_OCMode_PWM1;
    ocInitTypeDef.TIM_OCPolarity = TIM_OCPolarity_High;
    ocInitTypeDef.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    ocInitTypeDef.TIM_OutputState = TIM_OutputState_Enable;
    ocInitTypeDef.TIM_OutputNState = TIM_OutputNState_Enable;
    ocInitTypeDef.TIM_Pulse = 900;

    TIM_OC1Init(TIM1, &ocInitTypeDef);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    ocInitTypeDef.TIM_Pulse = 100;
    TIM_OC2Init(TIM1, &ocInitTypeDef);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    ocInitTypeDef.TIM_Pulse = 100;
    TIM_OC3Init(TIM1, &ocInitTypeDef);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 0x0A;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void Encoder_init() {
    NVIC_InitTypeDef nvicInitTypeDef;

    nvicInitTypeDef.NVIC_IRQChannelSubPriority = 0;
    nvicInitTypeDef.NVIC_IRQChannel = TIM4_IRQn;
    nvicInitTypeDef.NVIC_IRQChannelPreemptionPriority = 2;
    nvicInitTypeDef.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInitTypeDef);

    TIM4->ARR = 1599;
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
    //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
}

void Time_init() {
    NVIC_InitTypeDef nvicInitTypeDef;
    TIM_TimeBaseInitTypeDef timInitStruct;

    nvicInitTypeDef.NVIC_IRQChannelSubPriority = 6;
    nvicInitTypeDef.NVIC_IRQChannel = TIM2_IRQn;
    nvicInitTypeDef.NVIC_IRQChannelPreemptionPriority = 3;
    nvicInitTypeDef.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInitTypeDef);

    nvicInitTypeDef.NVIC_IRQChannelSubPriority = 7;
    nvicInitTypeDef.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_Init(&nvicInitTypeDef);

    timInitStruct.TIM_Prescaler = 71;
    timInitStruct.TIM_Period = 200;
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    // TIM2 freq 5kHz
    TIM_TimeBaseInit(TIM2, &timInitStruct);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // TIM3 freq 1kHz
    timInitStruct.TIM_Period = 1000;
    timInitStruct.TIM_Prescaler = 71;
    TIM_DeInit(TIM3);
    TIM_TimeBaseInit(TIM3, &timInitStruct);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}