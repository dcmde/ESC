#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

int main() {
    GPIO_InitTypeDef gpioInitTypeDef;
    TIM_TimeBaseInitTypeDef timInitStruct;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB,
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

    timInitStruct.TIM_RepetitionCounter = 0;
    timInitStruct.TIM_Prescaler = 720;
    timInitStruct.TIM_Period = 100;
    timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1, &timInitStruct);

    TIM_OCInitTypeDef ocInitTypeDef;
    ocInitTypeDef.TIM_OCIdleState = TIM_OCIdleState_Set;
    ocInitTypeDef.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    ocInitTypeDef.TIM_OCMode = TIM_OCMode_PWM2;
    ocInitTypeDef.TIM_OCPolarity = TIM_OCPolarity_High;
    ocInitTypeDef.TIM_OCNPolarity = TIM_OCNPolarity_High;
    ocInitTypeDef.TIM_OutputState = TIM_OutputState_Enable;
    ocInitTypeDef.TIM_OutputNState = TIM_OutputNState_Enable;
    ocInitTypeDef.TIM_Pulse = 50;

    TIM_OC1Init(TIM1, &ocInitTypeDef);
    TIM_OC2Init(TIM1, &ocInitTypeDef);
    TIM_OC3Init(TIM1, &ocInitTypeDef);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 0xFF;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_Cmd(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    while (1) {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}