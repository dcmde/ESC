#include <stm32f10x.h>
#include <stdio.h>
#include <math.h>

#include "peripheral_config.h"
#include "misc_user.h"
#include "motor_param_estimation.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

int main() {
    // SysTick_Handler every 1 ms
    SysTick_Config(72000);

    RCC_init();

    Gpio_init();

    ADC_init();

    PWM_Bridge_init();

    timeS_1kHz = 100;
    while (timeS_1kHz);

    UART_init();

    Time_init();

    Encoder_init();

//    set_loop(offset_tuning);
//    timeS_1kHz = 1;
//    while (timeS_1kHz);

    set_loop(transfer_function);

    while (1) { __asm__("nop");}
}

#pragma clang diagnostic pop