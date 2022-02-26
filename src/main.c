#include <stm32f10x.h>
#include <stdio.h>

#include "peripheral_config.h"
#include "global_variables.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

int main() {
    // SysTick_Handler every 1 ms
    SysTick_Config(72000);

    RCC_init();

    Gpio_init();

    ADC_init();

    PWM_Bridge_init();

    timeS_1kHz = 60;
    while (timeS_1kHz);

    UART_init();

    Time_init();

    Encoder_init();

    while (1) {
        // Send new frequency every second
        timeS_1kHz = 1000;
        while (timeS_1kHz);
        //u = 0.3;
        //theta_offset += ;
    }
}

#pragma clang diagnostic pop