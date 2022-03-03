#include <stm32f10x.h>
#include <stdio.h>

#include "peripheral_config.h"
#include "global.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

int16_t foo(int16_t speed_fil) {
    static int16_t speed_max_p = 0, speed_max_n = 0;
    // Change direction
    if (u > 0) {
        speed_max_p = speed_fil;
        u = -1.f;
    } else {
        speed_max_n = speed_fil;
        u = 1.f;
    }

    int16_t var1 = speed_max_p > 0 ? speed_max_p : -speed_max_p;
    int16_t var2 = speed_max_n > 0 ? speed_max_n : -speed_max_n;

    if (var2 - var1 > 2) {
        theta_offset -= 1;
    } else if (var2 - var1 < 2) {
        theta_offset += 1;
    }
    return theta_offset;
}

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

    set_loop_tim3(foo);

    Time_init();

    Encoder_init();

    uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};
    uint8_t i = 0;

    while (1) {
        // Send new frequency every second
        timeS_1kHz = 1000;
        while (timeS_1kHz);
        f = f_array[i++];
        if (i == 10) {
            i = 0;
        }

    }
}

#pragma clang diagnostic pop