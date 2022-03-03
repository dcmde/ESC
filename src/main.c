#include <stm32f10x.h>
#include <stdio.h>
#include <math.h>

#include "peripheral_config.h"
#include "global.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
/*
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
}*/

uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};

void transfer_function(volatile motor_control_struct_t *motorControlStruct) {
    static uint32_t time_tf_1kHz = 0, cpt_tf_1kHz = 0;
    static int16_t speed_fil = 0;
    static uint8_t i = 0;
    static float f;

    uint16_t theta_cur_pts = motorControlStruct->theta;

    ++time_tf_1kHz;

    motorControlStruct->u = 1.f * sinf(2 * M_PI * f / 1000. * time_tf_1kHz);

    if (++cpt_tf_1kHz == 1000) {
        if (++i == 10) {
            i = 0;
        }
        cpt_tf_1kHz = 0;
        f = f_array[i];
    }

    // Header
    uart_array[0] = 0x5A;
    uart_array[1] = 0xA5;
    // Time
    uart_array[2] = (time_tf_1kHz >> 8) & 0xFF;
    uart_array[3] = time_tf_1kHz & 0xFF;
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

    set_loop(transfer_function);

    Time_init();

    Encoder_init();

    while (1) {
        // Send new frequency every second
        timeS_1kHz = 1000;
        while (timeS_1kHz);
    }
}

#pragma clang diagnostic pop