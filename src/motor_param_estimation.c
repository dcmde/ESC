#include <math.h>
#include <stdint.h>

#include "stm32f10x_dma.h"
#include "motor_param_estimation.h"

void offset_tuning(volatile motor_control_struct_t *motorControlStruct) {
    static int16_t speed_max_p = 0, speed_max_n = 0, theta_offset;
    static uint32_t cpt_1kHz = 0;

    if (++cpt_1kHz == 1000) {
        cpt_1kHz = 0;

        float u = motorControlStruct->u;
        int16_t speed_fil = motorControlStruct->speed_filt;

        // Change direction
        if (u > 0) {
            speed_max_p = speed_fil;
            u = -1.f;
        } else {
            speed_max_n = speed_fil;
            u = 1.f;
        }

        motorControlStruct->u = u;

        int16_t var1 = speed_max_p > 0 ? speed_max_p : -speed_max_p;
        int16_t var2 = speed_max_n > 0 ? speed_max_n : -speed_max_n;

        if (var2 - var1 > 2) {
            theta_offset -= 1;
        } else if (var2 - var1 < 2) {
            theta_offset += 1;
        }
        motorControlStruct->theta_offset = theta_offset;
    }
}

uint8_t f_array[10] = {1, 5, 10, 20, 30, 40, 50, 80, 100, 150};
uint8_t t_array[10] = {10000, 5000, 2000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

void transfer_function(volatile motor_control_struct_t *motorControlStruct) {
    static uint32_t time_tf_1kHz = 0, cpt_1kHz = 0;
    static int16_t speed_fil = 0;
    static uint8_t i = 0;
    static float f = 1.f;

    uint16_t theta_cur_pts = motorControlStruct->theta;

    ++time_tf_1kHz;

    motorControlStruct->u = 1.f * sinf(2 * M_PI * f / 1000. * time_tf_1kHz);

    if (++cpt_1kHz == t_array[i]) {
        if (++i == 10) {
            i = 0;
        }
        cpt_1kHz = 0;
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
    uart_array[6] = (motorControlStruct->speed >> 8) & 0xFF;
    uart_array[7] = motorControlStruct->speed & 0xFF;
    // speed_fil
    uart_array[8] = (speed_fil >> 8) & 0xFF;
    uart_array[9] = speed_fil & 0xFF;
    // freq sin
    uart_array[10] = f_array[i] & 0xFF;

    DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
    DMA1_Channel7->CNDTR = UART_ARRAY_LEN;
    DMA1_Channel7->CCR |= DMA_CCR7_EN;

}