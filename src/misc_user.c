#include <stdint.h>
#include "misc_user.h"

uint16_t data_adc[12];
char uart_array[UART_ARRAY_LEN] = {0};
volatile uint32_t timeS_1kHz = 0;

f_ptr_t f_loop_tim3 = empty_loop;

void empty_loop(volatile motor_control_struct_t *motorControlStruct) {
    motorControlStruct->u = 0.f;
    motorControlStruct->theta_offset = 0;
}

int16_t get_speed(uint16_t theta_cur_pts) {
    static uint16_t theta_prev_pts = 0;
    int16_t delta_theta = theta_cur_pts - theta_prev_pts;
    int8_t delta_theta_sign = delta_theta > 0 ? 1 : -1;
    if (delta_theta * delta_theta_sign > (ENC_MAX_PTS >> 1)) {
        delta_theta -= delta_theta_sign * ENC_MAX_PTS;
    }
    // Update variable
    theta_prev_pts = theta_cur_pts;
    return delta_theta;
}

void set_loop(f_ptr_t f_ptr) {
    f_loop_tim3 = f_ptr;
}

void loop_run(volatile motor_control_struct_t *motorControlStruct) {
    return f_loop_tim3(motorControlStruct);
}
