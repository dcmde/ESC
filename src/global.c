#include <stdint.h>
#include "global.h"

volatile float u = 0.3f;
uint16_t data_adc[12];
char uart_array[UART_ARRAY_LEN] = {0};
volatile uint32_t timeS_1kHz = 0;
volatile int16_t theta_offset = 0, speed_cur = 0, speed_fil = 0;

f_ptr_t f_loop_tim3;

int16_t get_speed(uint16_t theta_cur_pts, uint8_t speed_dir) {
    static uint16_t theta_prev_pts = 0;
    if (speed_dir) {
        if (theta_prev_pts < theta_cur_pts) {
            speed_cur = -theta_cur_pts + theta_prev_pts + ENC_MAX_PTS;
        } else {
            speed_cur = theta_cur_pts - theta_prev_pts;
        }
    } else {
        if (theta_prev_pts > theta_cur_pts) {
            speed_cur = theta_cur_pts - theta_prev_pts + ENC_MAX_PTS;
        } else {
            speed_cur = theta_cur_pts - theta_prev_pts;
        }
    }
    theta_prev_pts = theta_cur_pts;
    return speed_cur;
}

void set_loop_tim3(f_ptr_t f_ptr) {
    f_loop_tim3 = f_ptr;
}

int16_t loop_tim3(int16_t speed) {
    return f_loop_tim3(speed);
}