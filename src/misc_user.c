#include <stdint.h>
#include "misc_user.h"
#include <math.h>

#define cordic_1K 0x00026DD3
#define CORDIC_NTAB 20
int cordic_ctab[] = {0x0003243F, 0x0001DAC6, 0x0000FADB, 0x00007F56, 0x00003FEA, 0x00001FFD, 0x00000FFF, 0x000007FF,
                     0x000003FF, 0x000001FF, 0x000000FF, 0x0000007F, 0x0000003F, 0x0000001F, 0x0000000F, 0x00000007,
                     0x00000003, 0x00000001, 0x00000000, 0x00000000,};

typedef enum {
    NORM,
    INF_PI_2,
    SUP_PI_2
} CORDIC_CASE;

void cordic(int theta, int *s, int *c) {
    int k, d, tx, ty, tz;
    CORDIC_CASE cCase = NORM;
    int x = cordic_1K, y = 0, z;

    // Set theta between -2.pi and 2.pi
    theta %= (half_pi << 2);
    // Map the angle between -pi and pi
    if (theta > (half_pi << 1)) {
        theta -= (half_pi << 2);
    } else if (theta < -(half_pi << 1)) {
        theta += (half_pi << 2);
    }
    // Logic aimed at finding the right value for
    // angle between -pi and pi towards -pi/2 and pi/2
    if (theta > half_pi) {
        theta -= half_pi;
        cCase = SUP_PI_2;
    } else if (theta < -half_pi) {
        theta += half_pi;
        cCase = INF_PI_2;
    }

    // Compute the angle
    z = theta;
    for (k = 0; k < CORDIC_NTAB; ++k) {
//        d = z>>32;
        //get sign. for other architectures, you might want to use the more portable version
        d = z >= 0 ? 0 : -1;
        tx = x - (((y >> k) ^ d) - d);
        ty = y + (((x >> k) ^ d) - d);
        tz = z - ((cordic_ctab[k] ^ d) - d);
        x = tx;
        y = ty;
        z = tz;
    }
    // Find the right angle according to the previous logic
    switch (cCase) {
        case NORM:
            *c = x;
            *s = y;
            break;
        case INF_PI_2:
            *c = y;
            *s = -x;
            break;
        case SUP_PI_2:
            *c = -y;
            *s = x;
            break;
    }
}

float sin_time_1kHz(int Te) {
    static time_s_1kHz = 0;
    ++time_s_1kHz;
    int theta;
    int k, d, tx, ty, tz;
    int x = cordic_1K, y = 0, z;
    time_s_1kHz = time_s_1kHz > Te ? 0 : time_s_1kHz;

    float time_s_f;
    time_s_f = time_s_1kHz;

    theta = (int) (2 * M_PI * time_s_f * MUL / (float) Te);

    if (theta > (half_pi << 1)) {
        if (theta > ((half_pi << 1) + half_pi)) {
            theta -= (half_pi << 2);
        } else {
            theta = -half_pi + (-theta + ((half_pi << 1) + half_pi));
        }
    } else {
        if (theta > half_pi) {
            theta = half_pi - (theta - half_pi);
        }
    }

    z = theta;
    for (k = 0; k < CORDIC_NTAB; ++k) {
        d = z >= 0 ? 0 : -1;
        tx = x - (((y >> k) ^ d) - d);
        ty = y + (((x >> k) ^ d) - d);
        tz = z - ((cordic_ctab[k] ^ d) - d);
        x = tx;
        y = ty;
        z = tz;
    }
    return (float) y / MUL;

}

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
