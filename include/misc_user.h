#include "stm32f10x_conf.h"

#pragma once
#define half_pi 0x0006487E
#define MUL 262144.000000f

extern uint16_t data_adc[12];
extern char uart_array[UART_ARRAY_LEN];
extern volatile uint32_t timeS_1kHz;

typedef struct {
    float u;
    uint16_t theta;
    int16_t theta_offset;
    int16_t speed;
    int16_t speed_filt;
    uint32_t time_xxHz;
} motor_control_struct_t;

typedef void (*f_ptr_t)(volatile motor_control_struct_t *);

void cordic(int theta, int *s, int *c);

float sin_time_1kHz(int Te);

int16_t get_speed(uint16_t theta_cur_pts);

void set_loop(f_ptr_t f_ptr);

void loop_run(volatile motor_control_struct_t *motorControlStruct);

void empty_loop(volatile motor_control_struct_t *motorControlStruct);
