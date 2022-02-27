#include "stm32f10x_conf.h"

#pragma once

extern volatile float u;
extern uint16_t data_adc[12];
extern char uart_array[UART_ARRAY_LEN];
extern volatile uint32_t timeS_1kHz;
extern volatile int16_t theta_offset;
extern volatile int16_t speed_cur;

typedef int16_t (*f_ptr_t)(int16_t);

int16_t get_speed(uint16_t theta_cur_pts, uint8_t speed_dir);

void set_loop_tim3(f_ptr_t f_ptr);

int16_t loop_tim3(int16_t speed);
