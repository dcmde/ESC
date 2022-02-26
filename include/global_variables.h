#include "stm32f10x_conf.h"

#pragma once

extern volatile float u;
extern uint16_t data_adc[12];
extern char uart_array[UART_ARRAY_LEN];
extern volatile uint32_t timeS_1kHz;
extern volatile int16_t theta_offset;
