#include <stdint.h>
#include "global_variables.h"

volatile float u = 0;
uint16_t data_adc[12];
char uart_array[UART_ARRAY_LEN] = {0};
volatile uint32_t timeS_1kHz = 0;
volatile int16_t theta_offset = 0;
