#ifndef TypeConversion_H
#define TypeConversion_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

float uint16_to_float(uint16_t x);
uint16_t float_to_uint16_t(float x);
uint16_t LinearTransformation_u16(uint16_t value, uint16_t in_min, uint16_t in_max,
                   uint16_t out_min, uint16_t out_max);
float LinearTransformation_f(uint16_t value, uint16_t in_min, uint16_t in_max,
                   float out_min, float out_max);

#endif