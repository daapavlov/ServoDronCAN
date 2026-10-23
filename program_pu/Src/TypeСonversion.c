#include <TypeConversion.h>
#include "stm32f1xx_hal.h"
float uint16_to_float(uint16_t x)
{
    uint32_t sign = (x & 0x8000) << 16;
    uint32_t exp  = (x & 0x7C00) >> 10;
    uint32_t mant = (x & 0x03FF);
    uint32_t f_bits;

    if (exp == 0) {
        if (mant != 0) {
            // Денормализованное число: приводим к нормализованному виду для float32
            exp = 127 - 15;
            while ((mant & 0x0400) == 0) {
                mant <<= 1;
                exp--;
            }
            f_bits = sign | ((exp + 1) << 23) | ((mant & 0x03FF) << 13);
        } else {
            f_bits = sign; // Точный ноль (+0.0 или -0.0)
        }
    } else if (exp == 31) {
        // Infinity или NaN
        f_bits = sign | 0x7F800000 | (mant << 13);
    } else {
        // Нормализованное число
        f_bits = sign | ((exp - 15 + 127) << 23) | (mant << 13);
    }

    float result;
    memcpy(&result, &f_bits, sizeof(result));
    return result;
}
uint16_t float_to_uint16_t(float x)
{
    uint32_t f_bits;
    memcpy(&f_bits, &x, sizeof(f_bits));

    uint32_t sign = (f_bits >> 16) & 0x8000;      // бит 31 → бит 15
    uint32_t exp  = (f_bits >> 23) & 0xFF;        // 8 бит экспоненты float32
    uint32_t mant = f_bits & 0x007FFFFF;           // 23 бита мантиссы float32

    // Особые случаи: NaN, Inf
    if (exp == 0xFF) {
        return (uint16_t)(sign | 0x7C00 | ((mant >> 13) & 0x3FF));
    }

    // Нули и денормализованные float32 (exp=0)
    if (exp == 0) {
        if (mant == 0) {
            return (uint16_t)sign; // +0 или -0
        }
        // Нормализуем вручную, чтобы корректно округлить
        uint32_t shift = 0;
        uint32_t m = mant;
        while ((m & 0x00800000) == 0 && m != 0) {
            m <<= 1;
            shift++;
        }
        int32_t effective_exp = 1 - 127 - (int32_t)shift;
        exp = (uint32_t)effective_exp;
        mant = m;
    }

    int32_t new_exp = (int32_t)exp - 127 + 15; // конвертация bias: 127 → 15

    // Округление мантиссы до 10 бит (round to nearest, ties to even)
    uint32_t round_bit = (mant >> 12) & 1;
    uint32_t sticky_bit = (mant & 0x00000FFF) != 0;
    mant = (mant >> 13) & 0x03FF;

    if (round_bit) {
        if (sticky_bit || (mant & 1)) {
            mant++;
            if (mant == 0x400) {
                mant = 0;
                new_exp++;
            }
        } else if (mant & 1) { // ровно половина, младший бит 1 → округляем вверх
            mant++;
            if (mant == 0x400) {
                mant = 0;
                new_exp++;
            }
        }
    }

    // Обработка переполнения/поднормализованных
    if (new_exp >= 31) {
        return (uint16_t)(sign | 0x7C00); // Infinity
    }
    if (new_exp <= 0) {
        if (new_exp <= -15) {
            return (uint16_t)sign; // слишком мало → ноль
        }
        int32_t shift = 1 - new_exp;
        mant >>= shift;
        new_exp = 0;
    }

    return (uint16_t)(sign | ((uint32_t)new_exp << 10) | mant);
}


float LinearTransformation_f(uint16_t value, uint16_t in_min, uint16_t in_max,
                   float out_min, float out_max)
{
    // Защита от деления на ноль
    if (in_min == in_max) {
        return out_min;
    }

    float in_range = (float)in_max - in_min;
    float out_range = (float)out_max - out_min;

    // Формула: out = out_min + (value - in_min) * out_range / in_range
    float result = (float)out_min +
                     (((float)value - in_min) * out_range) / in_range;

    return result;
}
uint16_t LinearTransformation_u16(float value, float in_min, float in_max,
                   uint16_t out_min, uint16_t out_max)
{
    // Защита от деления на ноль
    if (in_min == in_max) {
        return out_min;
    }

    uint16_t in_range = (uint16_t)(in_max - in_min);
    uint16_t out_range = (uint16_t)(out_max - out_min);

    // Формула: out = out_min + (value - in_min) * out_range / in_range
    uint16_t result = (uint16_t)(out_min +
                     (((uint16_t)value - in_min) * out_range) / in_range);

    return result;
}