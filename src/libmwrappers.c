#include <math.h>

float __ieee754_logf(float);
float __ieee754_log10f(float);
float __ieee754_atan2f(float, float);
float __ieee754_powf(float, float);

float logf(float x) {
    return __ieee754_logf(x);
}

float log10f(float x) {
    return __ieee754_log10f(x);
}

float atan2f(float y, float x) {
    return __ieee754_atan2f(y, x);
}

float powf(float x, float y) {
    return __ieee754_powf(x, y);
}

float __wrap_log10f(float x) {
    return __ieee754_log10f(x);
}

float __wrap_atan2f(float y, float x) {
    return __ieee754_atan2f(y, x);
}

float __wrap_powf(float x, float y) {
    return __ieee754_powf(x, y);
}
