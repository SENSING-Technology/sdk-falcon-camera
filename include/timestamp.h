#ifndef __MYTIMESTAMP__
#define __MYTIMESTAMP__
#ifdef _WINDOWS
#include <Windows.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void lc_reset(TIMESTAMPINFO *info);

int lc_is_full(TIMESTAMPINFO *info);

void lc_add_value(TIMESTAMPINFO *info, CSample val);

void lc_add_const_y_coefs(TIMESTAMPINFO *info, double dy);

void lc_calc_linear_coefs(TIMESTAMPINFO *info);

void lc_get_a_b(TIMESTAMPINFO *info, double x, double *a, double *b);

double lc_calc_value(TIMESTAMPINFO *info, double x);

int lc_update_samples_base(TIMESTAMPINFO *info, double x);

void lc_update_last_sample_time(TIMESTAMPINFO *info, double x);
double get_system_hw_time(TIMESTAMPINFO *info, MUTEXHANDLE mutex, double timestamp, int is_ready);
double get_system_hw_time_simple(TIMESTAMPINFO *info, MUTEXHANDLE mutex, double timestamp, int is_ready);
#ifdef __cplusplus
}
#endif
#endif
