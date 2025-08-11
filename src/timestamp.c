#include <stdio.h>
#include <math.h>
#ifdef _WINDOWS
#include <Windows.h>
#else
#include <pthread.h>
#endif
#include <stdlib.h>
#include "nvpfm.h"
#include "utils.h"
#include "timestamp.h"
static const double TIMESTAMP_USEC_TO_MSEC = 0.001;
static const double _time_span_ms = 1000;
static const unsigned int _buffer_size = 15;

void deque_clear(sampledeque **psampledeque) {
  sampledeque *tmp = *psampledeque;
  while (NULL != tmp) {
    sampledeque *tobefree = tmp;
    tmp = tmp->next;
    free(tobefree);
  }
  *psampledeque = NULL;
}
CSample deque_back(sampledeque *psampledeque) {
  sampledeque *tmp = psampledeque;
  while (tmp != NULL) {
    if (tmp->next == NULL) {
      return tmp->sample;
    }
  }
  CSample sample = {0, 0};
  return sample;
}

CSample deque_front(sampledeque *psampledeque) {
  return psampledeque->sample;
}
void deque_pop_back(sampledeque **psampledeque) {
  sampledeque *tmp = *psampledeque;
  while (NULL != tmp) {
    if (tmp->next == NULL) {
      sampledeque *last = tmp->prev;
      free(tmp);
      last->next = NULL;
      break;
    }
    tmp = tmp->next;
  }
}
void deque_push_front(sampledeque **psampledeque, CSample value) {
  sampledeque *tobeadd = (sampledeque *)calloc(1, sizeof(sampledeque));
  tobeadd->sample = value;

  if (*psampledeque == NULL) {
    *psampledeque = tobeadd;
  } else {
    tobeadd->next = *psampledeque;
    (*psampledeque)->prev = tobeadd;
    *psampledeque = tobeadd;
  }
}
unsigned int deque_size(sampledeque *psampledeque) {
  unsigned int count = 0;
  sampledeque *tmp = psampledeque;
  while (NULL != tmp) {
    count++;
    tmp = tmp->next;
  }
  return count;
}

CSample *subsample(CSample *from, CSample *other) {
  from->_x -= other->_x;
  from->_y -= other->_y;
  return from;
}
CSample *addsample(CSample *from, CSample *other) {
  from->_x += other->_x;
  from->_y += other->_y;
  return from;
}

void lc_reset(TIMESTAMPINFO *info) {
  deque_clear(&info->_last_values);
}

int lc_is_full(TIMESTAMPINFO *info) {
  return deque_size(info->_last_values) >= _buffer_size;
}

void lc_add_value(TIMESTAMPINFO *info, CSample val) {
  while (deque_size(info->_last_values) > _buffer_size) {
    deque_pop_back(&info->_last_values);
  }
  deque_push_front(&info->_last_values, val);
  lc_calc_linear_coefs(info);
}

void lc_add_const_y_coefs(TIMESTAMPINFO *info, double dy) {
  sampledeque *tmp = info->_last_values;
  while (tmp != NULL) {
    tmp->sample._y += dy;
    tmp = tmp->next;
  }
}

void lc_calc_linear_coefs(TIMESTAMPINFO *info) {
  // Calculate linear coefficients, based on calculus described in: https://www.statisticshowto.datasciencecentral.com/probability-and-statistics/regression-analysis/find-a-linear-regression-equation/
  // Calculate Std
  double n = ((double)(deque_size(info->_last_values)));
  double a = 1;
  double b = 0;
  double dt = 1;
  if (n == 1) {
    info->_base_sample = deque_back(info->_last_values);
    info->_dest_a = 1;
    info->_dest_b = 0;
    info->_prev_a = 0;
    info->_prev_b = 0;
    info->_last_request_time = deque_front(info->_last_values)._x;
  } else {
    double sum_x = 0;
    double sum_y = 0;
    double sum_xy = 0;
    double sum_x2 = 0;
    sampledeque *tmpdeque = info->_last_values;
    while (tmpdeque != NULL)
    //   for (auto sample = _last_values.begin(); sample != _last_values.end(); sample++)
    {
      CSample crnt_sample = tmpdeque->sample;
      subsample(&crnt_sample, &info->_base_sample);
      sum_x += crnt_sample._x;
      sum_y += crnt_sample._y;
      sum_xy += (crnt_sample._x * crnt_sample._y);
      sum_x2 += (crnt_sample._x * crnt_sample._x);

      tmpdeque = tmpdeque->next;
    }
    b = (sum_y * sum_x2 - sum_x * sum_xy) / (n * sum_x2 - sum_x * sum_x);
    a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);

    if (info->_last_request_time - info->_prev_time < _time_span_ms) {
      dt = (info->_last_request_time - info->_prev_time) / _time_span_ms;
    }
  }
  info->_prev_a = info->_dest_a * dt + info->_prev_a * (1 - dt);
  info->_prev_b = info->_dest_b * dt + info->_prev_b * (1 - dt);
  info->_dest_a = a;
  info->_dest_b = b;
  info->_prev_time = info->_last_request_time;
}

void lc_get_a_b(TIMESTAMPINFO *info, double x, double *a, double *b) {
  *a = info->_dest_a;
  *b = info->_dest_b;
  if (x - info->_prev_time < _time_span_ms) // 如果设备时间离最近一次设备时间小于1000ms,计算新的a、b
  {
    double dt = (x - info->_prev_time) / _time_span_ms;
    *a = info->_dest_a * dt + info->_prev_a * (1 - dt);
    *b = info->_dest_b * dt + info->_prev_b * (1 - dt);
  }
}

double lc_calc_value(TIMESTAMPINFO *info, double x) {
  double a, b;
  lc_get_a_b(info, x, &a, &b);
  double y = a * (x - info->_base_sample._x) + b + info->_base_sample._y;
  // printf(__FUNCTION__ << ": " << x << " -> " << y << " with coefs:" << a << ", " << b << ", " << _base_sample._x << ", " << _base_sample._y);
  return y;
}

int lc_update_samples_base(TIMESTAMPINFO *info, double x) {

  if (info->max_device_time < 1.0)
    info->max_device_time = pow(2, 32) * TIMESTAMP_USEC_TO_MSEC;

  double base_x;
  if (info->_last_values == NULL)
    return 0;
  if ((deque_front(info->_last_values)._x - x) > info->max_device_time / 2)
    base_x = info->max_device_time;
  else if ((x - deque_front(info->_last_values)._x) > info->max_device_time / 2)
    base_x = -info->max_device_time;
  else
    return 0;
  // printf(__FUNCTION__ << "(" << base_x << ")");

  double a, b;
  lc_get_a_b(info, x + base_x, &a, &b);
  sampledeque *tmpdeque = info->_last_values;
  while (tmpdeque != NULL)
  // for (auto &&sample : _last_values)
  {
    tmpdeque->sample._x -= base_x;

    tmpdeque = tmpdeque->next;
  }
  info->_prev_time -= base_x;
  info->_base_sample._y += a * base_x;
  return 1;
}

void lc_update_last_sample_time(TIMESTAMPINFO *info, double x) {
  info->_last_request_time = x;
}
void MUTEXLOCK(MUTEXHANDLE handle);
void MUTEXUNLOCK(MUTEXHANDLE handle);
double get_system_hw_time(TIMESTAMPINFO *info, MUTEXHANDLE mutex, double timestamp, int is_ready) {
  //    printf("will get global time of timestamp:%f\n", timestamp);
  MUTEXLOCK(mutex);
  if (is_ready) {
    // printf("it is ready!\n");
    lc_update_samples_base(info, timestamp);
    lc_update_last_sample_time(info, timestamp);
    double result = lc_calc_value(info, timestamp);
    MUTEXUNLOCK(mutex);
    return result;
  } else {
    //       printf("it is not ready,just return origin timestamp!\n");
    MUTEXUNLOCK(mutex);
    return timestamp;
  }
}

double lc_simple_value(TIMESTAMPINFO *info, double x) {
  CSample tmp = deque_front(info->_last_values);
  double y = x - tmp._x + tmp._y;
  return y;
}

double get_system_hw_time_simple(TIMESTAMPINFO *info, MUTEXHANDLE mutex, double timestamp, int is_ready) {
  //    printf("will get global time of timestamp:%f\n", timestamp);
  MUTEXLOCK(mutex);
  if (is_ready) {
    // printf("it is ready!\n");
    lc_update_samples_base(info, timestamp);
    lc_update_last_sample_time(info, timestamp);
    double result = lc_simple_value(info, timestamp);
    MUTEXUNLOCK(mutex);
    return result;
  } else {
    //       printf("it is not ready,just return origin timestamp!\n");
    MUTEXUNLOCK(mutex);
    return timestamp;
  }
}