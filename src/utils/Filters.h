// Simple median ring buffer and EMA helper
#pragma once

#include <math.h>
#include <stddef.h>

template <size_t N>
struct Ring {
  float d[N];
  size_t idx = 0, sz = 0;
  void push(float v) {
    d[idx] = v;
    idx = (idx + 1) % N;
    if (sz < N) sz++;
  }
  float median() const {
    if (!sz) return 0.0f;
    float tmp[N];
    for (size_t i = 0; i < sz; ++i) tmp[i] = d[i];
    for (size_t i = 1; i < sz; ++i) {
      float v = tmp[i];
      size_t j = i;
      while (j > 0 && tmp[j - 1] > v) { tmp[j] = tmp[j - 1]; --j; }
      tmp[j] = v;
    }
    return (sz & 1) ? tmp[sz / 2] : 0.5f * (tmp[sz / 2 - 1] + tmp[sz / 2]);
  }
};

static inline float emaUpdate(float prev, float value, float alpha) {
  return (1.0f - alpha) * prev + alpha * value;
}

