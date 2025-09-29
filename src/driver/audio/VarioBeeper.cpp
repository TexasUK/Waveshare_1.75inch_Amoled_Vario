#include "VarioBeeper.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float VarioBeeper::OUTPUT_SCALE = 0.25f;

struct ToneShape { 
  float f, on_ms, off_ms; 
  bool continuous; 
};

static inline float clampf(float x, float lo, float hi){ 
  return x < lo ? lo : (x > hi ? hi : x); 
}

static void shape_from_vz(float vz, ToneShape& s) {
  if (vz > -0.3f && vz < +0.25f) { 
    s = {0, 0, 140, false}; return; 
  }
  if (vz <= -2.0f) {
    float a = clampf(fabsf(vz), 2.0f, 6.0f);
    s = { clampf(320.0f + 110.0f*(a-2.0f), 320.0f, 760.0f), 1000.0f, 0.0f, true };
    return;
  }
  if (vz < -0.3f) {
    float a = clampf(((-vz) - 0.3f) / 1.7f, 0.0f, 1.0f);
    s = { 380.0f - 120.0f*a, 140.0f + 160.0f*(1.0f-a), 420.0f + 280.0f*a, false };
    return;
  }
  float a = clampf((vz - 0.25f) / 3.0f, 0.0f, 1.0f);
  float period = 560.0f - 430.0f*a;
  float duty = 0.35f + 0.30f*a;
  s = { 700.0f + 900.0f*a, period*duty, period*(1.0f - duty), false };
}

static inline int16_t next_sine(float& phase, float f_hz) {
  float s = sinf(phase) * VarioBeeper::OUTPUT_SCALE;
  phase += 2.0f * (float)M_PI * (f_hz / (float)VarioBeeper::SAMPLE_RATE);
  if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
  return (int16_t)(s * 32767.0f);
}

VarioBeeper::VarioBeeper() {}

VarioBeeper::~VarioBeeper() {
  if (task_handle_) vTaskDelete(task_handle_);
  if (mutex_) vSemaphoreDelete(mutex_);
}

bool VarioBeeper::begin() {
  mutex_ = xSemaphoreCreateMutex();
  if (!mutex_) return false;
  
  BaseType_t result = xTaskCreatePinnedToCore(
    beeper_task, "vario_beep", 4096, this, 5, &task_handle_, tskNO_AFFINITY
  );
  return (result == pdPASS);
}

void VarioBeeper::setVz(float vz_mps) {
  if (xSemaphoreTake(mutex_, portMAX_DELAY)) {
    vz_mps_ = vz_mps;
    xSemaphoreGive(mutex_);
  }
}

void VarioBeeper::setMuted(bool muted) {
  if (xSemaphoreTake(mutex_, portMAX_DELAY)) {
    muted_ = muted;
    xSemaphoreGive(mutex_);
  }
}

void VarioBeeper::beeper_task(void* parameter) {
  VarioBeeper* beeper = static_cast<VarioBeeper*>(parameter);
  
  const size_t samples = VarioBeeper::FRAMES * 2;
  const size_t bytes = samples * sizeof(int16_t);
  int16_t* buf = (int16_t*)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) { vTaskDelete(nullptr); return; }
  
  float phase = 0.0f;
  uint32_t gate_us_left = 0;
  bool gate_on = false;

  for (;;) {
    float vz = 0.0f; bool muted = false;
    if (xSemaphoreTake(beeper->mutex_, portMAX_DELAY)) {
      vz = beeper->vz_mps_; muted = beeper->muted_; 
      xSemaphoreGive(beeper->mutex_);
    }

    ToneShape sh; shape_from_vz(vz, sh);

    if (muted || sh.f < 1.0f) {
      memset(buf, 0, bytes);
      size_t w = 0; i2s_write(I2S_NUM_0, buf, bytes, &w, portMAX_DELAY);
      gate_us_left = 0; gate_on = false; continue;
    }

    if (gate_us_left == 0) {
      gate_on = sh.continuous ? true : !gate_on;
      gate_us_left = (uint32_t)((gate_on ? sh.on_ms : sh.off_ms) * 1000.0f);
    }

    uint32_t chunk_us = (uint32_t)((VarioBeeper::FRAMES * 1000000ULL) / VarioBeeper::SAMPLE_RATE);
    size_t frames = VarioBeeper::FRAMES;
    if (!sh.continuous && gate_us_left < chunk_us) {
      chunk_us = gate_us_left;
      frames = (size_t)((gate_us_left * (uint64_t)VarioBeeper::SAMPLE_RATE) / 1000000ULL);
    }

    const bool play = gate_on || sh.continuous;
    const size_t frames_to_write = frames;
    const size_t samples_to_write = frames_to_write * 2;
    
    for (size_t i = 0; i < frames_to_write; ++i) {
      int16_t s = play ? next_sine(phase, sh.f) : 0;
      buf[2*i+0] = s; buf[2*i+1] = s;
    }
    for (size_t i = samples_to_write; i < samples; ++i) buf[i] = 0;

    size_t w = 0; i2s_write(I2S_NUM_0, buf, bytes, &w, portMAX_DELAY);

    if (!sh.continuous) {
      if (gate_us_left > chunk_us) gate_us_left -= chunk_us; 
      else gate_us_left = 0;
      if (gate_us_left == 0 && !gate_on && sh.off_ms <= 0.0f) gate_on = true;
    }
  }
  heap_caps_free(buf);
}