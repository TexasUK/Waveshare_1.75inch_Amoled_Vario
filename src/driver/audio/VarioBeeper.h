#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"

class VarioBeeper {
public:
  VarioBeeper();
  ~VarioBeeper();
  
  bool begin();
  void setVz(float vz_mps);
  void setMuted(bool muted);
  
  static const int SAMPLE_RATE = 16000;
  static const int FRAMES = 256;
  static const float OUTPUT_SCALE;
  
private:
  static void beeper_task(void* parameter);
  
  TaskHandle_t task_handle_ = nullptr;
  SemaphoreHandle_t mutex_ = nullptr;
  float vz_mps_ = 0.0f;
  bool muted_ = false;
};