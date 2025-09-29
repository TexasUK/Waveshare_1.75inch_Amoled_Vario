#pragma once
#include "es8311.h"
#include <Arduino.h>
#include "driver/i2s.h"

class VarioBeeper;

class AudioManager {
public:
  AudioManager();
  ~AudioManager();
  
  bool begin(int volume_level);
  void setVolume(int volume_level);
  void setVario(float vz_mps);
  
  static int volumeToES8311(int volume_level);

private:
  bool i2s_init();
  bool es8311_init_codec(int volume_level);
  
  es8311_handle_t codec_ = nullptr;
  VarioBeeper* beeper_ = nullptr;
};