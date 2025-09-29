#pragma once
#include <Preferences.h>

struct AppSettings {
  int audio_volume = 7;
  int display_brightness = 200;
  bool use_metric = false;
};

class SettingsManager {
public:
  void begin();
  void save();
  void load();
  
  void setVolume(int volume);
  void setBrightness(int brightness);
  void setUseMetric(bool use_metric);
  
  AppSettings settings;

private:
  Preferences prefs_;
};