#include "SettingsManager.h"

void SettingsManager::begin() {
  prefs_.begin("vario-app", false);
  load();
}

void SettingsManager::save() {
  prefs_.putInt("volume", settings.audio_volume);
  prefs_.putInt("brightness", settings.display_brightness);
  prefs_.putBool("metric", settings.use_metric);
}

void SettingsManager::load() {
  settings.audio_volume = prefs_.getInt("volume", 7);
  settings.display_brightness = prefs_.getInt("brightness", 200);
  settings.use_metric = prefs_.getBool("metric", false);
}

void SettingsManager::setVolume(int volume) {
  settings.audio_volume = volume;
  save();
}

void SettingsManager::setBrightness(int brightness) {
  settings.display_brightness = brightness;
  save();
}

void SettingsManager::setUseMetric(bool use_metric) {
  settings.use_metric = use_metric;
  save();
}