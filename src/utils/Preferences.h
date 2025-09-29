#pragma once
#include <Preferences.h>  // Use Arduino's NVS-based Preferences

class SettingsManager {
public:
    struct AppSettings {
        int audio_volume = 7;
        int display_brightness = 200; 
        bool use_metric = false;
    };

    void begin();
    void save();
    void load();
    
    AppSettings settings;

private:
    Preferences prefs_;
};