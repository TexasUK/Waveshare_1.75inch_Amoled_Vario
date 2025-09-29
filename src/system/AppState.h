#pragma once
#include <Arduino.h>

struct AppState {
    bool in_settings = false;
    bool touch_initialized = false;
    uint32_t last_touch_time = 0;
    bool calibrating = false;
    
    // Demo data (replace with real sensors later)
    float vario_mps = 0;
    float altitude_ft = 3962;
    float airspeed_kts = 55;
};

extern AppState g_app_state;