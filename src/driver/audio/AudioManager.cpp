#include "AudioManager.h"
#include "es8311.h"
#include "VarioBeeper.h"
#include "../pins_config.h"

static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int AUDIO_SAMPLE_RATE = 16000;

AudioManager::AudioManager() {
}

AudioManager::~AudioManager() {
  if (beeper_) delete beeper_;
}

bool AudioManager::begin(int volume_level) {
  Serial.println("[AUDIO] Initializing audio system...");
  
  if (!i2s_init()) {
    Serial.println("[AUDIO] Failed to initialize I2S");
    return false;
  }
  
  if (!es8311_init_codec(volume_level)) {
    Serial.println("[AUDIO] Failed to initialize ES8311");
    return false;
  }
  
  beeper_ = new VarioBeeper();
  if (!beeper_->begin()) {
    Serial.println("[AUDIO] Failed to start vario beeper");
    return false;
  }
  
  Serial.println("[AUDIO] Audio system initialized successfully");
  return true;
}

bool AudioManager::i2s_init() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = AUDIO_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 6;
  cfg.dma_buf_len = 256;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = AUDIO_SAMPLE_RATE * 256;

  if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) return false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = BCLKPIN;
  pins.ws_io_num = WSPIN;
  pins.data_out_num = DOPIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  pins.mck_io_num = MCLK_OUT_PIN;

  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) return false;
  if (i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO) != ESP_OK) return false;
  
  return true;
}

bool AudioManager::es8311_init_codec(int volume_level) {
  codec_ = es8311_create(0, ES8311_ADDRRES_0);
  if (!codec_) {
    Serial.println("[AUDIO] ES8311 create failed");
    return false;
  }

  es8311_clock_config_t clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = AUDIO_SAMPLE_RATE * 256,
    .sample_frequency = AUDIO_SAMPLE_RATE
  };

  if (::es8311_init(codec_, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) {
    Serial.println("[AUDIO] ES8311 init failed");
    return false;
  }
  
  es8311_sample_frequency_config(codec_, clk.mclk_frequency, clk.sample_frequency);
  es8311_microphone_config(codec_, false);
  
  setVolume(volume_level);

  pinMode(PAPIN, OUTPUT);
  digitalWrite(PAPIN, HIGH);
  
  return true;
}

void AudioManager::setVolume(int volume_level) {
  int es8311_volume = volumeToES8311(volume_level);
  if (codec_) {
    es8311_voice_volume_set(codec_, es8311_volume, nullptr);
  }
}

void AudioManager::setVario(float vz_mps) {
  if (beeper_) beeper_->setVz(vz_mps);
}

int AudioManager::volumeToES8311(int volume_level) {
  volume_level = constrain(volume_level, 0, 10);
  return map(volume_level, 0, 10, 0, 100);
}