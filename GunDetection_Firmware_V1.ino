/* EE300W Acoustic Detection System - Firmware V1.0
 * Target: classic ESP32 (ESP32-D0WD / ESP32-WROOM-32)
 * Microphone: INMP441 digital I2S, LEFT channel (L/R -> GND)
 * Arduino-ESP32 core 2.x or 3.x with legacy driver/i2s.h available.
 * Experimental acoustic impulse detector, NOT a validated gunshot classifier.
 */
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
static constexpr int PIN_BCLK = 26;
static constexpr int PIN_WS = 25;
static constexpr int PIN_DATA = 33;
static constexpr int PIN_BUZZER = 27;
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr size_t FRAME_SAMPLES = 512; // 32 ms at 16 kHz
static constexpr float MIN_RMS = 0.006f;       // normalized, tune after observing real microphone values
static constexpr float TRIGGER_RATIO = 4.0f;
static constexpr float MIN_CREST = 2.2f;
static constexpr uint32_t REFRACTORY_MS = 450;
static constexpr uint32_t BUZZER_MS = 120;
static constexpr uint32_t LOG_INTERVAL_MS = 250;

static int32_t rawAudio[FRAME_SAMPLES];
static float noiseFloor = 0.002f;
static uint32_t eventCount = 0;
static uint32_t lastEventMs = 0;
static uint32_t lastLogMs = 0;
static uint32_t buzzerOffAt = 0;
static bool buzzerOn = false;
static bool firstEvent = true;

static void startI2S() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;
  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) { Serial.printf("I2S driver error: %d\n", (int)err); while (true) delay(1000); }
  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_BCLK;
  pins.ws_io_num = PIN_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_DATA;
  err = i2s_set_pin(I2S_PORT, &pins);
  if (err != ESP_OK) { Serial.printf("I2S pin error: %d\n", (int)err); while (true) delay(1000); }
  i2s_zero_dma_buffer(I2S_PORT);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  delay(800);
  Serial.println("EE300W Acoustic Detection Firmware V1.0");
  Serial.println("INMP441: BCLK=26 WS=25 SD=33 L/R=GND");
  Serial.println("NOTE: Candidate acoustic events only; not gunshot classification.");
  startI2S();
  Serial.println("I2S initialized. Start testing with ordinary claps/taps.");
}

void loop() {
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(I2S_PORT, rawAudio, sizeof(rawAudio), &bytesRead, pdMS_TO_TICKS(100));
  uint32_t now = millis();
  if (buzzerOn && (int32_t)(now - buzzerOffAt) >= 0) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn = false;
  }
  if (err != ESP_OK || bytesRead == 0) return;
  const size_t n = bytesRead / sizeof(int32_t);
  if (n < 16) return;

  // INMP441 commonly outputs signed 24-bit samples in 32-bit I2S slots.
  // Shift right by 8 and normalize against signed 24-bit full scale.
  double mean = 0.0;
  for (size_t i = 0; i < n; ++i) mean += (float)(rawAudio[i] >> 8) / 8388608.0f;
  mean /= n;
  double sumSquares = 0.0;
  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    float x = (float)(rawAudio[i] >> 8) / 8388608.0f - (float)mean;
    sumSquares += (double)x * x;
    float a = fabsf(x);
    if (a > peak) peak = a;
  }
  float rms = sqrtf((float)(sumSquares / n));
  float crest = peak / fmaxf(rms, 0.000001f);
  float threshold = fmaxf(MIN_RMS, noiseFloor * TRIGGER_RATIO);
  bool candidate = rms > threshold && crest >= MIN_CREST;
  bool ready = firstEvent || (uint32_t)(now - lastEventMs) >= REFRACTORY_MS;

  if (candidate && ready) {
    firstEvent = false;
    lastEventMs = now;
    ++eventCount;
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerOn = true;
    buzzerOffAt = now + BUZZER_MS;
    Serial.printf("EVENT,%lu,rms=%.5f,peak=%.5f,crest=%.2f,threshold=%.5f,time_ms=%lu\n",
                  (unsigned long)eventCount, rms, peak, crest, threshold, (unsigned long)now);
  }
  // Update the ambient noise estimate only when not detecting a candidate.
  // Fast downward adaptation and slow upward adaptation reduce noise bursts inflating the floor.
  if (!candidate) {
    float alpha = rms < noiseFloor ? 0.03f : 0.002f;
    noiseFloor = (1.0f - alpha) * noiseFloor + alpha * rms;
    noiseFloor = fmaxf(noiseFloor, 0.00001f);
  }
  if ((uint32_t)(now - lastLogMs) >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    Serial.printf("DATA,rms=%.5f,peak=%.5f,crest=%.2f,noise=%.5f,threshold=%.5f,count=%lu\n",
                  rms, peak, crest, noiseFloor, threshold, (unsigned long)eventCount);
  }
}
