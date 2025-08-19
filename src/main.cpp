#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"

#include "FilterCoeffs.h"
#include "MemCalib.h"
#include "SoundLevelMeter.h"
#include "SSD1306.h"
#include "GFX.h"
#include "Fonts.h"
#include "INMP441.h"

#include <atomic>
#include <array>

#define DISPLAY_I2C_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_BUFFER_SIZE ((uint32_t)(DISPLAY_WIDTH * DISPLAY_HEIGHT / 8))

volatile uint8_t gDisplayBuffer[DISPLAY_BUFFER_SIZE] = {0};

class I2CDisplayBus : public Display::IDisplayBus {
public:
  I2CDisplayBus(i2c_port_t port, const i2c_config_t& config, uint32_t timeout = 1000)
      : mPort(port), mTimeout(timeout) {
    i2c_param_config(port, &config);
    i2c_driver_install(port, config.mode, 0, 0, 1);
  }

  bool write(uint8_t address, const uint8_t* data, size_t len, bool isCommand) override {
    if (!data || len == 0) return false;

    const int max_attempts = 10;
    int attempts = 0;

    do {
      uint8_t control = isCommand ? 0x00 : 0x40;
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
      i2c_master_write(cmd, &control, 1, true);
      i2c_master_write(cmd, data, len, true);
      i2c_master_stop(cmd);

      esp_err_t ret = i2c_master_cmd_begin(mPort, cmd, pdMS_TO_TICKS(mTimeout));
      i2c_cmd_link_delete(cmd);
      if (ret == ESP_OK) {
        return true;
      }
      else {
        ESP_LOGW("I2CDisplayBus", "I2C write error %d, attempt %d/%d", ret, attempts + 1, max_attempts);
      }
    } while(++attempts < max_attempts);

    return false;
  }

private:
  i2c_port_t mPort;
  uint32_t mTimeout;
};

class I2SInterfaceESP32Float : public Microphone::I2SInterface<float> {
public:
  static constexpr size_t BUFFER_SIZE = 64;
  using BufferType = std::array<float, BUFFER_SIZE>;

  I2SInterfaceESP32Float()
    : mHead(0), mTail(0), mCount(0) {}

  bool initialize(const Microphone::INMP441Config& config) override {
    mRxHandle = nullptr;

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 8,
        .dma_frame_num = 256,
        .auto_clear = true,
        .auto_clear_before_cb = false,
        .intr_priority = 1,
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config.sampleRate),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_24BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 32,
            .ws_pol = false,
            .bit_shift = true,
            .msb_right = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_18,
            .ws = GPIO_NUM_19,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_21,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        }
    };

    bool ok = true;
    ok &= i2s_new_channel(&chan_cfg, NULL, &mRxHandle) == ESP_OK;
    ok &= i2s_channel_init_std_mode(mRxHandle, &std_cfg) == ESP_OK;
    ok &= i2s_channel_enable(mRxHandle) == ESP_OK;
    return ok;
  }

  bool readSamples() override{
    if (mCount >= BUFFER_SIZE) return false;
    int32_t i32Samples[64] {0};
    size_t bytesRead = 0;
    while (bytesRead < sizeof(i32Samples)) {
      esp_err_t err = i2s_channel_read(mRxHandle, (char*)&i32Samples + bytesRead, sizeof(i32Samples) - bytesRead, &bytesRead, 10);
      if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGE("I2SInterfaceESP32Float", "I2S read error: %s", esp_err_to_name(err));
        return false;
      }
    }
    size_t gotSamples = bytesRead / sizeof(int32_t);
    for (size_t i = 0; i < gotSamples; ++i) {
      int32_t i24Sample = (i32Samples[i] >> 8);
      float value = static_cast<float>(i24Sample) / 8388608.0f;
      mBuffer[mHead] = value;
      mHead = (mHead + 1) % BUFFER_SIZE;
      mCount++;
    }
    return true;
  }

  float getSample() override {
    if (mCount == 0) return 0.0f;
    float value = mBuffer[mTail];
    mTail = (mTail + 1) % BUFFER_SIZE;
    mCount--;
    return value;
  }

  size_t available() const override {
    return mCount.load();
  }

private:
  BufferType mBuffer;
  size_t mHead, mTail;
  i2s_chan_handle_t mRxHandle;
  std::atomic<size_t> mCount;
};

void vDisplayUpdateCallback(TimerHandle_t xTimer) {
  volatile bool* flag = static_cast<volatile bool*>(pvTimerGetTimerID(xTimer));
  if (flag) *flag = true;
}

extern "C"
void app_main(void) {
  ESP_LOGI("SLM", "Starting Sound Level Meter...");

  I2SInterfaceESP32Float i2sF;
  Microphone::INMP441Config micCfg(SAMPLE_RATE, false, true);
  Microphone::INMP441<float> mic(micCfg, i2sF);
  if (!mic.initialize()) {
    ESP_LOGE("SLM", "Failed to initialize INMP441 microphone");
    while(1);
  }

  slm::SLMConfig meterConfig;
  slm::SoundLevelMeter meter(meterConfig);

  mem_calib_init();

  mem_calib_data_t calibData;
  int32_t status = mem_calib_read(&calibData);

  if (status == MEM_CALIB_ERR_INVALID_DATA) {
    calibData.calibrationFactor = DEFAULT_INMP441_CALIBRATION_VALUE;
    calibData.frequencyWeighting = static_cast<uint32_t>(meterConfig.fW);
    calibData.timeWeighting = static_cast<uint32_t>(meterConfig.tW);
    mem_calib_write(&calibData);
  }
  meter.reset(meterConfig);
  meter.calibrate(calibData.calibrationFactor);

  i2c_config_t i2cConfig = {};
  i2cConfig.mode = I2C_MODE_MASTER;
  i2cConfig.sda_io_num = GPIO_NUM_23;
  i2cConfig.scl_io_num = GPIO_NUM_22;
  i2cConfig.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2cConfig.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2cConfig.master.clk_speed = 400000;
  I2CDisplayBus displayBus(I2C_NUM_0, i2cConfig);

  Display::SSD1306Config displayConfig(DISPLAY_I2C_ADDRESS, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  Display::SSD1306 display(displayConfig, displayBus);
  display.initialize();

  gfx::GFX gfx(DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_BUFFER_SIZE, (uint8_t*)gDisplayBuffer);
  display.writeData(gfx.buffer(), gfx.bufferSize());

  static volatile bool displayUpdateFlag = false;
  TimerHandle_t displayTimer = xTimerCreate(
    "DisplayUpdateTimer",
    pdMS_TO_TICKS(500),
    pdTRUE,
    (void*)&displayUpdateFlag,
    vDisplayUpdateCallback
  );

  char pPeakTextBuffer[16];
  char pRawTextBuffer[20];
  char pLeqTextBuffer[16];
  char pSPLTextBuffer[16];
  memset(pPeakTextBuffer, 0, sizeof(pPeakTextBuffer));
  memset(pRawTextBuffer, 0, sizeof(pRawTextBuffer));
  memset(pLeqTextBuffer, 0, sizeof(pLeqTextBuffer));
  memset(pSPLTextBuffer, 0, sizeof(pSPLTextBuffer));

  if (xTimerStart(displayTimer, 0) != pdPASS) {
    ESP_LOGE("SLM", "Failed to start display update timer");
    while(1);
  }

  ESP_LOGI("SLM", "System initialized...");

  while (true) {
    mic.readSamples();
    size_t availableSamples = mic.available();
    for (size_t i = 0; i < availableSamples; ++i) {
      float sample = mic.getSample();
      slm::MeterResults results = meter.process(sample);
      if (displayUpdateFlag) {
        gfx.fillScreen(0x00);
        gfx.drawRect(1, 1, DISPLAY_WIDTH - 2, DISPLAY_HEIGHT - 2, 0xFF);
        // snprintf(pPeakTextBuffer, 16, "peak: %6.1f dB", results.peak);
        snprintf(pRawTextBuffer, 20, "raw: %1.6f", sample);
        snprintf(pLeqTextBuffer, 16, "leq: %6.1f dB", results.leq);
        snprintf(pSPLTextBuffer, 16, "spl: %6.1f dB", results.spl);
        // gfx.drawString(4, 3, pPeakTextBuffer, 0xFF, font5x7);
        gfx.drawString(4, 3, pRawTextBuffer, 0xFF, font5x7);
        gfx.drawString(4, 11, pLeqTextBuffer, 0xFF, font5x7);
        gfx.drawString(4, 19, pSPLTextBuffer, 0xFF, font5x7);
        switch (meterConfig.tW) {
          case slm::TimeWeighting::FAST:
            gfx.drawString(100, 11, "FAST", 0xFF, font5x7);
            break;
          case slm::TimeWeighting::SLOW:
            gfx.drawString(100, 11, "SLOW", 0xFF, font5x7);
            break;
          case slm::TimeWeighting::IMPULSE:
            gfx.drawString(100, 11, "IMPL", 0xFF, font5x7);
            break;
        }
        switch (meterConfig.fW) {
          case slm::FrequencyWeighting::A:
            gfx.drawChar(110, 19, 'A', 0xFF, font5x7);
            break;
          case slm::FrequencyWeighting::C:
            gfx.drawChar(110, 19, 'C', 0xFF, font5x7);
            break;
          case slm::FrequencyWeighting::Z:
            gfx.drawChar(110, 19, 'Z', 0xFF, font5x7);
            break;
        }
        display.writeData(gfx.buffer(), gfx.bufferSize());
        displayUpdateFlag = false;
      }
    }
  }
}

