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
#include "soc/soc.h"

#include <atomic>
#include <array>
#include <cmath>

#define DISPLAY_I2C_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_BUFFER_SIZE ((uint32_t)(DISPLAY_WIDTH * DISPLAY_HEIGHT / 8))

#define BUTTON_TIME GPIO_NUM_32
#define BUTTON_WEIGHT GPIO_NUM_33

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
  static constexpr size_t BUFFER_SIZE = 256;
  using Buffer = std::array<int32_t, BUFFER_SIZE>;

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

  bool readSamples() override {
    if (mCount >= BUFFER_SIZE) return false;

    size_t freeTotal = BUFFER_SIZE - mCount.load();
    if (freeTotal == 0) return true;

    size_t firstSpan = std::min(freeTotal, BUFFER_SIZE - mHead);
    size_t bytesFirst = firstSpan * sizeof(int32_t);

    size_t gotBytes = 0;
    if (bytesFirst) {
      size_t chunk = 0;
      esp_err_t err = i2s_channel_read(
        mRxHandle,
        reinterpret_cast<char*>(&mBuffer[mHead]),
        bytesFirst,
        &chunk,
        10
      );
      if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGE("I2SInterfaceESP32Float", "I2S read error: %s", esp_err_to_name(err));
        return false;
      }
      gotBytes += chunk;
    }

    size_t gotSamples = gotBytes / sizeof(int32_t);
    mHead = (mHead + gotSamples) % BUFFER_SIZE;
    mCount += gotSamples;

    size_t stillFree = BUFFER_SIZE - mCount.load();
    if (stillFree > 0) {
      size_t secondSpan = std::min(stillFree, BUFFER_SIZE - mHead);
      size_t bytesSecond = secondSpan * sizeof(int32_t);
      if (bytesSecond) {
        size_t chunk2 = 0;
        esp_err_t err2 = i2s_channel_read(
          mRxHandle,
          reinterpret_cast<char*>(&mBuffer[mHead]),
          bytesSecond,
          &chunk2,
          10
        );
        if (err2 != ESP_OK && err2 != ESP_ERR_TIMEOUT) {
          ESP_LOGE("I2SInterfaceESP32Float", "I2S read error: %s", esp_err_to_name(err2));
          return false;
        }
        size_t got2 = chunk2 / sizeof(int32_t);
        mHead = (mHead + got2) % BUFFER_SIZE;
        mCount += got2;
      }
    }

    return true;
  }

  float getSample() override {
    if (mCount == 0) return 0.0f;
    int32_t raw = mBuffer[mTail];
    mTail = (mTail + 1) % BUFFER_SIZE;
    mCount--;

    int32_t i24 = raw >> 8;
    return static_cast<float>(i24) / 8388608.0f; // → [-1, 1)
  }

  size_t available() const override {
    return mCount.load();
  }

private:
  Buffer mBuffer;
  size_t mHead, mTail;
  i2s_chan_handle_t mRxHandle;
  std::atomic<size_t> mCount;
};

void vDisplayUpdateCallback(TimerHandle_t xTimer) {
  volatile bool* flag = static_cast<volatile bool*>(pvTimerGetTimerID(xTimer));
  if (flag) *flag = true;
}

volatile uint8_t gButtonEvent = 0;
volatile uint32_t gButtonIdx = 0;
void IRAM_ATTR buttonsISRHandler(void *arg) {
  gButtonEvent = 1;
  gButtonIdx = (uint32_t)arg;
}

void initializeButtons() {
  gpio_config_t ioConf = {};
  ioConf.intr_type = GPIO_INTR_NEGEDGE;
  ioConf.mode = GPIO_MODE_INPUT;
  ioConf.pin_bit_mask = (1ULL << BUTTON_TIME) | (1ULL << BUTTON_WEIGHT);
  ioConf.pull_up_en = GPIO_PULLUP_DISABLE;
  ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&ioConf);

  gpio_install_isr_service(0);

  gpio_isr_handler_add(BUTTON_TIME, buttonsISRHandler, (void*)BUTTON_TIME);
  gpio_isr_handler_add(BUTTON_WEIGHT, buttonsISRHandler, (void*)BUTTON_WEIGHT);
}

slm::SLMConfig handleButtons(slm::SLMConfig meterConfig, volatile uint32_t& buttonIdx) {
  if (buttonIdx == BUTTON_TIME) {
    buttonIdx = 0;
    switch (meterConfig.tW) {
      case slm::TimeWeighting::FAST:
        meterConfig.tW = slm::TimeWeighting::SLOW; break;
      case slm::TimeWeighting::SLOW:
        meterConfig.tW = slm::TimeWeighting::IMPULSE; break;
      case slm::TimeWeighting::IMPULSE:
        meterConfig.tW = slm::TimeWeighting::FAST; break;
    }
  }
  if (buttonIdx == BUTTON_WEIGHT) {
    buttonIdx = 0;
    switch (meterConfig.fW) {
      case slm::FrequencyWeighting::A:
        meterConfig.fW = slm::FrequencyWeighting::C; break;
      case slm::FrequencyWeighting::C:
        meterConfig.fW = slm::FrequencyWeighting::Z; break;
      case slm::FrequencyWeighting::Z:
        meterConfig.fW = slm::FrequencyWeighting::A; break;
    }
  }
  return meterConfig;
}

extern "C"
void app_main(void) {
  ESP_LOGI("SLM", "Starting Sound Level Meter...");

  initializeButtons();
  ESP_LOGI("SLM", "Buttons initialized...");

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
    calibData.frequencyWeighting = static_cast<uint32_t>(meterConfig.fW);
    calibData.timeWeighting = static_cast<uint32_t>(meterConfig.tW);
    mem_calib_write(&calibData);
  }
  meter.reset(meterConfig);

  i2c_config_t i2cConfig = {};
  i2cConfig.mode = I2C_MODE_MASTER;
  i2cConfig.sda_io_num = GPIO_NUM_22;
  i2cConfig.scl_io_num = GPIO_NUM_23;
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

  if (xTimerStart(displayTimer, 0) != pdPASS) {
    ESP_LOGE("SLM", "Failed to start display update timer");
    while(1);
  }

  ESP_LOGI("SLM", "System initialized...");

  char pSPLLabel[16];
  char pLeqLabel[32];
  char pTimeLabel[8];
  memset(pSPLLabel, 0, sizeof(pSPLLabel));
  memset(pLeqLabel, 0, sizeof(pLeqLabel));
  memset(pTimeLabel, 0, sizeof(pTimeLabel));

  while (true) {
    mic.readSamples();
    size_t availableSamples = mic.available();
    for (size_t i = 0; i < availableSamples; ++i) {
      float sample = mic.getSample();
      slm::MeterResults results = meter.process(sample);
      if (gButtonEvent) {
        meterConfig = handleButtons(meterConfig, gButtonIdx);
        meter.reset(meterConfig);
        gButtonEvent = 0;
        displayUpdateFlag = true;
      }
      if (displayUpdateFlag) {
        gfx.fillScreen(0x00);
        gfx.drawRect(1, 1, DISPLAY_WIDTH - 2, DISPLAY_HEIGHT - 2, 0xFF);

        char freqWLabel = [&meterConfig]() -> char {
          switch (meterConfig.fW) {
            case slm::FrequencyWeighting::A: return 'A';
            case slm::FrequencyWeighting::C: return 'C';
            case slm::FrequencyWeighting::Z: return 'Z';
            default: return '\0';
          }
        }();

        switch (meterConfig.tW) {
          case slm::TimeWeighting::FAST: snprintf(pTimeLabel, sizeof(pTimeLabel), "FAST"); break;
          case slm::TimeWeighting::SLOW: snprintf(pTimeLabel, sizeof(pTimeLabel), "SLOW"); break;
          case slm::TimeWeighting::IMPULSE: snprintf(pTimeLabel, sizeof(pTimeLabel), "IMPL"); break;
        }

        snprintf(pSPLLabel, sizeof(pSPLLabel), "%-4.1f", results.spl);
        snprintf(pLeqLabel, sizeof(pLeqLabel), "L%ceq: %-4.1f dB", freqWLabel, results.leq);

        gfx.drawString(30, 10, pSPLLabel, 0xFF, &font16x32);
        gfx.drawString(100, 20, "dB\0", 0xFF, &font8x16);
        gfx.drawString(4, 50, pLeqLabel, 0xFF, &font5x7);
        gfx.drawString(100, 50, pTimeLabel, 0xFF, &font5x7);

        display.writeData(gfx.buffer(), gfx.bufferSize());
        displayUpdateFlag = false;
      }
    }
  }
}

