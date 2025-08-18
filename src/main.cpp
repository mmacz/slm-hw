#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
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
  static constexpr size_t BUFFER_SIZE = 32;
  using BufferType = std::array<float, BUFFER_SIZE>;

  I2SInterfaceESP32Float()
    : mHead(0), mTail(0), mCount(0) {}

  bool initialize(const Microphone::INMP441Config& config) override {
    mRxHandle = nullptr;

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 4,
        .dma_frame_num = 64,
        .auto_clear = true,
        .auto_clear_before_cb = true,
        .intr_priority = 1,
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_24BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 32,
            .ws_pol = false,
            .bit_shift = false,
            .msb_right = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_18,
            .ws = GPIO_NUM_19,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_21,
        }
    };

    bool ok = true;
    ok &= i2s_new_channel(&chan_cfg, &mRxHandle, NULL) == ESP_OK;
    ok &= i2s_channel_init_std_mode(mRxHandle, &std_cfg) == ESP_OK;
    return ok;
  }

  bool writeSamples() override{
    if (mCount >= BUFFER_SIZE) return false;
    uint8_t raw[3];
    size_t bytes_read = 0;
    if (i2s_channel_read(mRxHandle, raw, 3, &bytes_read, 0) != ESP_OK || bytes_read != 3)
      return false;
    int32_t sample = (raw[0] << 8) | (raw[1] << 16) | (raw[2] << 24);
    sample >>= 8;
    float value = static_cast<float>(sample) / 8388608.0f;
    mBuffer[mHead] = value;
    mHead = (mHead + 1) % BUFFER_SIZE;
    mCount++;
    return true;
  }

  float readSample() override {
    if (mCount == 0) return 0.0f;
    float value = mBuffer[mTail];
    mTail = (mTail + 1) % BUFFER_SIZE;
    mCount--;
    return value;
  }

private:
  BufferType mBuffer;
  size_t mHead, mTail;
  i2s_chan_handle_t mRxHandle;
  std::atomic<size_t> mCount;
};

extern "C"
void app_main(void) {
  ESP_LOGI("SLM", "Starting Sound Level Meter...");

  I2SInterfaceESP32Float i2sF;
  Microphone::INMP441Config micCfg(24000, false, true);
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

  while (true) {
  }
}

