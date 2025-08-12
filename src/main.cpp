#include <memory.h>

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "FilterCoeffs.h"
#include "MemCalib.h"
#include "SoundLevelMeter.h"
#include "SSD1306.h"

#define DISPLAY_I2C_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_BUFFER_SIZE ((uint32_t)(DISPLAY_WIDTH * DISPLAY_HEIGHT / 8))

uint8_t gDisplayBuffer[DISPLAY_BUFFER_SIZE];

class I2CDisplayBus : public Display::IDisplayBus {
public:
  I2CDisplayBus(i2c_port_t port, const i2c_config_t& config, uint32_t timeout = 1000)
      : mPort(port), mTimeout(timeout) {
    i2c_param_config(port, &config);
    i2c_driver_install(port, config.mode, 0, 0, 1);
  }

  bool write(uint8_t address, const uint8_t* data, size_t len, bool isCommand) override {
    if (!data || len == 0) return false;

    uint8_t control = isCommand ? 0x00 : 0x40;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, &control, 1, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(mPort, cmd, pdMS_TO_TICKS(mTimeout));
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK;
  }

private:
  i2c_port_t mPort;
  uint32_t mTimeout;
};

extern "C"
void app_main(void) {
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

  i2c_config_t i2cConfig;
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

  memset((void*)gDisplayBuffer, 0xFF, sizeof(gDisplayBuffer));
  display.writeData((uint8_t*)gDisplayBuffer, sizeof(gDisplayBuffer));

  while (true) {
  }
}

