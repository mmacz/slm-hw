#include "MemCalib.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstdint>

#if defined __cplusplus
extern "C" {
#endif

#define CALIBRATION_HEADER_GUARD ((uint32_t)0x43414C42) // "CALB"
#define CALIBRATION_FOOTER_GUARD ((uint32_t)0x43414C46) // "CALF"

int32_t mem_calib_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  return ret;
}

int32_t mem_calib_read(mem_calib_data_t *data) {
  nvs_handle_t handle;
  if (nvs_open("calibration", NVS_READONLY, &handle) != ESP_OK) {
    return MEM_CALIB_ERR_CANNOT_OPEN;
  }
  uint32_t headerGuard = 0;
  if (nvs_get_u32(handle, "headerGuard", &headerGuard) != ESP_OK) {
    nvs_close(handle);
    return MEM_CALIB_ERR_READ;
  }

  size_t data_size = sizeof(mem_calib_data_t);
  if (nvs_get_blob(handle, "calibration_data", data, &data_size) != ESP_OK) {
    nvs_close(handle);
    return MEM_CALIB_ERR_READ;
  }

  uint32_t footerGuard = 0;
  if (nvs_get_u32(handle, "footerGuard", &footerGuard) != ESP_OK) {
    nvs_close(handle);
    return MEM_CALIB_ERR_READ;
  }

  if (headerGuard != CALIBRATION_HEADER_GUARD || footerGuard != CALIBRATION_FOOTER_GUARD) {
    nvs_close(handle);
    return MEM_CALIB_ERR_INVALID_DATA;
  }

  nvs_close(handle);
  return MEM_CALIB_OK;
}

int32_t mem_calib_write(const mem_calib_data_t* data) {
  nvs_handle_t handle;

  if (nvs_open("calibration", NVS_READWRITE, &handle) != ESP_OK) {
    return MEM_CALIB_ERR_CANNOT_OPEN;
  }
  if (nvs_set_u32(handle, "headerGuard", CALIBRATION_HEADER_GUARD) != ESP_OK) {
    nvs_close(handle);
    return MEM_CALIB_ERR_WRITE;
  }
  if (nvs_set_blob(handle, "calibration_data", data, sizeof(mem_calib_data_t)) != ESP_OK) {
    nvs_close(handle);
    return MEM_CALIB_ERR_WRITE;
  }
  if (nvs_set_u32(handle, "footerGuard", CALIBRATION_FOOTER_GUARD) != ESP_OK) {
    nvs_close(handle);
    return MEM_CALIB_ERR_WRITE;
  }

  nvs_close(handle);
  return MEM_CALIB_OK;
}


#if defined __cplusplus
} // extern "C"
#endif
