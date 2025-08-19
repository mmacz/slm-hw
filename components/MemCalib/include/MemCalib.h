#pragma once

#include "FilterCoeffs.h"

#if defined __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct mem_calib_data_s {
  uint32_t timeWeighting = 0;
  uint32_t frequencyWeighting = 0;
} mem_calib_data_t;

enum mem_calib_status_e {
  MEM_CALIB_OK,
  MEM_CALIB_ERR_INIT,
  MEM_CALIB_ERR_CANNOT_OPEN,
  MEM_CALIB_ERR_READ,
  MEM_CALIB_ERR_WRITE,
  MEM_CALIB_ERR_INVALID_DATA
};

int32_t mem_calib_init(void);
int32_t mem_calib_read(mem_calib_data_t *data);
int32_t mem_calib_write(const mem_calib_data_t *data);

#if defined __cplusplus
} // extern "C"
#endif
