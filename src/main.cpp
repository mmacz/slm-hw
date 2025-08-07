#include "FilterCoeffs.h"
#include "MemCalib.h"
#include "SoundLevelMeter.h"

extern "C"
void app_main(void) {
  slm::SLMConfig meterConfig;
  slm::SoundLevelMeter meter(meterConfig);

  mem_calib_init();

  mem_calib_data_t calibData;
  int32_t status = mem_calib_read(&calibData);

  if (status != MEM_CALIB_ERR_INVALID_DATA) {
    calibData.calibrationFactor = DEFAULT_INMP441_CALIBRATION_VALUE;
    mem_calib_write(&calibData);
  }

  meter.calibrate(calibData.calibrationFactor);

  while (true) {
    float sample = 0.0f; // Replace with actual audio sample acquisition logic
    slm::MeterResults results = meter.process(sample);
    (void)results;

    // Use results.peak, results.leq, results.spl as needed
    // For example, print or log the results
  }
}

