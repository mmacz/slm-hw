# Sound Level Meter for ESP32 (INMP441, IEC 61672-inspired)

This project is a hobbyist sound level meter based on the ESP32 and the INMP441 MEMS microphone, inspired by the requirements of the IEC 61672 standard (Sound Level Meters).

## Project Overview

- **Modular architecture**: Each functional block (filters, weightings, calculations) is implemented as a separate component.
- **IEC 61672-inspired**: Implements A and C frequency weightings, time weightings (Fast/Slow), and RMS algorithms according to the standard's guidelines.
- **Platform**: ESP32 with the ESP-IDF framework.
- **Microphone**: INMP441 (I2S MEMS).

## Limitations

- **Not formally IEC 61672 compliant**: The INMP441 is not a measurement-grade microphone and does not meet the accuracy, linearity, or calibration requirements of the standard.
- **For hobby and indicative use only**: Measurement results are approximate and should not be used for professional, inspection, or legal purposes.
- **Frequency range**: Limited by the microphone's characteristics and the chosen sampling rate.

## Project Structure

- `components/` – source code for individual blocks (filters, weightings, interfaces)
- `src/` – main application
- `platformio.ini` – PlatformIO configuration

## License

Open-source project for non-commercial use. Any professional or commercial use is at your own risk.

---

**Note:**  
This project is intended for educational and hobby purposes only. If you require IEC 61672 compliant measurements, use certified measurement equipment.
