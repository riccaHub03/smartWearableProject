# Smart Wearable — Real-Time Vital Signs Monitor on STM32U5

Firmware for a wearable vital-signs monitor based on an **STM32U575** microcontroller. The device acquires **PPG, skin temperature and motion** at 100 Hz, runs a **causal DSP pipeline on the MCU** to estimate **heart rate, SpO₂ and perfusion index** in real time, streams the results over **Bluetooth Low Energy** and logs the raw data to **NAND flash** for later download over **USB**.

The companion hardware is a custom **2-layer PCB designed in Altium Designer**.

<!-- 👉 Add photos here: Altium 3D view of the PCB, the smartphone app, a plot of the filtered PPG with detected beats -->
<!-- ![PCB](docs/pcb_3d.png) ![PPG](docs/ppg_beats.png) ![App](docs/app.png) -->

**Tech stack:** C · STM32U575 (Cortex-M33) · STM32 HAL · STM32CubeMX / STM32CubeIDE · CMake · I2C · SPI · UART · USB CDC · BLE · Altium Designer

---

## Hardware

| Component | Part | Interface | Role |
|---|---|---|---|
| MCU | **STM32U575RIT** (Arm Cortex-M33, ultra-low-power) | — | acquisition, DSP, communication |
| PPG sensor | **MAX30101** (red + infrared LEDs, 18-bit ADC) | I2C | heart rate, SpO₂, perfusion index |
| Temperature | **MAX30205** (clinical-grade) | I2C | skin temperature |
| IMU | **LSM6DSO16IS** (3-axis accelerometer + gyroscope) | I2C | motion |
| BLE module | **Microchip RN4871** | UART | wireless streaming |
| Storage | **Micron MT29F4G01** 4 Gb SPI NAND flash | SPI | data logging |
| USB | USB OTG FS, Virtual COM Port (CDC) | USB | data download |

---

## Firmware architecture

The application is a **state machine** driven by a single user button:

```
 IDLE ──button──► ACQUISITION ──button──► IDLE
   │                  │
   │ USB plugged      │ every 10 ms (TIM2 interrupt, 100 Hz)
   ▼                  ▼
 USB_CONNECTED     read PPG + IMU (temperature every 100 ms)
   │                 → DSP pipeline (HR, SpO₂, PI, quality)
   │ button          → BLE packet with vital signs
   ▼                 → raw sample written to NAND flash
 DOWNLOAD ──► flash content sent over USB VCP
```

- **Deterministic sampling:** TIM2 generates a 100 Hz tick; acquisition and processing run once per tick.
- **Register-level sensor drivers** for MAX30101, MAX30205 and LSM6DSO16IS (configuration of FIFO, LED currents, ODR, full scale, performance modes, WHO_AM_I checks).
- **BLE driver** for the RN4871 (command/data mode, low-power and dormant modes, packetised transmission). Each packet carries smoothed HR, SpO₂, perfusion index ×10 and a signal-quality flag.
- **NAND logging** with bad-block management: 29 bytes per sample (timestamp, PPG, temperature, accelerometer, gyroscope), 141 samples per 4 KB page.
- **Status LEDs** for sensor initialisation, acquisition and USB states.

---

## Signal processing (`Core/Src/bio_filt2.c`)

A sample-by-sample, **causal** C port of the offline Python analysis (`data_analyzer3.py`), designed to run in real time on the MCU at 100 Hz:

1. **DC component** — 2 s moving average of the red and IR channels
2. **Baseline wander removal** — subtraction of a 1.5 s moving average
3. **Dynamic artefact clipping** — signal limited to ±2.5 × the rolling standard deviation (2 s window)
4. **Band-pass filter 0.5–4 Hz** — Butterworth IIR implemented as a cascade of 4 biquad sections
5. **AC component and perfusion index** — RMS of the filtered signal converted to peak-to-peak; PI = AC/DC on the IR channel
6. **Signal-quality assessment** — rejects samples when the sensor is off the skin (DC proximity threshold) or the PI is out of range, with a 2 s hold-off after each artefact
7. **Beat detection (Pan-Tompkins-inspired)** — 5-point derivative, squaring, 150 ms moving-window integration, adaptive threshold and 450 ms refractory period
8. **Heart rate** — from the inter-beat interval (valid range 40–200 bpm), smoothed with an **exponential moving average** (α = 0.2) like a smartwatch display
9. **SpO₂** — ratio-of-ratios method, `SpO₂ = 110 − 25·R`, clamped to 80–100 % and smoothed with an EMA

---

## Validation

Heart rate and SpO₂ were compared with commercial **Garmin** smartwatches worn at the same time, with small deviations.
<!-- Add a table: mean absolute error (bpm), SpO₂ difference, test conditions (rest / walking) -->

---

## Repository structure

```
Core/
  Src/main.c              state machine, peripheral init, 100 Hz acquisition loop
  Src/bio_driver.c        MAX30101 (PPG) and MAX30205 (temperature) drivers
  Src/bio_filt2.c         real-time DSP: filtering, beat detection, HR, SpO2, PI
  Src/imu_driver.c        LSM6DSO16IS accelerometer / gyroscope driver
  Src/bluetooth.c         RN4871 BLE driver
  Src/SPI_NAND.c          MT29F4G01 SPI NAND driver
  Src/Memory_operations.c packet format, bad-block handling, read/write
  Src/led_driver.c        status LEDs
Drivers/                  STM32U5 HAL, CMSIS, MAX30101 / MAX30205 HAL helpers
USB_Device/               USB CDC (Virtual COM Port)
MainBoard_IMU_Logger.ioc  STM32CubeMX configuration
CMakeLists.txt            CMake build (arm-none-eabi-gcc or starm-clang)
```

## Build and flash

**STM32CubeIDE:** import the folder as an existing project, build and run on the board.

**CMake + VS Code:**
```bash
cmake --preset Debug
cmake --build --preset Debug
```
Flash the generated `.elf` with STM32CubeProgrammer or an ST-LINK.

## Usage

1. Power the board: the red LED turns off when all sensors are initialised (it blinks if a sensor is not found).
2. Press the user button to **start / stop acquisition**: vital signs are streamed over BLE and raw data is logged to flash.
3. Plug in USB and press the button to **download** the logged data through the Virtual COM Port.

## Acknowledgements

Developed for the *Smart Wearables Design and Prototyping* course at Politecnico di Milano, starting from the course's base firmware template (NAND memory and USB download).

## Author

**Riccardo Frasson** — MSc Biomedical Engineering, Politecnico di Milano
[github.com/riccaHub03](https://github.com/riccaHub03) · riccardo1.frasson@mail.polimi.it
