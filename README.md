# Final Project of STM32 Course

An embedded project implemented on **STM32 Nucleo-F411RE**.  
The system combines **environmental sensing** (temperature & pressure), **audio signal analysis** using FFT, and **real-time visualization** on an OLED display.

---

## ✨ Features

- **BMP280 sensor (I2C)**:  
  - Temperature measurement (°C)  
  - Pressure measurement (hPa)  

- **KY-037 microphone module (ADC)**:  
  - Real-time audio sampling from environment  
  - FFT analysis to extract frequency components  
  - Spectrum visualization on OLED  

- **OLED Display (I2C)**:  
  - Display of temperature & pressure values  
  - Live spectrum analyzer visualization of sound  

---

## 📐 System Architecture

- **MCU:** STM32F411RE (Nucleo board)  
- **Sensors/Inputs:**
  - BMP280 (I2C bus)
  - KY-037 microphone (ADC input)
- **Outputs:**
  - OLED display (I2C)
- **Processing:**
  - Data acquisition via HAL (I2C + ADC DMA)
  - FFT (Fast Fourier Transform) using CMSIS-DSP library
  - Real-time plotting of frequency spectrum

---

## 🧩 Software Design

- **Drivers:**
  - `bmp280.c` – sensor driver (temperature, pressure)
  - `oled.c` – OLED abstraction layer
  - `mic.c` – microphone input (ADC)
  - `fft.c` – frequency analysis using CMSIS-DSP

- **Application:**
  - Periodic acquisition of temperature & pressure
  - Continuous audio sampling into buffer (via DMA)
  - FFT processing on audio frames
  - OLED visualiz
