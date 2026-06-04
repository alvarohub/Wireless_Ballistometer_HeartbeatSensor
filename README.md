# Wireless Ballistometer Heartbeat Sensor (M5Stack AtomS3R)

Real-time heartbeat estimation from micro-vibrations (BCG/SCG-like signal) using the built-in IMU on an M5Stack AtomS3R.

The firmware samples accelerometer data, runs lightweight filtering + peak detection on-device, and streams data to a browser UI over WebSocket.

## Features

- ESP32-S3 + M5Unified firmware (PlatformIO)
- Built-in IMU sampling at configurable rates (10-500 Hz)
- Lock-free single-producer/single-consumer ring buffer
- On-device signal processing pipeline:
  - acceleration magnitude
  - band-pass style IIR filtering (HP + LP)
  - adaptive thresholding from sliding window statistics
  - Schmitt-trigger peak detection + refractory period
- Real-time BPM and HRV estimation
- Embedded web UI (served directly from firmware, no SPIFFS)
- CSV export from browser
- Works in standalone AP mode (no external router required)

## Hardware

- M5Stack AtomS3R (ESP32-S3 with onboard IMU)
- USB-C cable for flashing/power

## Firmware Stack

- Framework: Arduino (via PlatformIO)
- Board: `m5stack-atoms3`
- Platform: `espressif32`

Main dependencies:

- `m5stack/M5Unified`
- `links2004/WebSockets`
- `bblanchon/ArduinoJson`

See [platformio.ini](platformio.ini) for exact versions.

## Quick Start

### 1) Build and flash

```bash
pio run -t upload
```

### 2) Open serial monitor

```bash
pio device monitor -b 115200
```

### 3) Connect to device Wi-Fi AP

After boot, the device creates:

- SSID: `HeartBeat_Sensor`
- Password: `heartbeat`

### 4) Open web UI

In your browser:

- HTTP UI: `http://192.168.4.1`
- WebSocket endpoint: `ws://192.168.4.1:81`

## Runtime Controls

From the web UI:

- Start/Stop sampling
- Set sample rate (`10-500 Hz`)
- Set recording duration (`0 = continuous`)
- Set filter cutoffs (HP and LP)
- Select plotted signal (X, Y, Z, magnitude, server-filtered)
- Toggle raw/filtered visualization
- Export CSV

From the hardware button (`BtnA`):

- Toggle sampling start/stop

## Data Flow Overview

1. High-priority FreeRTOS task on Core 0 samples IMU data.
2. Samples are pushed to a lock-free ring buffer.
3. Main loop on Core 1 drains the buffer.
4. Signal processor computes filtered signal + detects peaks.
5. JSON batches are broadcast over WebSocket.
6. Browser UI renders live traces and displays BPM/HRV.

## WebSocket Protocol (JSON)

### Commands sent by UI

- `{"cmd":"start"}`
- `{"cmd":"stop"}`
- `{"cmd":"set_rate","value":100}`
- `{"cmd":"set_duration","value":0}`
- `{"cmd":"set_filters","hp":0.5,"lp":10.0}`

### Messages sent by firmware

- `config`: current sample rate, duration, sampling state, IMU status
- `data`: batched sample arrays, BPM, HRV, and detected peaks

## Project Structure

```text
include/
  ring_buffer.h         # Lock-free SPSC ring buffer + AccelSample struct
  signal_processing.h   # Filtering, adaptive thresholds, Schmitt peak detector
  web_content.h         # Embedded HTML/CSS/JS web app (served by ESP32)
src/
  main.cpp              # Firmware entry point, Wi-Fi AP, HTTP + WebSocket server
platformio.ini          # PlatformIO environment and dependencies
```

## Notes and Limitations

- Heartbeat from motion signals is sensitive to placement, motion artifacts, and environment.
- Best results are typically obtained with stable contact and minimal body movement.
- This repository is intended as an experimental prototype and research/demo baseline.

## Credits

Created by Alvaro Cassinelli, 2025.

## License

This project is licensed under the MIT License.
See the [LICENSE](LICENSE) file for details.
