# TAIGA System Architecture

## Overview

TAIGA uses a distributed architecture with multiple types of nodes, each running specialized firmware/software:

## Camera Nodes (8x)

Each camera node contains two MCUs:

### ESP32-S3 (Main Controller)
- **Firmware**: `firmware/taiga-cam/` (Rust + Embassy)
- **Responsibilities**:
  - Camera capture and image processing
  - AI wildlife classification (TinyYOLO)
  - PIR and mmWave detection sensors
  - 900MHz mesh networking
  - Meshtastic communication
  - GPS and IMU orientation tracking
  - Environmental sensor reading
  - Power management and sleep modes

### RP2040 (Unified Firmware)
- **Firmware**: `src/firmware/taiga-rp2040/` (Rust + Embassy-RP)
- **Variants**: Feature-based compilation for different roles
- **Camera Node** (`--features camera-node`, default):
  - MPPT solar charge controller (20W scale)
  - 3-sensor ultrasonic wind measurement (Carl47)
  - ESP32-S3 watchdog and hard reset capability
  - System health monitoring and flash logging
  - Battery management for camera power systems
- **Base Station** (`--features base-station`):
  - MPPT solar charge controller (100W+ scale)  
  - 6-sensor OpenWind professional weather station
  - Rain gauge interrupt processing
  - Raspberry Pi health monitoring (graceful approach)
  - Enhanced environmental sensors and weather prediction

## Base Stations (2x)

Each base station contains two processors:

### Raspberry Pi Zero 2 W (Main Computer)
- **Software**: `software/base-station/` (Rust + Tokio + Axum)
- **Responsibilities**:
  - 900MHz mesh network receiver
  - SQLite database for image metadata
  - Web interface and REST API
  - Image storage and management
  - Meshtastic command interface  
  - Network status monitoring
  - External drive management
  - WiFi access point (on-demand)

### RP2040 (Base Station Variant)
- **Firmware**: `src/firmware/taiga-rp2040/` compiled with `--features base-station`
- **Responsibilities**:
  - **MPPT solar charge controller** (higher power than camera nodes)
  - **Base station power management** and battery monitoring
  - OpenWind 6-sensor ultrasonic wind measurement
  - Rain gauge interrupt counting and processing
  - Environmental sensors (BME280, VEML6070, RTC)
  - Weather prediction algorithms
  - Precision timing for ultrasonics
  - Data logging redundancy
  - **Pi health monitoring** (different approach than ESP32-S3 reset)
  - I2C/SPI communication with Pi

## Software Architecture

```
Camera Nodes:
┌─────────────────┐    ┌──────────────────┐
│    ESP32-S3     │◄──►│     RP2040       │
│   (taiga-cam)   │    │ (taiga-rp2040    │
│                 │    │  camera-node)    │
│ • Camera/AI     │    │ • MPPT Control   │
│ • Mesh Network  │    │ • 3-Wind Sensors │
│ • Meshtastic    │    │ • ESP32 Watchdog │
│ • Detection     │    │ • Health Monitor │
└─────────────────┘    └──────────────────┘

Base Stations:
┌─────────────────┐    ┌──────────────────┐
│  Raspberry Pi   │◄──►│     RP2040       │
│ (base-station)  │    │ (taiga-rp2040    │
│                 │    │  base-station)   │
│ • Web Interface │    │ • MPPT Controller│
│ • Image Storage │    │ • 6-Wind OpenWind│
│ • 900MHz RX     │    │ • Rain Gauge     │
│ • Database      │    │ • Weather Predict│
│ • API Server    │    │ • Pi Monitor     │
│ • WiFi AP       │    │ • Env. Sensors   │
└─────────────────┘    └──────────────────┘
```

## Communication Flows

### Camera Node Internal
- **I2C**: ESP32-S3 ↔ RP2040 (sensor data, status)
- **GPIO**: Heartbeat, reset control, alerts

### Base Station Internal  
- **I2C/SPI**: Pi ↔ RP2040 (weather data, commands)
- **GPIO**: Status LEDs, control signals

### Network Communications
- **900MHz Mesh**: Camera nodes ↔ Base stations (images)
- **Meshtastic**: All nodes ↔ User (alerts, commands)
- **WiFi**: Base station ↔ User (web interface, API)

## Data Flow

1. **Detection**: Camera PIR/mmWave → ESP32-S3 
2. **Capture**: ESP32-S3 → Camera + AI processing
3. **Environment**: RP2040 → Wind/weather data → ESP32-S3
4. **Metadata**: ESP32-S3 → Embed all data in JPEG EXIF
5. **Transmission**: ESP32-S3 → 900MHz mesh → Base station
6. **Storage**: Base Pi → SQLite + file system
7. **Interface**: User → Web/API → Base Pi → Data

## Redundancy and Reliability

### Camera Nodes
- **Dual MCU**: ESP32-S3 main + RP2040 supervisor
- **Watchdog**: RP2040 monitors and resets ESP32-S3
- **Data Logging**: RP2040 maintains 30-day health log
- **Power**: MPPT + oversized battery + solar

### Base Stations  
- **Dual Processors**: Pi main + RP2040 weather
- **Dual Weather**: Both base stations have full weather capability
- **Data Backup**: RP2040 logs weather data independently
- **Network**: Redundant mesh paths, multiple base receivers

### System Wide
- **Distributed Storage**: Images copied across mesh nodes
- **Multiple Paths**: Mesh routing with automatic failover
- **Metadata Embedded**: No external database dependencies
- **Local Processing**: AI and weather prediction on-device