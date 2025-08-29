# TAIGA - Tactical Animal Intelligence & Geographic Analysis

> Off-Grid Trail Camera System for North Alberta Wildlife Monitoring

## Overview

TAIGA is a comprehensive wildlife monitoring and security system designed for harsh North Alberta conditions (-40°C to +30°C). The system combines AI-powered wildlife identification, distributed mesh storage, and hyperlocal weather monitoring to provide unparalleled hunting intelligence and property security.

## System Components

- **TAIGA-CAM**: 8x intelligent camera nodes with local wind sensing
- **TAIGA-BASE**: 2x base stations with OpenWind weather arrays  
- **TAIGA-MESH**: 900MHz image distribution network
- **TAIGA-AI**: On-device wildlife classification (whitetail buck/doe, moose, bear, wolf)

## Key Features

- Security monitoring for cabin protection
- Wildlife tracking with species/gender identification
- Hyperlocal wind measurement at every camera
- Distributed image storage with theft resilience
- Meshtastic network node support
- Remote configuration and monitoring

## Repository Structure

```
TAIGA/
├── firmware/         # Embedded firmware workspace (Rust)
│   ├── taiga-cam/         # ESP32-S3 camera firmware
│   └── taiga-rp2040/      # RP2040 unified firmware (camera + base variants)
├── software/         # Host software workspace (Rust)
│   ├── base-station/      # Pi Zero web interface & API
│   ├── ai-ml/             # TinyYOLO training & inference  
│   └── tools/             # Configuration & calibration tools
├── hardware/         # PCB design & mechanical
│   ├── pcb/          # KiCad schematics & layouts
│   ├── mechanical/   # CAD models & drawings
│   └── bom/          # Bills of materials
├── docs/             # Documentation
├── testing/          # Test plans & validation
├── scripts/          # Build & deployment automation
├── assets/           # 3D models, datasheets, images
├── build.sh          # Unified build script for all targets
└── Cargo.toml        # Workspace root configuration
```

## Getting Started

### Prerequisites

- **Rust toolchain** with embedded targets:
  ```bash
  # Install Rust
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  
  # Add embedded targets
  rustup target add thumbv6m-none-eabi  # For RP2040
  rustup target add xtensa-esp32s3-none-elf  # For ESP32-S3 (see ESP32 notes below)
  ```

- **ESP32 Toolchain** (for ESP32-S3 camera firmware):
  ```bash
  # Install espup
  cargo install espup
  espup install
  ```

- **Additional tools**:
  - Git for version control
  - KiCad for hardware design (optional)

### Build System

TAIGA uses a unified build script to handle the complex multi-target architecture:

```bash
# Clone repository
git clone https://github.com/vblimits/TAIGA.git
cd TAIGA

# Build all working components (host software + RP2040 firmware)
./build.sh all

# Build specific components
./build.sh host        # Host software only (base-station, ai-ml, tools)
./build.sh firmware    # RP2040 firmware (both camera and base-station variants)
./build.sh esp32       # ESP32-S3 firmware (see status notes below)

# Clean all build artifacts
./build.sh clean
```

### Component Status

| Component | Status | Notes |
|-----------|--------|-------|
| Host Software | ✅ Working | Base station, AI/ML, tools all build successfully |
| RP2040 Firmware | ✅ Working | Both camera and base-station variants |
| ESP32-S3 Firmware | ⚠️ Blocked | See ESP32 compatibility notes below |

### ESP32-S3 Build Issues

The ESP32-S3 camera firmware is currently blocked due to ecosystem compatibility issues:

- **Issue**: The `xtensa-lx-rt` crate hasn't been updated for newer Rust versions
- **Root cause**: Naked functions now require `#[unsafe(naked)]` and `naked_asm!` macros
- **Status**: The newer `xtensa-lx` crate exists but `esp-hal` ecosystem is still transitioning
- **Timeline**: Waiting for upstream `esp-rs` repository updates

**Workaround**: TAIGA camera nodes can use RP2040-only configurations until ESP32 support is restored.

### Lightning Detection Integration

The base station includes DFRobot lightning detection with the AS3935 chipset:

- Real-time storm tracking and distance measurement
- SQLite database storage for historical lightning events  
- REST API endpoints for lightning data retrieval
- Integration with weather monitoring system

### Workspace Architecture

TAIGA uses separate Cargo workspaces to avoid target conflicts:

- **Root workspace**: Coordinates builds across firmware and software
- **Firmware workspace**: Embedded targets (RP2040, ESP32-S3)
- **Software workspace**: Host targets (x86_64, ARM64)

This architecture prevents critical-section conflicts between embedded and host dependencies.

### Development Workflow

```bash
# Start development
./build.sh all                    # Verify everything builds

# Make changes to firmware
cd firmware/taiga-rp2040
cargo build --features camera-node    # Test camera variant
cargo build --features base-station   # Test base station variant

# Make changes to host software  
cd software/base-station
cargo run                        # Run base station locally
cargo test                       # Run tests

# Clean builds when switching between targets
cargo clean                      # Clean current workspace
./build.sh clean                 # Clean everything
```

## Documentation

- [Design Document](docs/Design%20doc.md) - Complete system specification
- [Architecture Overview](docs/ARCHITECTURE.md) - System architecture and communication
- [Hardware Guide](docs/hardware/) - PCB design and assembly
- [Installation Guide](docs/deployment/) - Field deployment procedures
- [API Reference](docs/api/) - Software interfaces

## License

[License information to be determined]

## Contributing

[Contributing guidelines to be determined]