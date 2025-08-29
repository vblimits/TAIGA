#!/bin/bash
# TAIGA Build Script - Builds all components correctly

set -e  # Exit on any error

echo "🔨 TAIGA Build Script"
echo "===================="

case "${1:-all}" in
    "host"|"software")
        echo "Building host software..."
        cd software && cargo build
        ;;
    
    "firmware")
        echo "Building RP2040 firmware..."
        cd firmware
        echo "  📱 Camera node variant..."
        cargo build -p taiga-rp2040 --target thumbv6m-none-eabi --features camera-node
        echo "  🏠 Base station variant..."
        cargo build -p taiga-rp2040 --target thumbv6m-none-eabi --features base-station
        ;;
    
    "esp32")
        echo "Building ESP32-S3 firmware..."
        if [ ! -f ~/export-esp.sh ]; then
            echo "❌ ESP toolchain not found. Run: espup install"
            exit 1
        fi
        echo "⚠️  ESP32-S3 build currently broken due to xtensa-lx-rt incompatibility"
        echo "    The newer xtensa-lx crate exists but esp-hal still depends on xtensa-lx-rt"
        echo "    Issues: naked functions require #[unsafe(naked)] and naked_asm! macros"
        echo "    Status: esp-hal ecosystem transitioning to xtensa-lx, not yet complete"
        echo ""
        echo "Attempting build anyway (will likely fail)..."
        source ~/export-esp.sh
        cd firmware
        cargo +esp build -p taiga-cam --target xtensa-esp32s3-none-elf -Z build-std=core,alloc --release || echo "❌ ESP32-S3 build failed as expected"
        ;;
    
    "all")
        echo "Building all components..."
        $0 host
        echo ""
        $0 firmware
        echo ""
        echo "⚠️  ESP32 build skipped (run './build.sh esp32' separately)"
        ;;
        
    "clean")
        echo "Cleaning all build artifacts..."
        cargo clean
        cd software && cargo clean
        cd ../firmware && cargo clean
        ;;
    
    *)
        echo "Usage: $0 [host|firmware|esp32|all|clean]"
        echo ""
        echo "Commands:"
        echo "  host      - Build host software (base-station, ai-ml, tools)"
        echo "  firmware  - Build RP2040 firmware (both variants)"
        echo "  esp32     - Build ESP32-S3 firmware"
        echo "  all       - Build host + RP2040 firmware"
        echo "  clean     - Clean all build artifacts"
        exit 1
        ;;
esac

echo "✅ Build completed successfully!"