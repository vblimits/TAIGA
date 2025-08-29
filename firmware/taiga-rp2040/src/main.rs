#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_time::{Duration, Timer};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    
    #[cfg(feature = "camera-node")]
    info!("TAIGA RP2040 Camera Node Supervisor starting...");
    
    #[cfg(feature = "base-station")]
    info!("TAIGA RP2040 Base Station Processor starting...");

    // Core tasks common to both variants
    spawner.spawn(mppt_controller()).unwrap();
    spawner.spawn(system_monitor()).unwrap();
    spawner.spawn(wind_measurement()).unwrap();
    
    // Feature-specific tasks
    #[cfg(feature = "camera-node")]
    spawner.spawn(esp32_monitor(p.PIN_3.into())).unwrap();
    
    #[cfg(feature = "base-station")]
    {
        spawner.spawn(openwind_processor()).unwrap();
        spawner.spawn(rain_gauge_monitor(p.PIN_2.into())).unwrap();
        spawner.spawn(lightning_detector(p.PIN_5.into())).unwrap();
        spawner.spawn(pi_monitor(p.PIN_4.into())).unwrap();
    }

    loop {
        Timer::after(Duration::from_secs(30)).await;
        
        #[cfg(feature = "camera-node")]
        info!("Camera RP2040 heartbeat - MPPT + Supervisor operational");
        
        #[cfg(feature = "base-station")]
        info!("Base RP2040 heartbeat - MPPT + Weather station operational");
    }
}

#[embassy_executor::task]
async fn mppt_controller() {
    info!("MPPT Controller starting on Core 0");
    // TODO: Implement MPPT charge controller
    // - Solar voltage/current monitoring
    // - Battery voltage/temperature monitoring  
    // - Perturb & Observe algorithm
    // - PWM control of MOSFET switching
    // - Temperature compensation for LiFePO4
    // 
    // Power scaling based on variant:
    // - Camera nodes: ~20W solar, 10-20Ah battery
    // - Base stations: ~100W+ solar, 50-100Ah battery
    
    loop {
        Timer::after(Duration::from_millis(100)).await;
        // MPPT algorithm runs at 10Hz for both variants
    }
}

#[embassy_executor::task]
async fn system_monitor() {
    info!("System Monitor task starting on Core 1");
    // TODO: Implement system health monitoring
    // - Environmental sensor reading (common sensors)
    // - Data logging to flash (30-day ring buffer)
    // - Battery management and health tracking
    // - Power consumption profiling
    // - Anomaly detection and alerts
    
    loop {
        Timer::after(Duration::from_secs(600)).await;
        // System monitoring runs every 10 minutes
    }
}

#[embassy_executor::task]
async fn wind_measurement() {
    #[cfg(feature = "camera-node")]
    info!("Wind Measurement (3-sensor) starting");
    
    #[cfg(feature = "base-station")]
    info!("Wind Measurement (6-sensor OpenWind) starting");
    
    // TODO: Implement ultrasonic wind measurement
    // Camera nodes: 3x JSN-SR04T sensors (Carl47 algorithm)
    // Base stations: 6x JSN-SR04T sensors (OpenWind algorithm)
    // - Time-of-flight measurements
    // - Temperature compensation
    // - Wind vector calculation
    // - Gust detection and averaging
    
    loop {
        Timer::after(Duration::from_secs(60)).await;
        // Wind measurements every minute for both variants
    }
}

// Camera node specific tasks
#[cfg(feature = "camera-node")]
#[embassy_executor::task]
async fn esp32_monitor(_reset_pin: gpio::AnyPin) {
    info!("ESP32-S3 Monitor starting");
    // TODO: Implement ESP32-S3 watchdog and reset
    // - Heartbeat monitoring via GPIO
    // - Hard reset capability when ESP32-S3 hangs
    // - Communication via I2C for status/commands
    // - Power control for ESP32-S3 subsystem
    
    loop {
        Timer::after(Duration::from_secs(1)).await;
        // Monitor ESP32-S3 health every second
    }
}

// Base station specific tasks
#[cfg(feature = "base-station")]
#[embassy_executor::task]
async fn openwind_processor() {
    info!("OpenWind 6-sensor processor starting");
    // TODO: Implement professional OpenWind weather station
    // - 6x JSN-SR04T ultrasonic sensors in hexagonal array
    // - Precision time-of-flight measurement (microsecond accuracy)
    // - Least-squares wind vector calculation
    // - Temperature compensation for sound speed
    // - Weather prediction using pressure trends
    
    loop {
        Timer::after(Duration::from_secs(1)).await;
        // Professional weather measurements at 1Hz
    }
}

#[cfg(feature = "base-station")]
#[embassy_executor::task]
async fn rain_gauge_monitor(_interrupt_pin: gpio::AnyPin) {
    info!("Rain gauge monitor starting");
    // TODO: Implement rain gauge processing
    // - Tipping bucket interrupt counting
    // - 0.2mm per tip resolution
    // - Rainfall rate calculation
    // - Daily accumulation tracking
    
    loop {
        Timer::after(Duration::from_secs(5)).await;
        // Check rain gauge status every 5 seconds
    }
}

#[cfg(feature = "base-station")]
#[embassy_executor::task]
async fn lightning_detector(_interrupt_pin: gpio::AnyPin) {
    info!("DFRobot Lightning Detector (AS3935) starting");
    // TODO: Implement AS3935 lightning detection via manual I2C
    // 
    // DFRobot Lightning Sensor (SEN0290) based on AS3935 chipset
    // I2C Address: 0x03 (default, configurable to 0x01, 0x02, 0x03)
    // 
    // Key registers:
    // - 0x00: AFE Gain Boost (indoor/outdoor mode)
    // - 0x01: Watchdog threshold, noise floor level
    // - 0x02: Spike rejection, disturber detection
    // - 0x03: Lightning interrupt, distance estimation
    // - 0x04-0x06: Energy calculation (lightning intensity)
    // - 0x07: Distance to storm front (1-63km, 0=out of range)
    // - 0x08: Oscillator calibration
    // 
    // Interrupt types:
    // - 0x01: Noise level too high
    // - 0x04: Disturber detected  
    // - 0x08: Lightning detected
    // 
    // Implementation plan:
    // 1. Initialize AS3935 over I2C (Embassy RP I2C driver)
    // 2. Configure for outdoor operation (AFE gain, thresholds)
    // 3. Monitor interrupt pin for lightning events
    // 4. Read strike distance and energy when detected
    // 5. Implement disturber rejection algorithms
    // 6. Track storm patterns and approach/departure
    // 7. Send lightning data to Pi via I2C for database storage
    
    loop {
        Timer::after(Duration::from_secs(1)).await;
        // Monitor interrupt pin for AS3935 events
        // Process lightning strikes, disturbances, noise
        // Update storm tracking data
        // Communicate lightning events to Raspberry Pi
    }
}

#[cfg(feature = "base-station")]
#[embassy_executor::task]
async fn pi_monitor(_reset_pin: gpio::AnyPin) {
    info!("Raspberry Pi monitor starting");
    // TODO: Implement Pi monitoring and communication
    // - Different approach than ESP32-S3 (graceful vs hard reset)
    // - I2C/SPI communication for weather + power data
    // - Pi health monitoring via heartbeat
    // - Optional Pi restart capability
    // - Command interface for calibration
    
    loop {
        Timer::after(Duration::from_secs(1)).await;
        // Monitor Pi health every second
    }
}