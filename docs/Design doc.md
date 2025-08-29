# TAIGA - Tactical Animal Intelligence & Geographic Analysis
## Off-Grid Trail Camera System Design Document
### North Alberta Deployment

### Executive Summary
TAIGA is a comprehensive wildlife monitoring and security system designed for harsh North Alberta conditions (-40°C to +30°C). The system combines AI-powered wildlife identification, distributed mesh storage, and hyperlocal weather monitoring to provide unparalleled hunting intelligence and property security.

**System Name:** TAIGA  
**Configuration:** 8 camera nodes + 2 base stations  
**Key Innovation:** Every camera measures local wind conditions for scent/movement prediction  
**Data Architecture:** All metadata embedded in JPEG EXIF - no database required

### System Overview

**TAIGA System Components:**
- **TAIGA-CAM**: 8x intelligent camera nodes with local wind sensing
- **TAIGA-BASE**: 2x base stations with OpenWind weather arrays
- **TAIGA-MESH**: 900MHz image distribution network
- **TAIGA-AI**: On-device wildlife classification (whitetail buck/doe, moose, bear, wolf)

**Primary Functions:**
- Security monitoring for cabin protection
- Wildlife tracking with species/gender identification  
- Hyperlocal wind measurement at every camera
- Distributed image storage with theft resilience
- Meshtastic network node support
- Remote configuration and monitoring

**Key Specifications:**
- Detection Range: ~10 meters
- Camera: 5MP with 850nm IR illumination
- Operating Temperature: -40°C to +30°C
- Power: Solar + LiFePO4 battery with custom MPPT
- Communication: 900MHz image mesh + Meshtastic
- Local Storage: 64GB SD card per unit
- Environmental: Wind, temperature, humidity, pressure at EVERY camera
- AI: On-device TinyYOLO for wildlife classification
- Data Storage: All metadata embedded in JPEG EXIF

### Hardware Architecture

#### TAIGA-CAM Node Components

**Core Processing:**
- MCU: ESP32-S3 (dual-core, AI acceleration)
  - Handles camera, YOLO inference, mesh networking
  - Built-in hardware watchdog for reliability
  - Cost: ~$8-12 CAD

**Detection System:**
- Primary: PIR sensor (AM312 or similar)
  - Ultra-low power wake trigger (~10µA standby)
  - Cost: ~$2
- Secondary: 24GHz mmWave radar (LD2410 or similar)
  - Velocity, direction, size estimation
  - False positive filtering
  - Cost: ~$5-8

**Imaging:**
- Camera Module: OV5640 5MP or similar
  - CSI interface to ESP32-S3
  - Cost: ~$15-20
- IR Illumination: 850nm LED array
  - 10m illumination range
  - Cost: ~$10

**Environmental Sensors:**
- Temperature/Humidity: SHT31 or BME280
  - Metadata and system monitoring
  - Cost: ~$3-5 CAD
- **Pressure Sensor: BME280 or BMP390**
  - Barometric pressure for weather prediction
  - Altitude compensation
  - Cost: ~$5 CAD (BME280 includes temp/humidity)
- IMU: MPU6050
  - Theft detection trigger
  - Camera orientation tracking
  - Cost: ~$2 CAD
- GPS: NEO-6M
  - Location/time sync
  - Theft tracking
  - Cost: ~$8-10 CAD
- **Wind Sensor (ALL cameras):**
  - Based on Carl47 open-source design
  - 3x JSN-SR04T waterproof ultrasonic sensors
  - 120° triangular configuration
  - Processed by RP2040 supervisor
  - Critical for microclimate detection
  - Cost: ~$24 CAD per camera

**Communication:**
- Meshtastic: RA-01H or similar LoRa module
  - Text alerts, configuration commands
  - Cost: ~$15 CAD
- Image Mesh: EByte E32-900T30D or similar
  - 900MHz ISM band
  - 1W output for forest penetration
  - 19.2-115.2 kbps data rate
  - Cost: ~$15-20 CAD

**Supervisor MCU & MPPT:**
- MCU: RP2040 (dual core)
  - Core 0: MPPT control algorithm
  - Core 1: System monitoring & logging
  - Cost: ~$1.50 CAD
- MPPT Components:
  - N-channel MOSFET (IRLZ44N or similar): $2 CAD
  - Current sense resistors (0.1Ω 1%): $0.50 CAD
  - Inductor (100µH, 3A): $2 CAD
  - Capacitors: $1 CAD
  - Total MPPT addition: ~$7-8 CAD

**Storage:**
- 64GB microSD card (user-supplied)
- SD card interface via ESP32-S3 SDIO
- 2MB Flash on RP2040 for system logs

**Power System:**
- Solar Panel: 20-25W (separate purchase ~$40-60 CAD)
- Battery: LiFePO4 3.2V cells in 4S configuration
  - 12.8V nominal, 14.6V max
  - Capacity: 10-20Ah recommended
  - Cost: ~$60-100 CAD
- Power Management: 
  - RP2040-based MPPT (replaces CN3791)
  - Buck converters for 3.3V and 5V rails
  - Cost: ~$8 CAD (integrated with supervisor)

**Mechanical:**
- Enclosure: CNC machined 6061 aluminum
  - Slide-lock mounting system
  - Tree-bolt interface
- Connector: MIL-spec single connector
  - Power + digital bus
  - Environmental sealed
  - Cost: ~$15-20 CAD

**Canadian Suppliers:**
- DigiKey.ca (Winnipeg warehouse - fast shipping)
- Mouser.ca (Ontario warehouse)
- Solarbotics.com (Calgary based)
- RobotShop.ca (Montreal based)
- Elmwood Electronics (Canadian distributor)

**Estimated Cost per Camera Node:** 
- Components: ~$168-208 CAD
- Enclosure/mechanical: ~$40-60 CAD
- **Total: $208-268 CAD** (within target)

#### TAIGA-BASE Station Components

**Processing:**
- **Raspberry Pi Zero 2 W**
  - WiFi capability (normally disabled)
  - Handles web interface and storage
  - Cost: ~$20-25 CAD
- **RP2040 Co-processor (on carrier board)**
  - Precision timing for ultrasonic sensors
  - Hardware interrupt counting for rain gauge
  - Real-time sensor sampling
  - Dedicated weather processing
  - Cost: ~$1.50 CAD

**TAIGA-BASE Carrier Board Design:**
```
Base Station Carrier Board Components:
- RP2040 microcontroller
- 6x JSN-SR04T connectors (OpenWind)
- Rain gauge interrupt input (RJ11 connector)
- BME280 for pressure/temp/humidity
- VEML6070 UV sensor
- RTC with backup battery (DS3231)
- Power regulation (5V, 3.3V rails)
- USB-C for programming/debug
- GPIO header to Pi Zero
- 900MHz radio interface
- Meshtastic radio interface
- Status LEDs
```

**Weather Station (Both Base Stations):**
Since you're making a carrier board, might as well equip BOTH base stations with weather capability for redundancy:
- **Wind Sensor:** OpenWind 6-sensor ultrasonic design
  - Higher accuracy than camera sensors
  - Full 360° wind rose data
  - Components: 6x JSN-SR04T sensors
  - Cost: ~$48 CAD
- **Rain Gauge:** Tipping bucket sensor
  - 0.2mm resolution
  - Magnetic reed switch counter
  - Cost: ~$15-20 CAD
- **Weather Processor:** RP2040 (on carrier)
  - Handles ultrasonic timing measurements
  - Counts rain gauge tips via interrupt
  - Manages all sensor polling
  - Communicates with Pi via I2C/SPI
  - Cost: ~$1.50 CAD
- **Additional Sensors:**
  - BME280 for pressure/temp/humidity
  - UV sensor (VEML6070)
  - RTC for accurate timestamping
  - Total additional: ~$15 CAD

**Communication:**
- 900MHz receiver: Matching E32-900T30D
- High-gain antenna: 9dBi directional
- Meshtastic module for control

**Storage:**
- External USB drive (user-supplied)
- SD card for OS and buffering

**Power:**
- 5V USB power supply (base stations have AC power)
- Optional battery backup
- Supercapacitor for RTC backup

**Estimated Base Station Cost:** 
- Pi Zero 2 W: $25 CAD
- Carrier board PCB: $15 CAD
- RP2040 + components: $20 CAD
- Weather sensors: $65 CAD
- Radios + antennas: $35 CAD
- **Total per base station: ~$160 CAD**

### Software Architecture

#### Supervisor MCU Firmware (RP2040 - Rust)

**Development Stack:**
- Language: Rust (no_std)
- Framework: embassy-rp for async runtime
- Key crates:
  - `embassy-rp` - Hardware abstraction
  - `fixed` - Fixed-point math for MPPT
  - `postcard` - Efficient serialization for logs
  - `heapless` - Ring buffer for sensor history

**Dual Core Architecture:**

**Core 0 - MPPT Controller & Wind Measurement:**
```rust
#[embassy_executor::task]
async fn mppt_controller() {
    let mut last_power = 0.0;
    let mut duty_cycle = 0.5;
    let mut delta = 0.01;
    
    loop {
        // Read solar panel metrics
        let v_solar = adc.read_solar_voltage().await;
        let i_solar = adc.read_solar_current().await;
        let power = v_solar * i_solar;
        
        // Perturb & Observe with adaptive step size
        if power > last_power {
            if power - last_power > 0.1 {
                delta = 0.02; // Bigger steps when far from MPP
            } else {
                delta = 0.005; // Fine tune near MPP
            }
            duty_cycle += delta;
        } else {
            duty_cycle -= delta;
            delta *= 0.8; // Reduce step size on direction change
        }
        
        // Temperature compensation for LiFePO4
        let temp = read_battery_temp().await;
        let charge_voltage = compensate_voltage(14.6, temp);
        
        // Apply limits and safety
        duty_cycle = duty_cycle.clamp(0.1, 0.9);
        if battery_voltage > charge_voltage {
            duty_cycle = 0.0; // Stop charging
        }
        
        pwm.set_duty(duty_cycle);
        last_power = power;
        
        Timer::after(Duration::from_millis(100)).await;
    }
}

// Wind measurement task (Carl47 algorithm) - runs on cameras with wind sensors
#[embassy_executor::task] 
async fn wind_measurement() {
    // Based on Carl47's open-source ultrasonic anemometer
    // 3 sensors at 120° for redundancy and direction finding
    const SOUND_SPEED_0C: f32 = 331.3; // m/s at 0°C
    
    loop {
        let temp = sensors.temperature;
        // Adjust sound speed for temperature
        let c = SOUND_SPEED_0C * (1.0 + temp / 273.15).sqrt();
        
        // Measure time-of-flight between sensor pairs
        let tof_12 = measure_ultrasonic_tof(SENSOR_1, SENSOR_2).await;
        let tof_23 = measure_ultrasonic_tof(SENSOR_2, SENSOR_3).await;
        let tof_31 = measure_ultrasonic_tof(SENSOR_3, SENSOR_1).await;
        
        // Reverse measurements for differential calculation
        let tof_21 = measure_ultrasonic_tof(SENSOR_2, SENSOR_1).await;
        let tof_32 = measure_ultrasonic_tof(SENSOR_3, SENSOR_2).await;
        let tof_13 = measure_ultrasonic_tof(SENSOR_1, SENSOR_3).await;
        
        // Calculate wind components using Carl47's algorithm
        let v_12 = SENSOR_DISTANCE * (1.0/tof_12 - 1.0/tof_21) / 2.0;
        let v_23 = SENSOR_DISTANCE * (1.0/tof_23 - 1.0/tof_32) / 2.0;
        let v_31 = SENSOR_DISTANCE * (1.0/tof_31 - 1.0/tof_13) / 2.0;
        
        // Transform to x,y wind components
        let wind_x = (2.0 * v_12 - v_23 - v_31) / 3.0;
        let wind_y = (v_23 - v_31) * 0.577; // 1/sqrt(3)
        
        let wind_speed = (wind_x * wind_x + wind_y * wind_y).sqrt();
        let wind_direction = wind_y.atan2(wind_x).to_degrees();
        
        // Store in shared buffer for ESP32-S3
        WIND_DATA.lock().await.update(WindData {
            speed_kmh: wind_speed * 3.6,
            direction_deg: (wind_direction + 360.0) % 360.0,
            temperature: temp,
            timestamp: rtc.now(),
        });
        
        Timer::after(Duration::from_secs(60)).await; // Measure every minute
    }
}
```

**Core 1 - System Monitor:**
```rust
#[embassy_executor::task]
async fn system_monitor() {
    let mut log_buffer = RingBuffer::<SensorLog, 8640>::new(); // 30 days @ 10min
    let mut esp32_heartbeat_timeout = 0;
    
    loop {
        // Collect all sensor data
        let log_entry = SensorLog {
            timestamp: rtc.now(),
            temp_internal: sensors.read_temp_internal().await,
            temp_external: sensors.read_temp_external().await,
            humidity: sensors.read_humidity().await,
            battery_voltage: adc.read_battery().await,
            solar_voltage: adc.read_solar().await,
            solar_current: adc.read_solar_current().await,
            load_current: adc.read_load_current().await,
            esp32_alive: gpio.esp32_heartbeat.is_high(),
            wind_vibration: sample_imu_vibration().await,
        };
        
        // Store in ring buffer
        log_buffer.push(log_entry);
        
        // Check ESP32-S3 health
        if !log_entry.esp32_alive {
            esp32_heartbeat_timeout += 1;
            if esp32_heartbeat_timeout > 3 {
                // Hard reset main MCU
                gpio.esp32_reset.set_low();
                Timer::after(Duration::from_millis(100)).await;
                gpio.esp32_reset.set_high();
                esp32_heartbeat_timeout = 0;
                
                // Log reset event
                log_system_event(SystemEvent::MainMcuReset);
            }
        } else {
            esp32_heartbeat_timeout = 0;
        }
        
        // Detect anomalies
        if log_entry.temp_internal < -35.0 {
            // Enable battery heater if available
            gpio.battery_heater.set_high();
        }
        
        // Wind detection via IMU
        let vibration_level = analyze_vibration_pattern().await;
        if vibration_level > WindLevel::Moderate {
            // Signal ESP32 about high wind conditions
            gpio.wind_alert.set_high();
        }
        
        // Save critical logs to flash every hour
        if rtc.now().minute() == 0 {
            save_logs_to_flash(&log_buffer).await;
        }
        
        Timer::after(Duration::from_secs(600)).await; // 10 minute interval
    }
}

// Wind detection using IMU vibration analysis
async fn sample_imu_vibration() -> VibrationMetric {
    let mut samples = [0f32; 100];
    for i in 0..100 {
        samples[i] = imu.read_acceleration().await.magnitude();
        Timer::after(Duration::from_millis(10)).await;
    }
    
    // Calculate frequency domain characteristics
    let fft_result = simple_fft(&samples);
    let dominant_freq = find_peak_frequency(&fft_result);
    
    VibrationMetric {
        amplitude: samples.iter().max() - samples.iter().min(),
        frequency: dominant_freq,
        pattern: classify_vibration_pattern(dominant_freq),
    }
}
```

**Communication with ESP32-S3:**
- I2C bus for sensor data sharing
- GPIO heartbeat signal (toggle every second)
- GPIO reset control
- Interrupt line for critical events

**System Log Structure:**
```rust
struct SensorLog {
    timestamp: u32,           // Unix timestamp
    temp_internal: i16,       // 0.01°C resolution  
    temp_external: i16,       // 0.01°C resolution
    humidity: u16,            // 0.01% resolution
    battery_voltage: u16,     // mV
    solar_voltage: u16,       // mV
    solar_current: u16,       // mA
    load_current: u16,        // mA
    esp32_alive: bool,
    wind_vibration: u8,       // 0-255 scale
}
```

**Power Budget Addition:**
- RP2040 active: ~20mA @ 3.3V
- RP2040 dormant: ~1mA @ 3.3V
- Can clock down to save power when solar input is low
- Net positive because it replaces external MPPT controller

#### Camera Firmware (ESP32-S3 - Rust)

**Development Stack:**
- Language: Rust (no_std environment)
- Framework: esp-idf-hal + embassy for async
- Key crates:
  - `esp32s3-hal` - Hardware abstraction
  - `embassy` - Async runtime and power management
  - `serde` - YAML config parsing
  - `exif` - Metadata embedding
  - `nalgebra` - FOV calculations
  - `heapless` - Collections without allocation
  - `defmt` - Logging
  - `smoltcp` - Network stack

**Boot Sequence:**
1. Read YAML configuration from SD card
2. Initialize sensors and radios
3. Calibrate IMU and establish reference orientation
4. Sync GPS time (if available)
5. Enter main loop

**Main Operation Loop:**
```rust
// Pseudo-code structure
async fn main_loop() {
    loop {
        // Deep sleep until PIR interrupt
        embassy::sleep_until_interrupt().await;
        
        // Wake and validate with mmWave
        let detection = validate_detection().await;
        
        if detection.is_valid() {
            // Get current wind data from RP2040
            let wind = rp2040.get_current_wind().await;
            
            // Get orientation for FOV
            let orientation = imu.get_orientation().await;
            let fov_bounds = calculate_fov_bounds(
                gps_coords, 
                orientation, 
                camera_specs
            );
            
            // Capture and process
            for i in 0..config.burst_count {
                let image = capture_image().await;
                let ai_result = run_inference(&image).await;
                
                // Build comprehensive metadata
                let metadata = CameraMetadata {
                    // Standard fields
                    timestamp: rtc.now(),
                    gps_coords,
                    camera_id: config.camera_id,
                    
                    // Detection data
                    detection_type: ai_result.class,
                    detection_subclass: ai_result.subclass,
                    detection_confidence: ai_result.confidence,
                    mmwave_velocity: detection.velocity_mps,
                    mmwave_direction: detection.direction_deg,
                    
                    // Environmental
                    wind_speed_kmh: wind.speed_kmh,
                    wind_direction_deg: wind.direction_deg,
                    temperature_c: sensors.temp,
                    humidity_percent: sensors.humidity,
                    
                    // Camera orientation
                    compass_heading: orientation.yaw,
                    tilt_angle: orientation.pitch,
                    fov_polygon: fov_bounds,
                    
                    // System health
                    battery_voltage: adc.battery_v,
                    solar_current_ma: adc.solar_ma,
                    
                    // Burst info
                    burst_index: i,
                    burst_total: config.burst_count,
                };
                
                // Embed using hybrid approach
                embed_metadata_hybrid(&mut image, &metadata).await;
                save_to_sd(image).await;
                
                // Still create thumbnail for mesh transfer
                let thumbnail = create_thumbnail(image, 50_000).await;
                queue_for_mesh(thumbnail).await;
            }
            
            send_meshtastic_alert(detection, wind).await;
            
            Timer::after(Duration::from_secs(
                config.sleep_after_trigger
            )).await;
        }
    }
}

// Hybrid metadata embedding for maximum compatibility
fn embed_metadata_hybrid(image: &mut JpegImage, metadata: &CameraMetadata) {
    // 1. Standard EXIF tags for universal compatibility
    image.set_exif_tag(ExifTag::Make, "Custom Trail Camera");
    image.set_exif_tag(ExifTag::Model, &metadata.camera_id);
    image.set_exif_tag(ExifTag::DateTimeOriginal, metadata.timestamp);
    image.set_exif_tag(ExifTag::GPSLatitude, metadata.gps_coords.lat);
    image.set_exif_tag(ExifTag::GPSLongitude, metadata.gps_coords.lon);
    image.set_exif_tag(ExifTag::GPSAltitude, metadata.gps_coords.alt);
    
    // 2. Complete data as JSON in UserComment (EXIF tag 0x9286)
    // UserComment can hold up to 64KB - plenty for our ~2-3KB of data
    let json_data = serde_json::to_string(&metadata).unwrap();
    image.set_exif_tag(ExifTag::UserComment, &json_data);
    
    // 3. IPTC fields for searchability in photo management software
    image.set_iptc("Keywords", &format!(
        "{},{},{}-kmh,{}-deg,{}C", 
        metadata.detection_type,
        metadata.detection_subclass,
        metadata.wind_speed_kmh.round() as i32,
        metadata.wind_direction_deg.round() as i32,
        metadata.temperature_c.round() as i32
    ));
    
    // Short description in IPTC Caption
    image.set_iptc("Caption-Abstract", &format!(
        "{} detected by {} at {} | Wind: {}km/h @ {}° | Temp: {}°C",
        metadata.detection_type,
        metadata.camera_id,
        metadata.timestamp.format("%Y-%m-%d %H:%M:%S"),
        metadata.wind_speed_kmh,
        metadata.wind_direction_deg,
        metadata.temperature_c
    ));
    
    // 4. XMP for structured data (better Immich/Lightroom compatibility)
    let xmp = format!(r#"
    <?xpacket begin='﻿' id='W5M0MpCehiHzreSzNTczkc9d'?>
    <x:xmpmeta xmlns:x='adobe:ns:meta/'>
        <rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>
            <rdf:Description rdf:about=''
                xmlns:dc='http://purl.org/dc/elements/1.1/'
                xmlns:trail='http://trailcam.custom/1.0/'>
                <dc:description>{}</dc:description>
                <trail:cameraId>{}</trail:cameraId>
                <trail:detectionType>{}</trail:detectionType>
                <trail:detectionConfidence>{}</trail:detectionConfidence>
                <trail:windSpeed>{}</trail:windSpeed>
                <trail:windDirection>{}</trail:windDirection>
                <trail:temperature>{}</trail:temperature>
                <trail:humidity>{}</trail:humidity>
                <trail:mmwaveVelocity>{}</trail:mmwaveVelocity>
                <trail:compassHeading>{}</trail:compassHeading>
                <trail:fovPolygon>{}</trail:fovPolygon>
                <trail:batteryVoltage>{}</trail:batteryVoltage>
            </rdf:Description>
        </rdf:RDF>
    </x:xmpmeta>
    <?xpacket end='w'?>"#,
        metadata.detection_type,
        metadata.camera_id,
        metadata.detection_type,
        metadata.detection_confidence,
        metadata.wind_speed_kmh,
        metadata.wind_direction_deg,
        metadata.temperature_c,
        metadata.humidity_percent,
        metadata.mmwave_velocity,
        metadata.compass_heading,
        serde_json::to_string(&metadata.fov_polygon).unwrap(),
        metadata.battery_voltage
    );
    
    image.embed_xmp(&xmp);
    
    // 5. Backup critical data in MakerNote (EXIF tag 0x927C)
    // Some software ignores this, but it's good redundancy
    image.set_exif_tag(ExifTag::MakerNote, &json_data);
}

// Reading metadata back from images
fn extract_metadata(image_path: &Path) -> CameraMetadata {
    let exif = exif::Reader::new()
        .read_from_file(image_path)
        .unwrap();
    
    // Try UserComment first (our primary storage)
    if let Some(user_comment) = exif.get_field(Tag::UserComment, In::PRIMARY) {
        if let Ok(metadata) = serde_json::from_str(&user_comment.display_value()) {
            return metadata;
        }
    }
    
    // Fallback to parsing XMP if UserComment fails
    if let Ok(xmp_data) = extract_xmp(image_path) {
        return parse_xmp_metadata(&xmp_data);
    }
    
    // Last resort: reconstruct from standard EXIF + IPTC
    reconstruct_from_standard_tags(&exif)
}
```

**Metadata Storage Strategy:**
1. **Standard EXIF** (GPS, timestamp) - Universal compatibility
2. **UserComment JSON** - Complete data in one place (up to 64KB)
3. **IPTC Keywords** - Searchable in photo managers
4. **XMP Structured Data** - Professional software compatibility
5. **MakerNote Backup** - Redundancy for critical data

**Compatibility Matrix:**
| Software | Standard EXIF | UserComment | IPTC | XMP |
|----------|--------------|-------------|------|-----|
| ExifTool | ✓ | ✓ | ✓ | ✓ |
| Immich | ✓ | ✓ | ✓ | ✓ |
| Lightroom | ✓ | Partial | ✓ | ✓ |
| Windows Explorer | ✓ | No | Partial | No |
| macOS Finder | ✓ | No | ✓ | Partial |
| Linux (various) | ✓ | ✓ | ✓ | ✓ |
| Custom Software | ✓ | ✓ | ✓ | ✓ |

**IMU FOV Calculation:**
```rust
// Calculate camera field of view polygon for map display
fn calculate_fov_bounds(
    gps: GpsCoordinate,
    orientation: Orientation,
    camera: CameraSpecs
) -> Vec<GpsCoordinate> {
    // Camera FOV specs (configurable)
    let h_fov = camera.horizontal_fov; // e.g., 65 degrees
    let v_fov = camera.vertical_fov;   // e.g., 50 degrees
    let range = camera.effective_range; // e.g., 10 meters
    
    // Convert IMU quaternion to euler angles
    let heading = orientation.yaw;   // Compass direction
    let pitch = orientation.pitch;    // Up/down tilt
    let roll = orientation.roll;      // Side tilt
    
    // Calculate FOV corner points in 3D space
    let corners = [
        (-h_fov/2, -v_fov/2),  // Bottom left
        (h_fov/2, -v_fov/2),   // Bottom right
        (h_fov/2, v_fov/2),    // Top right
        (-h_fov/2, v_fov/2),   // Top left
    ];
    
    // Project to GPS coordinates
    corners.iter().map(|(h, v)| {
        let distance = range * cos(v.to_radians());
        let bearing = heading + h;
        
        // Calculate GPS point at bearing and distance
        gps_project_point(gps, bearing, distance)
    }).collect()
}
```

**Power Management:**
- Embassy's async runtime for efficient sleep
- Aggressive sleep modes between detections
- Configurable wake windows for mesh sync
- GPS powered only during sync or theft detection
- WiFi/BT disabled unless configuration mode

**Memory Safety Benefits:**
- No buffer overflows or use-after-free bugs
- Compile-time guaranteed thread safety
- Safe peripheral sharing with RTIC
- Predictable memory usage (no heap fragmentation)

**Mesh Networking:**
- Store-and-forward protocol
- Automatic routing to base station
- Distributed storage of thumbnails
- Priority queuing (own images > relay images)
- Zero-copy packet handling with DMA

**AI Processing:**
- TinyYOLO v3 optimized model (ONNX → Rust)
- Custom trained classes:
  - Whitetail deer (buck/doe differentiation)
  - Moose, bear, wolf, elk
  - Human, vehicle
  - Small animals (coyote, fox, etc.)
- Inference at capture time using ESP32-S3's vector instructions
- Results embedded in image EXIF

**Configuration (via YAML):**
```yaml
camera_id: "North Trail"
network:
  mesh_channel: 1
  meshtastic_channel: "LongFast"
capture:
  burst_count: 5
  burst_delay_ms: 1000
  sleep_after_trigger_sec: 30
detection:
  pir_sensitivity: 7
  mmwave_threshold: 0.8
  mmwave_min_distance_m: 2
  mmwave_max_distance_m: 10
power:
  gps_sync_hour: 3
  wake_for_mesh_minutes: 5
storage:
  keep_days: 365
  thumbnail_quality: 60
camera:
  horizontal_fov_deg: 65
  vertical_fov_deg: 50
  effective_range_m: 10
imu:
  calibration_samples: 100
  magnetic_declination: -12.5  # For North Alberta
```

#### Base Station Software (Pi Zero - Rust)

**Development Stack:**
- Language: Rust
- Framework: Tokio async runtime
- Key crates:
  - `axum` - Web server for image interface
  - `sqlx` - SQLite for image metadata database
  - `tokio-serial` - 900MHz radio and sensor communication
  - `meshtastic` - Meshtastic protocol implementation
  - `image` - Thumbnail processing

**Weather Station Components (Base Station #1):**
```rust
// OpenWind 6-sensor implementation for base station
struct WeatherStation {
    wind_sensors: [UltrasonicSensor; 6],
    rain_gauge: RainGauge,
    bme280: Bme280Sensor,  // Pressure, temp, humidity
    uv_sensor: VemlSensor,
    rp2040: Option<Rp2040Companion>, // Optional helper MCU
}

impl WeatherStation {
    async fn read_all_sensors(&self) -> WeatherData {
        WeatherData {
            wind: self.read_wind_openwind().await,
            rainfall: self.read_rainfall().await,
            pressure_hpa: self.bme280.read_pressure().await,
            temperature_c: self.bme280.read_temperature().await,
            humidity_percent: self.bme280.read_humidity().await,
            uv_index: self.uv_sensor.read_uv_index().await,
            pressure_trend: self.calculate_pressure_trend(3), // 3 hour trend
            weather_prediction: self.predict_local_weather().await,
        }
    }
    
    async fn predict_local_weather(&self) -> LocalWeatherForecast {
        // Use pressure trends and wind shifts for prediction
        let pressure_3h = self.calculate_pressure_trend(3);
        let pressure_24h = self.calculate_pressure_trend(24);
        let wind_shift = self.detect_wind_shift(6); // 6 hours
        
        // Traditional weather forecasting rules
        let forecast = match (self.pressure_hpa, pressure_3h) {
            (p, trend) if p < 1000.0 && trend < -1.0 => {
                "Major storm approaching"
            }
            (p, trend) if p > 1025.0 && trend > 0.5 => {
                "Fair weather, stable conditions"
            }
            (p, trend) if p < 1010.0 && trend > 1.0 => {
                "Storm passing, conditions improving"
            }
            _ => "Variable conditions"
        };
        
        // Alberta-specific patterns
        if wind_shift.from_west_to_east && pressure_3h > 2.0 {
            return LocalWeatherForecast::ChinookConditions;
        }
        
        LocalWeatherForecast {
            description: forecast,
            pressure_hpa: self.pressure_hpa,
            trend_3h: pressure_3h,
            trend_24h: pressure_24h,
            storm_probability: calculate_storm_chance(pressure_3h, wind_shift),
        }
    }
}

// Integration with camera network - broadcast weather with predictions
async fn broadcast_weather_to_cameras(&self, weather: WeatherData) {
    // Send weather data via Meshtastic to all cameras
    let message = format!(
        "WEATHER:{}kph@{}°,{}mm/hr,{}hPa({:+.1}),{}C,{}%RH,{}",
        weather.wind.speed_kmh,
        weather.wind.direction_deg,
        weather.rainfall.rate_mm_hr,
        weather.pressure_hpa,
        weather.pressure_trend, // Include trend
        weather.temperature_c,
        weather.humidity_percent,
        weather.weather_prediction
    );
    
    self.meshtastic.broadcast(message).await;
}

impl WeatherStation {
    async fn read_wind_openwind(&self) -> WindReading {
        // OpenWind uses 6 sensors in hexagonal pattern
        // Provides redundancy and better accuracy
        let mut measurements = Vec::new();
        
        // Take measurements between all sensor pairs
        for i in 0..6 {
            for j in i+1..6 {
                let tof_ij = self.measure_tof(i, j).await;
                let tof_ji = self.measure_tof(j, i).await;
                measurements.push((i, j, tof_ij, tof_ji));
            }
        }
        
        // OpenWind's least-squares solver for overdetermined system
        let (wind_speed, wind_direction) = solve_wind_vector(measurements);
        
        // Calculate additional metrics
        let gust_speed = self.calculate_gust_speed().await;
        let turbulence = self.calculate_turbulence_intensity().await;
        
        WindReading {
            speed_kmh: wind_speed * 3.6,
            direction_deg: wind_direction,
            gust_kmh: gust_speed * 3.6,
            turbulence_percent: turbulence * 100.0,
            timestamp: Utc::now(),
        }
    }
    
    async fn read_rainfall(&mut self) -> RainfallReading {
        // Each tip = 0.2mm of rain
        let tips = self.rain_gauge.get_tips_since_last_read();
        let rainfall_mm = tips as f32 * 0.2;
        
        // Calculate rainfall rate
        let duration = Utc::now() - self.rain_gauge.last_read_time;
        let rate_mm_per_hour = rainfall_mm / duration.num_seconds() as f32 * 3600.0;
        
        RainfallReading {
            accumulation_mm: rainfall_mm,
            rate_mm_hr: rate_mm_per_hour,
            daily_total_mm: self.rain_gauge.daily_total,
            timestamp: Utc::now(),
        }
    }
}

// Optional RP2040 companion for precise timing
struct Rp2040WeatherCompanion {
    // Handles microsecond-precision ultrasonic measurements
    // Counts rain gauge interrupts
    // Offloads Pi Zero for better accuracy
}

// Database schema for weather data
async fn init_weather_db(pool: &SqlitePool) {
    sqlx::query!(
        r#"
        CREATE TABLE IF NOT EXISTS weather_data (
            id INTEGER PRIMARY KEY,
            timestamp DATETIME,
            wind_speed_kmh REAL,
            wind_direction_deg REAL,
            wind_gust_kmh REAL,
            turbulence_percent REAL,
            rainfall_mm REAL,
            rainfall_rate_mm_hr REAL,
            temperature_c REAL,
            humidity_percent REAL,
            pressure_hpa REAL,
            uv_index REAL
        )"#
    ).execute(pool).await.unwrap();
}

// Integration with camera network
async fn broadcast_weather_to_cameras(&self, weather: WeatherData) {
    // Send weather data via Meshtastic to all cameras
    let message = format!(
        "WEATHER:{}kph@{}°,{}mm/hr,{}C,{}%RH",
        weather.wind_speed_kmh,
        weather.wind_direction_deg,
        weather.rainfall_rate,
        weather.temperature,
        weather.humidity
    );
    
    self.meshtastic.broadcast(message).await;
}
```

**Core Functions:**
- Receive and store images from mesh
- WiFi access point on demand
- REST API for Immich integration
- Web interface with weather dashboard
- **Real-time weather monitoring and logging**
- **Weather data correlation with wildlife activity**
- Meshtastic command interface
- Automatic FOV polygon generation for map display

**Database Schema:**
```sql
CREATE TABLE images (
    id INTEGER PRIMARY KEY,
    camera_id TEXT,
    timestamp DATETIME,
    gps_lat REAL,
    gps_lon REAL,
    fov_polygon TEXT,  -- GeoJSON polygon for Immich
    heading REAL,      -- Compass direction
    pitch REAL,        -- Camera tilt
    detection_class TEXT,
    confidence REAL,
    temperature REAL,
    humidity REAL,
    thumbnail BLOB,
    full_image_path TEXT,
    mesh_copies TEXT   -- JSON array of cameras that have copy
);
```

**Immich Integration:**
```rust
// Generate Immich-compatible metadata
async fn prepare_immich_metadata(image: &Image) -> ImmichMetadata {
    ImmichMetadata {
        device_id: image.camera_id.clone(),
        created_at: image.timestamp,
        latitude: image.gps_lat,
        longitude: image.gps_lon,
        // Custom FOV polygon for map visualization
        custom_metadata: json!({
            "fov_polygon": image.fov_polygon,
            "heading": image.heading,
            "detection": image.detection_class,
            "confidence": image.confidence,
            "temperature": image.temperature,
            "humidity": image.humidity
        })
    }
}
```

**Operation:**
1. Normal state: WiFi off, receiving only
2. Meshtastic command: "base wifi on"
3. Enable WiFi AP mode
4. Serve web interface with map view
5. Allow image download/management
6. Auto-disable WiFi after timeout

### Communication Protocols

#### Meshtastic Messages
- Alert format: `[Camera_ID]: [Detection_Type] @ [Time], [Count] photos, [Direction], [Speed]`
- **Weather broadcast**: `WEATHER:[speed]kph@[dir]°,[rain]mm/hr,[temp]C,[humidity]%RH`
- Commands: 
  - `config set [camera_id] [parameter] [value]`
  - `delete mesh copies [camera_id]`
  - `base wifi on/off`
  - `status request [camera_id]`
  - `weather request` - Get current conditions

#### 900MHz Mesh Protocol
- Packet size: 250 bytes
- Thumbnail transfer: ~200 packets (50KB)
- Full image transfer: ~2000 packets (500KB)
- **Weather data packets**: 50 bytes (efficient broadcast)
- ACK-based reliability
- Automatic retry with backoff

#### Wind Data Sharing
Cameras with wind sensors share data every 10 minutes:
- Wind speed and direction
- Temperature compensated values
- Gust detection
- Other cameras log this with their images for correlation

### Power Budget Analysis

**Winter Scenario (December/January):**
- Solar input: 2-3 hours × 20W × 0.15 efficiency = 6-9Wh/day
- **MPPT improvement: +15-30% efficiency = 7-12Wh/day**
- Battery capacity at -30°C: ~40% of rated
- Minimum battery: 20Ah LiFePO4 = 256Wh × 0.4 = 102Wh usable

**Daily Consumption:**
- ESP32-S3 sleep: 1mA × 24h × 3.3V = 0.08Wh
- **RP2040 monitoring: 1mA × 24h × 3.3V = 0.08Wh**
- Detections (20/day): 500mA × 30sec × 20 × 3.3V = 0.28Wh
- Mesh sync (4 hours/day): 50mA × 4h × 3.3V = 0.66Wh
- GPS sync: 50mA × 5min × 3.3V = 0.01Wh
- **MPPT overhead: 20mA × 4h × 3.3V = 0.26Wh** (during charging)
- **Total: ~1.37Wh/day**

**Survival Time:**
- Without solar: 102Wh / 1.37Wh = ~74 days
- With minimal winter solar + MPPT: Indefinite
- **MPPT advantage: 30% more energy harvested in low-light conditions**

### Thermal Management

**Cold Weather Considerations:**
- LiFePO4 self-heating circuit (if below -20°C during charge)
- Conformal coating on all PCBs
- Thermal isolation of battery compartment
- Industrial temp-rated components (-40°C to +85°C)

### Security Features

- IMU-triggered GPS tracking on movement
- Distributed image storage across all nodes
- Encrypted mesh communication (AES-128)
- Remote wipe capability for mesh-stored images
- Tamper detection logging
- **Camera orientation tracking for precise FOV mapping**
- **Immich integration for visual coverage analysis**

### Installation Guide

1. **Pre-deployment:**
   - Configure all SD cards with YAML files
   - Label cameras with assigned IDs
   - Charge batteries fully
   - Test mesh connectivity
   - **Calibrate IMU magnetic declination for region**

2. **Field Installation:**
   - Mount aluminum enclosure to tree at ~1.5m height
   - **Use compass app to record exact mounting direction**
   - **Level camera using IMU readout (0° pitch for horizontal)**
   - Install solar/antenna assembly 3-5m up tree
   - Connect MIL-spec cable
   - Angle solar panel for winter sun (60-70° from horizontal)
   - Orient antenna toward base station or next mesh node
   - Insert configured SD card
   - Slide and lock camera module

3. **Commissioning:**
   - Verify GPS lock (LED indicator)
   - **Confirm IMU orientation matches physical mounting**
   - Test PIR trigger locally
   - Confirm Meshtastic connectivity
   - Validate base station reception
   - **Test FOV polygon appears correctly on map**

### Maintenance Schedule

**Monthly:**
- Check battery voltage via Meshtastic
- Review detection statistics
- **Download system health logs from RP2040**
- **Check MPPT efficiency metrics**

**Seasonally:**
- Clean solar panels
- Adjust solar angle for season
- Check enclosure seals
- **Review power generation trends**
- **Analyze wind/vibration patterns**
- Update YOLO model if needed

**Annually:**
- Replace desiccant packs
- Inspect connectors for corrosion
- Retrieve and archive SD cards
- **Extract full year sensor logs from RP2040**
- **Calibrate current sense resistors**

### Future Enhancements

- Dual-spectrum camera (visible + thermal)
- Starlight sensor for color night vision
- Edge TPU for advanced AI models
- Satellite communication backup (Swarm)
- Bluetooth tag detection for asset tracking
- Audio classification (gunshots, vehicles)
- **3D coverage mapping with overlapping FOV analysis**
- **Automatic Immich album generation by detection type**
- **Historical heatmap generation from FOV data**

### BOM Summary (Per Camera)

| Component | Model | Supplier | Cost (CAD) |
|-----------|-------|----------|------------|
| MCU | ESP32-S3 | DigiKey.ca | $10 |
| Supervisor MCU | RP2040 | DigiKey.ca | $1.50 |
| Camera | OV5640 5MP | DigiKey.ca | $18 |
| PIR | AM312 | DigiKey.ca | $2 |
| mmWave | LD2410 | DigiKey.ca | $7 |
| GPS | NEO-6M | RobotShop.ca | $9 |
| IMU | MPU6050 | DigiKey.ca | $2 |
| Temp/Humidity | SHT31 | Mouser.ca | $4 |
| 900MHz Radio | E32-900T30D | Solarbotics.ca | $18 |
| LoRa | RA-01H | Elmwood | $15 |
| IR LEDs | 850nm Array | DigiKey.ca | $10 |
| Wind Sensors | 3x JSN-SR04T | DigiKey.ca | $24 |
| MPPT Components | Various | DigiKey.ca | $6 |
| Power Management | Various | DigiKey.ca | $8 |
| Connectors | MIL-spec | DigiKey.ca | $18 |
| PCB | Custom 4-layer | JLCPCB | $20 |
| Enclosure | Aluminum | Local CNC | $50 |
| Misc Components | | DigiKey.ca | $20 |
| **Total per camera** | | | **~$242 CAD** |

### Base Station BOM

| Component | Model | Supplier | Cost (CAD) |
|-----------|-------|----------|------------|
| **Standard Base Station** | | | |
| Pi Zero 2 W | | BuyAPI.ca | $25 |
| 900MHz Radio | E32-900T30D | Solarbotics.ca | $18 |
| Antenna | 9dBi Directional | DigiKey.ca | $15 |
| Power Supply | 5V 3A | Amazon.ca | $12 |
| Misc | | | $10 |
| **Subtotal** | | | **$80 CAD** |
| | | | |
| **Weather Station Add-on** | | | |
| Wind Sensors | 6x JSN-SR04T | DigiKey.ca | $48 |
| Rain Gauge | Tipping Bucket | Amazon.ca | $18 |
| RP2040 (optional) | | DigiKey.ca | $1.50 |
| BME280 | | DigiKey.ca | $8 |
| UV Sensor | VEML6070 | DigiKey.ca | $4 |
| **Weather Add-on Total** | | | **$80 CAD** |
| | | | |
| **Weather Base Station Total** | | | **$160 CAD** |

### System Features Summary

**TAIGA Intelligence Capabilities:**
- Smart MPPT charging with 15-30% efficiency gain
- Continuous system health monitoring
- Black-box logging (30 days of 10-minute samples)
- Automatic ESP32-S3 reset on hang
- **Wind measurement at EVERY camera location**
- Local weather prediction using pressure trends
- Temperature-compensated battery charging
- Power consumption profiling
- Anomaly detection and alerts
- Battery heater control for extreme cold
- True dual-MCU watchdog functionality

**Environmental Monitoring:**
- Wind speed and direction (every camera)
- Temperature, humidity, pressure (every camera)
- Professional weather stations at bases (OpenWind + rainfall)
- Solar generation patterns
- Battery health metrics
- Component thermal cycles

**TAIGA Hunting Intelligence:**
- **Wildlife movement correlation with hyperlocal weather**
- **Scent drift prediction at each camera location**
- **Pattern discovery** (e.g., "Bucks use north trail when local wind shifts from W to NW")
- **Microclimate analysis** across property
- **False trigger reduction in high wind**
- **Historical weather pattern correlation**

**System Reliability:**
- Hardware watchdog (ESP32-S3 internal)
- Supervisor watchdog (RP2040 external)
- Automatic recovery from crashes
- Forensic logging for field debugging
- Redundant sensor monitoring
- Distributed image storage across mesh

### Why TAIGA is Unique

**Hyperlocal Environmental Intelligence:**
- No other trail camera system measures wind at EVERY camera
- Discovers patterns invisible to traditional cameras
- Scent cone prediction for each camera position
- Correlates animal behavior with immediate conditions, not distant weather stations

**Professional-Grade Redundancy:**
- Images backed up across mesh network
- Dual weather base stations
- Metadata embedded in JPEG (no database to corrupt)
- Multiple MCUs prevent single point of failure

**True Off-Grid Capability:**
- Local weather prediction without internet
- Solar generation forecasting
- Adaptive power management
- Years of operation without intervention

### Deployment Strategy

**TAIGA Network Layout:**
- 8x TAIGA-CAM nodes with ultrasonic wind sensors
- Each camera measures its own microclimate
- Critical for Alberta terrain with varied topography:
  - Ridge winds differ from valley winds
  - Forest edges create turbulence
  - Clearings have different patterns than dense woods
  - Coulees and draws channel wind differently
- Local wind data essential for:
  - Understanding scent drift at each location
  - Correlating animal behavior with immediate conditions
  - Identifying thermal currents and eddies

**TAIGA-BASE Stations:**
- TAIGA-BASE 1: Primary weather station at cabin
  - OpenWind 6-sensor array for reference
  - Rain gauge for precipitation events
  - Full environmental monitoring
  - Acts as primary data collection point
- TAIGA-BASE 2: Secondary weather station
  - Different location for redundancy
  - Provides second weather reference point
  - Backup data collection

**Total TAIGA System Cost:**
- 8x TAIGA-CAM nodes: 8 × $242 = $1,936 CAD
- 2x TAIGA-BASE stations: 2 × $160 = $320 CAD
- **Total: ~$2,256 CAD** (within $2,400 budget)

### Base Station Carrier Board Advantages

**Why RP2040 on Carrier Makes Sense:**
- Already designing custom PCB
- RP2040 only adds $1.50 to BOM
- Microsecond timing precision for ultrasonics
- Hardware interrupts for rain gauge
- Offloads real-time tasks from Pi
- Can run even if Pi crashes
- Dual-core for parallel processing

**Carrier Board Architecture:**
```rust
// RP2040 Firmware for Base Station Carrier
impl BaseStationRP2040 {
    // Core 0: OpenWind ultrasonic processing
    async fn core0_wind_task() {
        loop {
            // 6 sensors, 15 measurements (all pairs)
            let measurements = measure_all_ultrasonic_pairs().await;
            let wind = openwind_solve(measurements);
            
            // Update shared buffer for Pi
            WIND_BUFFER.lock().update(wind);
            
            Timer::after(Duration::from_secs(1)).await;
        }
    }
    
    // Core 1: Everything else
    async fn core1_sensor_task() {
        loop {
            // Read all I2C sensors
            let weather = WeatherReading {
                pressure: bme280.read_pressure().await,
                temperature: bme280.read_temperature().await,
                humidity: bme280.read_humidity().await,
                uv_index: veml6070.read_uv().await,
                rain_tips: RAIN_COUNTER.load(Ordering::Relaxed),
                timestamp: rtc.now(),
            };
            
            // Send to Pi via I2C slave mode
            I2C_BUFFER.lock().update(weather);
            
            // Log to local flash for redundancy
            log_to_flash(weather).await;
            
            Timer::after(Duration::from_secs(10)).await;
        }
    }
    
    // Interrupt handler for rain gauge
    #[interrupt]
    fn RAIN_GAUGE_IRQ() {
        RAIN_COUNTER.fetch_add(1, Ordering::Relaxed);
        LAST_TIP_TIME.store(timestamp(), Ordering::Relaxed);
    }
}
```

**Benefits of Both Base Stations Having Weather:**
- Complete redundancy
- Compare readings between locations
- Better weather pattern detection
- One at cabin, one at different elevation/exposure
- If one fails, still have weather data

### Why Local Wind Matters in Alberta

**Microclimate Variations:**
- Wind can vary 180° between locations only 100m apart
- Ridges vs valleys can have 20+ km/h speed differences
- Tree lines create wind shadows and vortices
- Morning thermals flow opposite to evening winds

**Hunting Intelligence:**
- Each camera knows its exact scent cone
- Animals approach differently based on local wind
- Identify preferred travel routes for wind conditions
- Discover bedding areas based on wind protection

**Data Storage Advantage:**
- All environmental data embedded in JPEG EXIF
- No database means no data loss if base fails
- Each image is self-contained with complete context
- Can analyze images years later with all data intact

### Risk Mitigation

**Power Failure:**
- Oversized battery bank
- Conservative power budget
- Reduced feature set winter mode

**Environmental:**
- IP67 rating minimum
- Redundant sealing
- Drainage channels in enclosure

**Wildlife Damage:**
- Metal enclosure
- Elevated mounting of sensitive components
- No exposed wires

**Theft:**
- Distributed storage
- GPS tracking
- Camouflaged installation
- Security cable option

### Compliance Notes

- FCC Part 15 compliant (ISM band)
- Industry Canada RSS-210
- RoHS compliant components
- Wildlife camera regulations compliance

---

*Document Version: 1.0*  
*System Name: TAIGA (Tactical Animal Intelligence & Geographic Analysis)*  
*Target Deployment: North Alberta, Canada*  
*Operating Environment: -40°C to +30°C*  
*Design Philosophy: Every camera is a weather station, every image tells the complete environmental story*
