-- Initial database schema for TAIGA base station
CREATE TABLE IF NOT EXISTS images (
    id INTEGER PRIMARY KEY,
    camera_id TEXT NOT NULL,
    timestamp DATETIME NOT NULL,
    gps_lat REAL,
    gps_lon REAL,
    detection_class TEXT,
    confidence REAL,
    temperature REAL,
    humidity REAL,
    wind_speed REAL,
    wind_direction REAL,
    thumbnail BLOB,
    full_image_path TEXT
);

CREATE INDEX IF NOT EXISTS idx_images_camera_id ON images(camera_id);
CREATE INDEX IF NOT EXISTS idx_images_timestamp ON images(timestamp);
CREATE INDEX IF NOT EXISTS idx_images_detection ON images(detection_class);