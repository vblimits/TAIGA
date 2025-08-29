-- Lightning detection data schema for TAIGA base station
CREATE TABLE IF NOT EXISTS lightning_events (
    id INTEGER PRIMARY KEY,
    timestamp DATETIME NOT NULL,
    distance_km INTEGER, -- 1-63km, NULL for out of range
    energy INTEGER NOT NULL, -- Raw energy value from AS3935
    event_type TEXT NOT NULL CHECK(event_type IN ('lightning', 'disturber', 'noise')),
    strike_count INTEGER DEFAULT 1, -- Number of strikes in this event
    temperature REAL, -- Environmental conditions at time of event
    humidity REAL,
    pressure REAL,
    wind_speed REAL,
    wind_direction REAL
);

CREATE TABLE IF NOT EXISTS storm_tracking (
    id INTEGER PRIMARY KEY,
    storm_start DATETIME NOT NULL,
    storm_end DATETIME,
    total_strikes INTEGER DEFAULT 0,
    max_intensity INTEGER,
    closest_distance_km INTEGER,
    storm_direction REAL, -- Degrees, storm movement direction
    storm_speed_kmh REAL, -- Storm movement speed
    is_approaching BOOLEAN DEFAULT TRUE
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_lightning_timestamp ON lightning_events(timestamp);
CREATE INDEX IF NOT EXISTS idx_lightning_distance ON lightning_events(distance_km);
CREATE INDEX IF NOT EXISTS idx_lightning_type ON lightning_events(event_type);
CREATE INDEX IF NOT EXISTS idx_storm_start ON storm_tracking(storm_start);
CREATE INDEX IF NOT EXISTS idx_storm_active ON storm_tracking(storm_end) WHERE storm_end IS NULL;