use anyhow::Result;
use axum::{
    extract::{Path, Query, State},
    http::StatusCode,
    response::Json,
    routing::{get, post},
    Router,
};
use chrono::{DateTime, Utc};
use clap::Parser;
use serde::{Deserialize, Serialize};
use sqlx::{sqlite::SqlitePool, Row};
use std::{collections::HashMap, net::SocketAddr};
use tokio::{net::TcpListener, spawn};
use tracing::{info, warn};

#[derive(Parser)]
#[command(name = "taiga-base")]
#[command(about = "TAIGA Base Station - Wildlife Monitoring Hub")]
struct Args {
    #[arg(long, default_value = "0.0.0.0:3000")]
    bind: SocketAddr,
    
    #[arg(long, default_value = "taiga.db")]
    database: String,
    
    #[arg(long, default_value = "/mnt/images")]
    image_path: String,
}

#[derive(Clone)]
struct AppState {
    db: SqlitePool,
    image_path: String,
}

#[derive(Serialize, Deserialize)]
struct CameraImage {
    id: i64,
    camera_id: String,
    timestamp: DateTime<Utc>,
    detection_class: Option<String>,
    confidence: Option<f64>,
    gps_lat: Option<f64>,
    gps_lon: Option<f64>,
    temperature: Option<f64>,
    wind_speed: Option<f64>,
    wind_direction: Option<f64>,
}

#[derive(Serialize, Deserialize)]
struct WeatherData {
    timestamp: DateTime<Utc>,
    temperature: f64,
    humidity: f64,
    pressure: f64,
    wind_speed: f64,
    wind_direction: f64,
    rainfall_mm: f64,
    uv_index: f64,
}

#[derive(Serialize, Deserialize)]
struct LightningEvent {
    id: i64,
    timestamp: DateTime<Utc>,
    distance_km: Option<i32>, // 1-63km, None for out of range
    energy: i32, // Raw energy value from AS3935
    event_type: String, // "lightning", "disturber", or "noise"
    strike_count: i32,
    temperature: Option<f64>,
    humidity: Option<f64>,
    pressure: Option<f64>,
    wind_speed: Option<f64>,
    wind_direction: Option<f64>,
}

#[derive(Serialize, Deserialize)]
struct StormTracking {
    id: i64,
    storm_start: DateTime<Utc>,
    storm_end: Option<DateTime<Utc>>,
    total_strikes: i32,
    max_intensity: Option<i32>,
    closest_distance_km: Option<i32>,
    storm_direction: Option<f64>, // Degrees, storm movement direction
    storm_speed_kmh: Option<f64>, // Storm movement speed
    is_approaching: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    
    let args = Args::parse();
    info!("TAIGA Base Station starting...");
    
    // Initialize database
    let db = SqlitePool::connect(&format!("sqlite:{}", args.database)).await?;
    sqlx::migrate!().run(&db).await?;
    
    let state = AppState {
        db,
        image_path: args.image_path,
    };
    
    // Spawn background tasks
    spawn(radio_receiver_task(state.clone()));
    spawn(weather_processor_task(state.clone()));
    spawn(mesh_network_task(state.clone()));
    
    // Build web application
    let app = Router::new()
        .route("/", get(dashboard))
        .route("/api/images", get(list_images))
        .route("/api/images/:id", get(get_image))
        .route("/api/weather", get(get_weather))
        .route("/api/cameras", get(list_cameras))
        .route("/api/status", get(system_status))
        .route("/api/lightning", get(get_lightning_events))
        .route("/api/lightning/recent", get(get_recent_lightning))
        .route("/api/storms", get(get_active_storms))
        .route("/api/storms/:id", get(get_storm_details))
        .with_state(state);
    
    info!("Starting web server on {}", args.bind);
    let listener = TcpListener::bind(args.bind).await?;
    axum::serve(listener, app).await?;
    
    Ok(())
}

async fn dashboard() -> &'static str {
    // TODO: Serve web dashboard HTML
    "TAIGA Base Station - Wildlife Monitoring Dashboard"
}

async fn list_images(
    State(state): State<AppState>,
    Query(params): Query<HashMap<String, String>>,
) -> Result<Json<Vec<CameraImage>>, StatusCode> {
    // TODO: Query images from database with filtering
    info!("Listing images with filters: {:?}", params);
    
    // Placeholder implementation
    let images = vec![];
    Ok(Json(images))
}

async fn get_image(
    State(state): State<AppState>,
    Path(id): Path<i64>,
) -> Result<Json<CameraImage>, StatusCode> {
    // TODO: Get specific image by ID
    info!("Getting image ID: {}", id);
    Err(StatusCode::NOT_FOUND)
}

async fn get_weather(State(state): State<AppState>) -> Result<Json<WeatherData>, StatusCode> {
    // TODO: Get latest weather data from RP2040
    info!("Getting current weather data");
    
    let weather = WeatherData {
        timestamp: Utc::now(),
        temperature: 0.0,
        humidity: 0.0,
        pressure: 0.0,
        wind_speed: 0.0,
        wind_direction: 0.0,
        rainfall_mm: 0.0,
        uv_index: 0.0,
    };
    
    Ok(Json(weather))
}

async fn list_cameras(State(state): State<AppState>) -> Result<Json<Vec<String>>, StatusCode> {
    // TODO: List all known camera IDs
    info!("Listing camera nodes");
    Ok(Json(vec!["North Trail".to_string(), "South Ridge".to_string()]))
}

async fn system_status(State(state): State<AppState>) -> Result<Json<HashMap<String, String>>, StatusCode> {
    // TODO: Get system status from all components
    info!("Getting system status");
    
    let mut status = HashMap::new();
    status.insert("status".to_string(), "operational".to_string());
    status.insert("uptime".to_string(), "24h".to_string());
    status.insert("cameras_online".to_string(), "8".to_string());
    
    Ok(Json(status))
}

async fn radio_receiver_task(state: AppState) {
    info!("Starting 900MHz radio receiver task");
    // TODO: Implement 900MHz mesh network receiver
    // - Listen for incoming image packets
    // - Reassemble fragmented images
    // - Store images and metadata in database
    // - Handle mesh routing and acknowledgments
    
    loop {
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        // Radio processing loop
    }
}

async fn weather_processor_task(state: AppState) {
    info!("Starting weather data processor task");
    // TODO: Implement communication with RP2040 weather processor
    // - Read weather data via I2C/SPI
    // - Store weather measurements in database
    // - Generate weather trends and predictions
    // - Broadcast weather updates to mesh network
    
    loop {
        tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
        // Weather processing every 10 seconds
    }
}

async fn mesh_network_task(state: AppState) {
    info!("Starting mesh network management task");
    // TODO: Implement mesh network coordination
    // - Monitor network topology
    // - Handle routing table updates
    // - Manage image distribution priorities
    // - Coordinate with Meshtastic for commands
    
    loop {
        tokio::time::sleep(tokio::time::Duration::from_secs(30)).await;
        // Network management every 30 seconds
    }
}

async fn get_lightning_events(
    State(_state): State<AppState>,
    Query(_params): Query<HashMap<String, String>>,
) -> Result<Json<Vec<LightningEvent>>, StatusCode> {
    info!("Getting lightning events");
    // TODO: Query lightning events from database with filtering
    // Support filters: start_time, end_time, event_type, min_distance, max_distance
    
    // Placeholder implementation
    let events = vec![];
    Ok(Json(events))
}

async fn get_recent_lightning(State(_state): State<AppState>) -> Result<Json<Vec<LightningEvent>>, StatusCode> {
    info!("Getting recent lightning activity (last 24 hours)");
    // TODO: Query lightning events from last 24 hours
    // Return only actual lightning strikes, filter out noise/disturbers
    
    let recent_events = vec![];
    Ok(Json(recent_events))
}

async fn get_active_storms(State(_state): State<AppState>) -> Result<Json<Vec<StormTracking>>, StatusCode> {
    info!("Getting active storm tracking data");
    // TODO: Query active storms (storm_end IS NULL)
    // Include storm progression, intensity, and approach/departure status
    
    let active_storms = vec![];
    Ok(Json(active_storms))
}

async fn get_storm_details(
    State(_state): State<AppState>,
    Path(_id): Path<i64>,
) -> Result<Json<StormTracking>, StatusCode> {
    info!("Getting storm details for ID: {}", _id);
    // TODO: Get detailed storm tracking data including:
    // - All lightning events for this storm
    // - Storm path and movement analysis
    // - Intensity progression over time
    // - Strike distribution and patterns
    
    Err(StatusCode::NOT_FOUND)
}