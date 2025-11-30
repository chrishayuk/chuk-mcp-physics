use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::IntoResponse,
    routing::{delete, get, post},
    Json, Router,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tower_http::cors::CorsLayer;
use tracing::{info, warn};
use uuid::Uuid;

mod physics;
use physics::{Simulation, SimulationConfig};

// Application state
type AppState = Arc<RwLock<HashMap<String, Simulation>>>;

#[tokio::main]
async fn main() {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "rapier_physics_service=info,tower_http=info".into()),
        )
        .init();

    // Shared state for simulations
    let simulations: AppState = Arc::new(RwLock::new(HashMap::new()));

    // Build router
    let app = Router::new()
        .route("/health", get(health_check))
        .route("/simulations", post(create_simulation))
        .route("/simulations/:sim_id/state", get(get_simulation_state))
        .route("/simulations/:sim_id", delete(destroy_simulation))
        .route("/simulations/:sim_id/bodies", post(add_body))
        .route("/simulations/:sim_id/step", post(step_simulation))
        .route("/simulations/:sim_id/bodies/:body_id/trajectory", post(record_trajectory))
        .layer(CorsLayer::permissive())
        .with_state(simulations);

    // Start server
    let addr = "0.0.0.0:9000";
    info!("🦀 Rapier Physics Service starting on {}", addr);

    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

// Health check endpoint
async fn health_check() -> impl IntoResponse {
    Json(serde_json::json!({
        "status": "healthy",
        "service": "rapier-physics-service",
        "version": env!("CARGO_PKG_VERSION")
    }))
}

// Request/Response types
#[derive(Debug, Deserialize)]
struct CreateSimulationRequest {
    gravity: [f32; 3],
    dimensions: u8,
    #[serde(default = "default_dt")]
    dt: f32,
    #[serde(default)]
    integrator: String,
}

fn default_dt() -> f32 {
    0.016
}

#[derive(Debug, Serialize)]
struct CreateSimulationResponse {
    sim_id: String,
    config: SimulationConfigResponse,
}

#[derive(Debug, Serialize)]
struct SimulationConfigResponse {
    gravity: [f32; 3],
    dimensions: u8,
    dt: f32,
    integrator: String,
}

#[derive(Debug, Deserialize)]
struct AddBodyRequest {
    id: String,
    kind: String, // "static", "dynamic", "kinematic"
    shape: String, // "box", "sphere", "capsule"
    size: Vec<f32>,
    #[serde(default)]
    mass: Option<f32>,
    #[serde(default)]
    position: Option<[f32; 3]>,
    #[serde(default)]
    orientation: Option<[f32; 4]>, // quaternion [x, y, z, w]
    #[serde(default)]
    velocity: Option<[f32; 3]>,
    #[serde(default)]
    angular_velocity: Option<[f32; 3]>,
    #[serde(default = "default_friction")]
    friction: f32,
    #[serde(default = "default_restitution")]
    restitution: f32,
}

fn default_friction() -> f32 {
    0.5
}

fn default_restitution() -> f32 {
    0.3
}

#[derive(Debug, Serialize)]
struct AddBodyResponse {
    body_id: String,
}

#[derive(Debug, Deserialize)]
struct StepSimulationRequest {
    steps: usize,
    #[serde(default)]
    dt: Option<f32>,
}

#[derive(Debug, Serialize)]
struct SimulationStateResponse {
    sim_id: String,
    time: f32,
    bodies: Vec<BodyStateResponse>,
}

#[derive(Debug, Serialize)]
struct BodyStateResponse {
    id: String,
    position: [f32; 3],
    orientation: [f32; 4],
    velocity: [f32; 3],
    angular_velocity: [f32; 3],
    contacts: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct RecordTrajectoryRequest {
    steps: usize,
    #[serde(default)]
    dt: Option<f32>,
}

#[derive(Debug, Serialize)]
struct TrajectoryResponse {
    body_id: String,
    frames: Vec<TrajectoryFrame>,
    total_time: f32,
    num_frames: usize,
}

#[derive(Debug, Clone, Serialize)]
struct TrajectoryFrame {
    time: f32,
    position: [f32; 3],
    orientation: [f32; 4],
    velocity: [f32; 3],
}

// Handlers
async fn create_simulation(
    State(state): State<AppState>,
    Json(req): Json<CreateSimulationRequest>,
) -> Result<Json<CreateSimulationResponse>, StatusCode> {
    let sim_id = Uuid::new_v4().to_string();

    let config = SimulationConfig {
        gravity: req.gravity,
        dimensions: req.dimensions,
        dt: req.dt,
        integrator: if req.integrator.is_empty() {
            "verlet".to_string()
        } else {
            req.integrator
        },
    };

    let simulation = Simulation::new(config.clone());

    info!("Created simulation {}", sim_id);
    state.write().await.insert(sim_id.clone(), simulation);

    Ok(Json(CreateSimulationResponse {
        sim_id: sim_id.clone(),
        config: SimulationConfigResponse {
            gravity: config.gravity,
            dimensions: config.dimensions,
            dt: config.dt,
            integrator: config.integrator,
        },
    }))
}

async fn add_body(
    State(state): State<AppState>,
    Path(sim_id): Path<String>,
    Json(req): Json<AddBodyRequest>,
) -> Result<Json<AddBodyResponse>, StatusCode> {
    let mut sims = state.write().await;
    let sim = sims.get_mut(&sim_id).ok_or(StatusCode::NOT_FOUND)?;

    sim.add_body(
        req.id.clone(),
        req.kind,
        req.shape,
        req.size,
        req.mass,
        req.position,
        req.orientation,
        req.velocity,
        req.angular_velocity,
        req.friction,
        req.restitution,
    );

    info!("Added body {} to simulation {}", req.id, sim_id);

    Ok(Json(AddBodyResponse { body_id: req.id }))
}

async fn step_simulation(
    State(state): State<AppState>,
    Path(sim_id): Path<String>,
    Json(req): Json<StepSimulationRequest>,
) -> Result<Json<SimulationStateResponse>, StatusCode> {
    let mut sims = state.write().await;
    let sim = sims.get_mut(&sim_id).ok_or(StatusCode::NOT_FOUND)?;

    sim.step(req.steps, req.dt);

    Ok(Json(simulation_state_to_response(sim_id, sim)))
}

async fn get_simulation_state(
    State(state): State<AppState>,
    Path(sim_id): Path<String>,
) -> Result<Json<SimulationStateResponse>, StatusCode> {
    let sims = state.read().await;
    let sim = sims.get(&sim_id).ok_or(StatusCode::NOT_FOUND)?;

    Ok(Json(simulation_state_to_response(sim_id, sim)))
}

async fn record_trajectory(
    State(state): State<AppState>,
    Path((sim_id, body_id)): Path<(String, String)>,
    Json(req): Json<RecordTrajectoryRequest>,
) -> Result<Json<TrajectoryResponse>, StatusCode> {
    let mut sims = state.write().await;
    let sim = sims.get_mut(&sim_id).ok_or(StatusCode::NOT_FOUND)?;

    let mut frames = Vec::new();
    let start_time = sim.time;

    for _ in 0..req.steps {
        // Get body state before step
        if let Some(body_state) = sim.get_body_state(&body_id) {
            frames.push(TrajectoryFrame {
                time: sim.time,
                position: body_state.position,
                orientation: body_state.orientation,
                velocity: body_state.velocity,
            });
        }

        sim.step(1, req.dt);
    }

    let total_time = sim.time - start_time;

    Ok(Json(TrajectoryResponse {
        body_id,
        frames: frames.clone(),
        total_time,
        num_frames: frames.len(),
    }))
}

async fn destroy_simulation(
    State(state): State<AppState>,
    Path(sim_id): Path<String>,
) -> Result<StatusCode, StatusCode> {
    let mut sims = state.write().await;

    if sims.remove(&sim_id).is_some() {
        info!("Destroyed simulation {}", sim_id);
        Ok(StatusCode::NO_CONTENT)
    } else {
        warn!("Attempted to destroy non-existent simulation {}", sim_id);
        Err(StatusCode::NOT_FOUND)
    }
}

// Helper functions
fn simulation_state_to_response(sim_id: String, sim: &Simulation) -> SimulationStateResponse {
    let bodies = sim
        .get_all_bodies()
        .iter()
        .map(|(id, state)| BodyStateResponse {
            id: id.clone(),
            position: state.position,
            orientation: state.orientation,
            velocity: state.velocity,
            angular_velocity: state.angular_velocity,
            contacts: state.contacts.clone(),
        })
        .collect();

    SimulationStateResponse {
        sim_id,
        time: sim.time,
        bodies,
    }
}
