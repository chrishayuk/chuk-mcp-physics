# Rapier Physics Service

This document specifies the Rust microservice that provides physics simulation using Rapier.

## Architecture

```text
┌─────────────────┐
│   LLM Client    │
└────────┬────────┘
         │ MCP Protocol
         ▼
┌─────────────────┐
│ Physics MCP     │ (Python, chuk-mcp-server)
│   Server        │ - Tool definitions
└────────┬────────┘ - Session management
         │ HTTP/JSON  - Response formatting
         ▼
┌─────────────────┐
│ Rapier Service  │ (Rust, Axum)
│                 │ - Simulation management
└────────┬────────┘ - Physics stepping
         │            - State tracking
         ▼
┌─────────────────┐
│   rapier3d      │ (Rust library)
│                 │ - Rigid body dynamics
└─────────────────┘ - Collision detection
```

## Why Separate Service?

1. **Language barriers**: Rapier is Rust, MCP server is Python
2. **Performance**: Rust for compute-heavy physics, Python for API/MCP
3. **Scalability**: Can run multiple Rapier instances behind load balancer
4. **Simplicity**: Clean JSON API, no PyO3/FFI complexity

## API Specification

Base URL: `http://localhost:9000` (configurable)

### 1. Create Simulation

```http
POST /simulations
Content-Type: application/json

{
  "gravity": [0.0, -9.81, 0.0],
  "dimensions": 3,
  "dt": 0.016,
  "integrator": "verlet"
}
```

**Response (201 Created):**
```json
{
  "sim_id": "sim_01HXYZ...",
  "config": {
    "gravity": [0.0, -9.81, 0.0],
    "dimensions": 3,
    "dt": 0.016,
    "integrator": "verlet"
  }
}
```

### 2. Add Rigid Body

```http
POST /simulations/{sim_id}/bodies
Content-Type: application/json

{
  "id": "car_body",
  "kind": "dynamic",
  "shape": "box",
  "size": [2.0, 0.5, 4.0],
  "mass": 1200.0,
  "position": [0.0, 1.0, 0.0],
  "orientation": [0.0, 0.0, 0.0, 1.0],
  "velocity": [0.0, 0.0, 0.0],
  "angular_velocity": [0.0, 0.0, 0.0],
  "restitution": 0.1,
  "friction": 1.0,
  "is_sensor": false
}
```

**Response (201 Created):**
```json
{
  "body_id": "car_body"
}
```

**Shape Types:**
- `box`: size = [width, height, depth]
- `sphere`: radius = float
- `capsule`: radius, half_height
- `cylinder`: radius, half_height
- `plane`: normal = [x, y, z], offset = float

### 3. Step Simulation

```http
POST /simulations/{sim_id}/step
Content-Type: application/json

{
  "steps": 600,
  "dt": 0.016
}
```

**Response (200 OK):**
```json
{
  "sim_id": "sim_01HXYZ...",
  "time": 9.6,
  "bodies": [
    {
      "id": "car_body",
      "position": [10.2, 0.5, 35.1],
      "orientation": [0.0, 0.1, 0.0, 0.99],
      "velocity": [12.1, 0.0, 38.0],
      "angular_velocity": [0.0, 0.1, 0.0],
      "contacts": [
        {
          "with_body": "ground",
          "point": [10.2, 0.0, 35.1],
          "normal": [0, 1, 0],
          "impulse": 3200.0,
          "distance": -0.01
        }
      ]
    }
  ]
}
```

### 4. Get Simulation State

```http
GET /simulations/{sim_id}/state
```

**Response (200 OK):**
Same as step response, but doesn't advance simulation.

### 5. Record Trajectory

```http
POST /simulations/{sim_id}/bodies/{body_id}/trajectory
Content-Type: application/json

{
  "steps": 1000,
  "dt": 0.016
}
```

**Response (200 OK):**
```json
{
  "body_id": "car_body",
  "frames": [
    {
      "time": 0.0,
      "position": [0.0, 1.0, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "velocity": [0.0, 0.0, 0.0]
    },
    {
      "time": 0.016,
      "position": [0.0, 0.998, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "velocity": [0.0, -0.157, 0.0]
    }
  ],
  "total_time": 16.0,
  "num_frames": 1000
}
```

### 6. Apply Force/Impulse

```http
POST /simulations/{sim_id}/bodies/{body_id}/impulse
Content-Type: application/json

{
  "impulse": [0.0, 0.0, 5000.0],
  "point": [0.0, 0.5, 0.0]
}
```

**Response (204 No Content)**

### 7. Destroy Simulation

```http
DELETE /simulations/{sim_id}
```

**Response (204 No Content)**

## Error Responses

```json
{
  "error": "simulation_not_found",
  "message": "Simulation 'sim_abc' does not exist",
  "sim_id": "sim_abc"
}
```

Common error codes:
- `400 Bad Request`: Invalid input
- `404 Not Found`: Simulation or body not found
- `500 Internal Server Error`: Physics engine error

## Implementation Guide (Rust)

### Dependencies

```toml
[dependencies]
axum = "0.7"
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
rapier3d = "0.19"
uuid = { version = "1", features = ["v7"] }
```

### Core Structs

```rust
use rapier3d::prelude::*;
use std::collections::HashMap;

pub struct Simulation {
    rigid_bodies: RigidBodySet,
    colliders: ColliderSet,
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    user_id_to_handle: HashMap<String, RigidBodyHandle>,
    current_time: f32,
}

pub struct SimulationManager {
    simulations: HashMap<String, Simulation>,
}
```

### Example Handler

```rust
async fn step_simulation(
    Path(sim_id): Path<String>,
    Json(request): Json<StepRequest>,
) -> Result<Json<StepResponse>, StatusCode> {
    let mut manager = SIMULATION_MANAGER.lock().await;
    let sim = manager.simulations
        .get_mut(&sim_id)
        .ok_or(StatusCode::NOT_FOUND)?;

    for _ in 0..request.steps {
        sim.physics_pipeline.step(
            &sim.gravity,
            &sim.integration_parameters,
            &mut sim.island_manager,
            &mut sim.broad_phase,
            &mut sim.narrow_phase,
            &mut sim.rigid_bodies,
            &mut sim.colliders,
            &mut sim.impulse_joints,
            &mut sim.multibody_joints,
            &mut sim.ccd_solver,
            None,
            &(),
            &(),
        );
        sim.current_time += request.dt.unwrap_or(sim.integration_parameters.dt);
    }

    let response = build_state_response(sim);
    Ok(Json(response))
}
```

## Deployment

### Local Development
```bash
cd rapier-service
cargo run --release
# Listens on http://localhost:9000
```

### Docker
```dockerfile
FROM rust:1.75 as builder
WORKDIR /app
COPY Cargo.* ./
COPY src ./src
RUN cargo build --release

FROM debian:bookworm-slim
COPY --from=builder /app/target/release/rapier-service /usr/local/bin/
EXPOSE 9000
CMD ["rapier-service"]
```

### Fly.io
```bash
fly launch --name chuk-rapier-service
fly scale memory 512
fly deploy
```

## Performance Considerations

1. **Memory**: Each simulation ~1-10 MB depending on body count
2. **CPU**: Single-threaded per simulation (Rapier design)
3. **Timeout**: Large step counts may timeout - limit to 10,000 steps
4. **Cleanup**: Implement TTL for inactive simulations (e.g., 1 hour)

## Future Enhancements

- [ ] Joints (hinges, ball joints, fixed)
- [ ] Sensors and triggers
- [ ] Raycasting
- [ ] WASM build for client-side physics
- [ ] 2D variant using `rapier2d`
- [ ] Binary protocol (msgpack/protobuf) for large responses

## Testing

```bash
# Create simulation
curl -X POST http://localhost:9000/simulations \
  -H "Content-Type: application/json" \
  -d '{"gravity": [0, -9.81, 0], "dimensions": 3, "dt": 0.016}'

# Response: {"sim_id": "sim_..."}

# Add ground plane
curl -X POST http://localhost:9000/simulations/sim_.../bodies \
  -H "Content-Type: application/json" \
  -d '{
    "id": "ground",
    "kind": "static",
    "shape": "plane",
    "normal": [0, 1, 0],
    "offset": 0
  }'

# Add falling box
curl -X POST http://localhost:9000/simulations/sim_.../bodies \
  -H "Content-Type: application/json" \
  -d '{
    "id": "box",
    "kind": "dynamic",
    "shape": "box",
    "size": [1, 1, 1],
    "mass": 10,
    "position": [0, 5, 0]
  }'

# Step simulation
curl -X POST http://localhost:9000/simulations/sim_.../step \
  -H "Content-Type: application/json" \
  -d '{"steps": 100}'
```

## References

- [Rapier Documentation](https://rapier.rs/)
- [Rapier Rust API](https://docs.rs/rapier3d/)
- [Axum Web Framework](https://docs.rs/axum/)
