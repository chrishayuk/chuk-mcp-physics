# Architecture Overview

## System Design

```
┌──────────────────────────────────────────────────────────────┐
│                      LLM Client Layer                         │
│  (Claude Desktop, Claude Code, Custom MCP Clients)           │
└──────────────────────┬───────────────────────────────────────┘
                       │
                       │ MCP Protocol (stdio/HTTP)
                       │
┌──────────────────────▼───────────────────────────────────────┐
│               chuk-mcp-physics Server                        │
│                  (Python + FastMCP)                          │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │              11 MCP Tools                          │    │
│  │  • calculate_projectile_motion                     │    │
│  │  • check_collision                                 │    │
│  │  • calculate_force, kinetic_energy, momentum       │    │
│  │  • create_simulation, add_rigid_body, step, etc.   │    │
│  └────────────────────────────────────────────────────┘    │
│                       │                                      │
│  ┌────────────────────▼─────────────────────┐              │
│  │        Provider Factory                   │              │
│  │   (Routes to correct provider)            │              │
│  └────────────┬─────────────────┬────────────┘              │
│               │                 │                            │
│      ┌────────▼─────┐  ┌───────▼────────┐                 │
│      │  Analytic    │  │  Rapier        │                 │
│      │  Provider    │  │  Provider      │                 │
│      │  (Built-in)  │  │  (HTTP Client) │                 │
│      └──────────────┘  └───────┬────────┘                 │
└─────────────────────────────────┼──────────────────────────┘
                                  │
                                  │ REST API (HTTP/JSON)
                                  │
                    ┌─────────────▼──────────────┐
                    │  rapier-physics-service    │
                    │     (Rust + Axum)          │
                    │                            │
                    │  ┌──────────────────────┐ │
                    │  │  Rapier3D Engine     │ │
                    │  │  Physics Pipeline    │ │
                    │  │  Collision Detection │ │
                    │  │  Rigid Body Dynamics │ │
                    │  └──────────────────────┘ │
                    └────────────────────────────┘
```

## Component Breakdown

### 1. MCP Server (Python)
**Location:** `/src/chuk_mcp_physics/`
**Technology:** Python 3.11+, chuk-mcp-server, Pydantic
**Responsibilities:**
- Expose 11 physics tools via MCP protocol
- Route requests to appropriate provider
- Handle type validation (Pydantic models)
- Manage provider lifecycle

**Deployment:**
- Fly.io app: `chuk-mcp-physics`
- Minimal CPU/memory requirements
- Auto-sleep capable

### 2. Analytic Provider (Python)
**Location:** `/src/chuk_mcp_physics/providers/analytic.py`
**Technology:** NumPy, pure Python math
**Responsibilities:**
- Instant physics calculations
- Projectile motion (kinematic equations)
- Sphere-sphere collision detection
- Force, energy, momentum calculations

**Characteristics:**
- ✅ Zero latency (no network calls)
- ✅ No external dependencies
- ✅ Perfect for education and simple scenarios
- ❌ Limited to spherical objects
- ❌ No multi-body dynamics

### 3. Rapier Provider (Python HTTP Client)
**Location:** `/src/chuk_mcp_physics/providers/rapier.py`
**Technology:** httpx, async HTTP client
**Responsibilities:**
- Delegate to Rapier service via REST API
- Manage simulation lifecycle
- Handle retries and timeouts
- Falls back to analytic for simple calculations

**Configuration:**
```python
RAPIER_SERVICE_URL = os.getenv("RAPIER_SERVICE_URL", "http://localhost:9000")
TIMEOUT = 30.0
MAX_RETRIES = 3
```

### 4. Rapier Physics Service (Rust)
**Location:** `/rapier-service/`
**Technology:** Rust, Rapier3D, Axum, Tokio
**Responsibilities:**
- Full rigid-body physics simulation
- Collision detection (all shapes)
- Constraints, joints, friction, restitution
- Trajectory recording
- Multi-body dynamics

**API Endpoints:**
```
POST   /simulations                          Create simulation
GET    /simulations/:id/state                Get state
DELETE /simulations/:id                      Destroy
POST   /simulations/:id/bodies               Add rigid body
POST   /simulations/:id/step                 Step simulation
POST   /simulations/:id/bodies/:id/trajectory Record trajectory
GET    /health                               Health check
```

**Deployment:**
- Fly.io app: `chuk-rapier-physics`
- CPU-optimized instances
- Independent scaling from MCP server

## Data Flow

### Example 1: Simple Projectile (Analytic)

```
User: "How far does a ball go if thrown at 20 m/s at 45°?"
  │
  ▼
MCP Client → calculate_projectile_motion(20, 45)
  │
  ▼
MCP Server → ProviderFactory.get_provider("projectile_motion")
  │           └─> Returns AnalyticProvider (hardcoded for analytic tools)
  ▼
AnalyticProvider.calculate_projectile_motion()
  │  • Uses kinematic equations
  │  • Calculates max_height, range, time_of_flight
  │  • Generates trajectory points
  ▼
Response: {max_height: 10.2m, range: 40.8m, time: 2.87s, trajectory: [...]}
  │
  ▼
User sees answer instantly
```

**Latency:** < 10ms (pure computation)

### Example 2: Complex Simulation (Rapier)

```
User: "Simulate 10 boxes falling and stacking"
  │
  ▼
MCP Client → create_simulation({gravity: [0, -9.81, 0]})
  │
  ▼
MCP Server → ProviderFactory.get_provider("simulations")
  │           └─> Returns RapierProvider (configured via env/yaml)
  ▼
RapierProvider → HTTP POST to rapier-physics-service
  │                URL: https://chuk-rapier-physics.fly.dev/simulations
  ▼
Rapier Service:
  │  • Creates Rapier physics world
  │  • Initializes gravity vector
  │  • Returns simulation ID
  ▼
Response: {sim_id: "abc-123", config: {...}}
  │
  ▼
For each box:
  MCP Client → add_rigid_body(sim_id, {shape: "box", mass: 1, ...})
    ▼
  RapierProvider → HTTP POST /simulations/abc-123/bodies
    ▼
  Rapier Service:
    • Creates RigidBody + Collider
    • Adds to physics pipeline
    ▼
  Response: {body_id: "box_0"}

Then:
  MCP Client → step_simulation(sim_id, steps=1000)
    ▼
  RapierProvider → HTTP POST /simulations/abc-123/step
    ▼
  Rapier Service:
    • Runs physics pipeline 1000 times
    • Detects collisions
    • Updates velocities, positions
    • Resolves contacts
    ▼
  Response: {sim_id, time: 16.0, bodies: [{id, position, velocity, ...}, ...]}
```

**Latency:**
- Network: 20-100ms per request (Fly.io region dependent)
- Computation: 1-50ms for 1000 steps
- Total: ~100-200ms for complete simulation

## Provider Selection Logic

```python
# File: src/chuk_mcp_physics/providers/factory.py

def get_provider_for_tool(tool_name: str) -> PhysicsProvider:
    """Route tool to correct provider."""

    # Analytic tools ALWAYS use analytic provider
    if tool_name in ["projectile_motion", "collision_check",
                     "force", "kinetic_energy", "momentum"]:
        return AnalyticProvider()

    # Simulation tools use configured provider
    if tool_name == "simulations":
        provider_type = ProviderConfig.SIMULATION_PROVIDER  # From env/yaml

        if provider_type == "rapier":
            return RapierProvider(
                service_url=RapierConfig.SERVICE_URL,
                timeout=RapierConfig.TIMEOUT
            )
        else:
            return AnalyticProvider()  # Fallback
```

## Configuration Hierarchy

1. **Environment Variables** (highest priority)
   ```bash
   export PHYSICS_PROVIDER=rapier
   export RAPIER_SERVICE_URL=https://chuk-rapier-physics.fly.dev
   export RAPIER_TIMEOUT=30.0
   ```

2. **YAML Configuration**
   ```yaml
   # physics.yaml
   default_provider: rapier
   rapier:
     service_url: https://chuk-rapier-physics.fly.dev
   ```

3. **Hardcoded Defaults** (lowest priority)
   ```python
   DEFAULT_PROVIDER = "analytic"
   SERVICE_URL = "http://localhost:9000"
   TIMEOUT = 30.0
   ```

## Deployment Topologies

### Development
```
Local Machine
├── MCP Server (localhost:stdio)
└── Rapier Service (localhost:9000)
```

### Production Option 1: Separate Apps (Recommended)
```
Fly.io Region: sjc (San Jose)
├── chuk-mcp-physics.fly.dev (Python MCP)
│   └── RAPIER_SERVICE_URL=https://chuk-rapier-physics.fly.dev
└── chuk-rapier-physics.fly.dev (Rust Service)
```

### Production Option 2: Monolith
```
Fly.io Region: sjc
└── chuk-mcp-physics.fly.dev
    ├── Python MCP Server (process 1)
    └── Rapier Service (process 2)
```

## Performance Characteristics

| Operation | Provider | Latency | Accuracy | Scalability |
|-----------|----------|---------|----------|-------------|
| Projectile Motion | Analytic | <1ms | Exact | ∞ |
| Simple Collision | Analytic | <1ms | Exact (spheres) | ∞ |
| Force/Energy/Momentum | Analytic | <1ms | Exact | ∞ |
| Rigid Body Sim (10 bodies) | Rapier | ~50ms | High | 100+ req/s |
| Rigid Body Sim (100 bodies) | Rapier | ~200ms | High | 20+ req/s |
| Complex Constraints | Rapier | ~500ms | High | 5+ req/s |

## Scaling Strategy

### Horizontal Scaling (Rapier Service)
```bash
# Scale Rapier for physics workloads
fly scale count 3 -a chuk-rapier-physics

# Auto-scale based on load
fly autoscale set min=1 max=10 -a chuk-rapier-physics
```

### Vertical Scaling
```bash
# Upgrade Rapier to performance CPU
fly scale vm performance-1x -a chuk-rapier-physics

# MCP server stays minimal
fly scale vm shared-cpu-1x -a chuk-mcp-physics
```

### Caching (Future Enhancement)
- Cache simulation results by input hash
- Redis for shared cache across instances
- TTL-based invalidation

## Security Considerations

### Internal Network (Recommended)
```toml
# rapier-service/fly.toml
# Remove public HTTP ports, use 6PN only
[services]
  internal_port = 9000
  # No [[services.ports]] = private to Fly.io network
```

```bash
# MCP server uses internal DNS
fly secrets set RAPIER_SERVICE_URL=http://chuk-rapier-physics.internal:9000
```

### Authentication (Future)
- API keys for Rapier service
- Rate limiting per client
- Request validation

## Monitoring & Observability

### Metrics to Track
- **MCP Server:**
  - Tool invocation rate
  - Provider selection distribution
  - Error rates per tool

- **Rapier Service:**
  - Request latency (p50, p95, p99)
  - Simulation steps/second
  - Active simulations count
  - Memory usage per simulation
  - CPU utilization

### Logging
```bash
# Aggregate logs from both services
fly logs -a chuk-mcp-physics &
fly logs -a chuk-rapier-physics
```

### Health Checks
- MCP Server: Process-based (stdio)
- Rapier Service: HTTP `/health` endpoint

## Future Enhancements

1. **WebSocket Support**
   - Real-time simulation updates
   - Streaming trajectory data
   - Lower latency for interactive use

2. **Simulation Persistence**
   - Save/load simulations to database
   - Resume long-running simulations
   - Share simulation states

3. **Advanced Physics**
   - Soft body dynamics
   - Fluid simulation
   - Cloth simulation
   - Particle systems

4. **Optimization**
   - Parallel simulation execution
   - GPU acceleration
   - Simulation result caching
   - Connection pooling

5. **Observability**
   - Prometheus metrics export
   - Distributed tracing (OpenTelemetry)
   - Performance profiling
   - Cost tracking per simulation
