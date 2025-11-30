# Physics MCP Server

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)

**MCP server for physics simulations and calculations using Rapier physics engine**

Provides LLMs with physics superpowers: ballistic calculations, collision prediction, rigid-body simulations, and trajectory recording for 3D visualization.

## 🚀 Quick Start (30 seconds)

```bash
# Try it instantly with uvx (no installation needed)
uvx chuk-mcp-physics

# Or with the public Rapier service for simulations
RAPIER_SERVICE_URL=https://rapier.chukai.io uvx chuk-mcp-physics
```

**For Claude Desktop:** Add to your config file:
```json
{
  "mcpServers": {
    "physics": {
      "command": "uvx",
      "args": ["chuk-mcp-physics"],
      "env": {
        "RAPIER_SERVICE_URL": "https://rapier.chukai.io"
      }
    }
  }
}
```

---

## 🎯 Use Cases

### 1. **Interactive Physics Education**
**LLM as Physics Tutor**
```
User: "If I throw a ball at 20 m/s at 45°, how far will it go?"
LLM: [calls calculate_projectile_motion]
     "The ball will travel 40.8 meters and reach a maximum height of 10.2 meters..."
     [generates visualization with trajectory points]
```

**Real-World Problem Solving**
- Students ask physics questions in natural language
- LLM calculates exact answers using the MCP tools
- Can generate trajectory plots, force diagrams, energy graphs
- Interactive "what-if" scenarios: "What if gravity was half?"

---

### 2. **Game Development & Prototyping**
**Ballistics Design**
```
User: "I'm designing a cannon in my game. Initial velocity 50 m/s, 30° angle.
       Will it clear a 15m wall at 80m distance?"
LLM: [calls calculate_projectile_motion]
     "At 80m, the projectile is at 18.6m height - it WILL clear the wall.
      It lands at 110.9m range."
```

**Collision Detection**
```
User: "Two spaceships: Ship A at (0,0,0) moving at (10,0,0) m/s, Ship B at (100,5,0)
       moving at (-8,0,0) m/s. Will they collide?"
LLM: [calls check_collision]
     "Yes, collision in 5.3 seconds at position (53.0, 2.65, 0.0) with impact speed 18 m/s"
```

**Rapid Iteration**
- Test different launch angles, speeds, masses without coding
- Verify collision logic before implementation
- Generate realistic physics data for procedural content

---

### 3. **3D Visualization & Animation (React Three Fiber)**
**Automated Animation Data Generation**
```
User: "Create a realistic basketball shot animation - 7m throw into 3m high basket"
LLM: [calls calculate_projectile_motion with solved angle]
     [returns trajectory_points array]
     "Here's the trajectory data for R3F. The ball needs 52° launch angle..."

     <mesh position={interpolate(trajectory)}>
       <sphereGeometry args={[0.12]} />
     </mesh>
```

**Rigid-Body Simulations**
```
User: "Simulate 10 boxes falling and stacking in a pile"
LLM: [calls create_simulation]
     [calls add_rigid_body for ground + 10 boxes with random positions]
     [calls record_trajectory for each box]
     "Here are 10 trajectory arrays for your R3F scene, ready to use..."
```

**Use Cases:**
- Product visualizations (dropping phones, bouncing balls)
- Architectural collapse simulations (building demolition previews)
- Sports analytics visualizations (ball trajectories, collision analysis)
- Sci-fi effects (asteroid fields, debris clouds)

---

### 4. **Engineering & Design Analysis**
**Vehicle Crash Prediction**
```
User: "Two cars: Car A (1500kg) at 30 m/s, Car B (1200kg) at 25 m/s approaching.
       Distance 100m. When do they collide and what's the impact energy?"
LLM: [calls check_collision for timing]
     [calls calculate_kinetic_energy for both cars]
     [calls calculate_momentum for momentum analysis]
     "Collision in 1.82 seconds. Total kinetic energy: 1,050,000 J..."
```

**Safety Analysis**
- Calculate impact forces for crash test scenarios
- Predict collision times for autonomous vehicle planning
- Analyze momentum transfer in industrial equipment

**Structural Testing (with Rapier simulations)**
- Simulate falling objects hitting structures
- Test load-bearing capacity under dynamic loads
- Model chain reactions (domino effects)

---

### 5. **Sports & Athletics**
**Performance Optimization**
```
User: "What's the optimal angle to throw a javelin at 25 m/s for maximum distance?"
LLM: [calls calculate_projectile_motion at multiple angles]
     "45° gives maximum range of 63.7 meters on flat ground.
      However, with 2m release height, 43° is optimal at 64.9m..."
```

**Shot Analysis**
- Basketball: arc optimization, free throw angles
- Golf: ball trajectory, wind effects (with manual adjustments)
- Baseball: pitch speed vs. distance calculations
- Track & field: javelin, shot put, long jump optimization

---

### 6. **Astrophysics & Space Exploration**
**Orbital Mechanics (Simplified)**
```
User: "Two asteroids on collision course. A: 500m radius at (0,0,0) moving 15 km/s.
       B: 300m radius at (100km, 2km, 0) moving -12 km/s. Impact prediction?"
LLM: [calls check_collision with appropriate units]
     "Collision in 3.47 seconds at closest approach distance 650m.
      They will NOT collide - miss distance is 150m..."
```

**Applications:**
- Asteroid impact prediction
- Satellite collision avoidance
- Debris field analysis
- Launch trajectory planning (simplified cases)

---

### 7. **Film & VFX Pre-visualization**
**Stunt Planning**
```
User: "Car jumps off 3m ramp at 25 m/s, 20° angle. How far does it fly and where does it land?"
LLM: [calls calculate_projectile_motion]
     "Airtime: 1.75 seconds, landing at 42.9m horizontal distance,
      impact speed 26.3 m/s. Recommend crashmat at 40-45m mark..."
```

**Destruction Sequences**
- Building collapses with rigid-body sim
- Explosion debris trajectories
- Vehicle stunts and crashes
- Realistic object interactions

---

### 8. **Military & Defense (Training/Education)**
**Ballistics Training**
```
User: "Artillery shell: muzzle velocity 800 m/s, 45° elevation. Range and time of flight?"
LLM: [calls calculate_projectile_motion]
     "Range: 65.3 km, flight time: 115.5 seconds, max altitude: 16.3 km"
```

**Collision Avoidance**
- Projectile trajectory analysis
- Impact point prediction
- Intercept course calculations

---

### 9. **Robotics & Automation**
**Path Planning**
```
User: "Robot arm needs to toss part into bin 2m away, 0.5m higher.
       What velocity is needed?"
LLM: [reverse-calculates using projectile_motion multiple times]
     "Minimum velocity: 4.7 m/s at 38° angle. Recommend 5.0 m/s for safety margin..."
```

**Collision Detection**
- Multi-robot coordination
- Object catching/throwing
- Assembly line optimization

---

### 10. **Data Science & Research**
**Physics Simulations for ML Training Data**
```python
# Generate thousands of collision scenarios for ML model training
for i in range(10000):
    result = await check_collision(random_params())
    training_data.append({
        'features': params,
        'label': result.will_collide,
        'impact_time': result.collision_time
    })
```

**Use Cases:**
- Generate labeled physics data for ML models
- Validate physics-informed neural networks
- Test scientific hypotheses with rapid iteration
- Monte Carlo simulations (vary parameters, aggregate results)

---

## 🚀 Real-World Example Workflows

### Workflow 1: Basketball Shot Optimizer
```
1. User: "I'm 2m tall shooting from free-throw line (4.6m). Basket is 3.05m high.
          What's the minimum velocity needed?"

2. LLM calls calculate_projectile_motion with varying velocities
   - Try v=5 m/s → doesn't reach
   - Try v=7 m/s → reaches
   - Binary search finds minimum: v=6.2 m/s at 52° angle

3. LLM: "Minimum velocity is 6.2 m/s at 52° launch angle.
         For comfortable margin, use 7.0 m/s (typical free throw speed).
         Here's the trajectory visualization..."
```

### Workflow 2: Car Crash Investigation
```
1. User: "Analyze accident: Car A (1500kg) skid marks 30m, Car B (1200kg) skid marks 25m.
          Coefficient of friction 0.7. What were impact speeds?"

2. LLM:
   - Calculates deceleration from friction: a = μg = 0.7 × 9.81 = 6.87 m/s²
   - Uses v² = 2ad to find velocities
   - Calls calculate_kinetic_energy for both cars
   - Calls calculate_momentum for momentum analysis

3. LLM: "Car A impact speed: ~20.3 m/s (45 mph), Car B: ~18.5 m/s (41 mph).
         Total kinetic energy at impact: 513,000 J. Here's the force analysis..."
```

### Workflow 3: Game Level Design with Physics Simulation
```
1. User: "Create a Rube Goldberg machine: ball rolls down ramp, hits dominos,
          dominos knock ball into basket"

2. LLM:
   - Calls create_simulation(gravity_y=-9.81)
   - Adds ground plane (static)
   - Adds ramp (static, angled)
   - Adds ball (dynamic, sphere, position at ramp top)
   - Adds 10 dominos (dynamic boxes in a line)
   - Adds basket (static)
   - Calls step_simulation(steps=1000)
   - Analyzes contacts to verify chain reaction
   - Calls record_trajectory for each piece

3. LLM: "Simulation complete! Ball triggers all dominos successfully.
         Here are the trajectories for R3F visualization.
         Domino #3 falls at t=1.2s, domino #7 at t=2.1s..."
```

### Workflow 4: Satellite Collision Warning
```
1. User: "Satellite A: position (6700km, 0, 0), velocity (0, 7.5km/s, 0)
          Satellite B: position (6650km, 50km, 0), velocity (0, 7.6km/s, 0.1km/s)
          Collision risk?"

2. LLM:
   - Calls check_collision with satellite data
   - Analyzes closest approach

3. LLM: "No collision. Closest approach: 48.3 km at t=412 seconds.
         Satellites are in similar orbits but safe separation.
         Recommend: monitor as orbits may precess over time."
```

---

## 🎨 Visualization Integration

### React Three Fiber (R3F) Example
```tsx
function PhysicsAnimation() {
  const [trajectory, setTrajectory] = useState([]);

  useEffect(() => {
    // LLM generated this trajectory data via MCP
    fetch('/api/mcp/record_trajectory', {
      body: JSON.stringify({
        sim_id: "sim_xyz",
        body_id: "ball",
        steps: 300
      })
    }).then(res => setTrajectory(res.frames));
  }, []);

  return (
    <Canvas>
      <AnimatedBall trajectory={trajectory} />
      <Ground />
    </Canvas>
  );
}

function AnimatedBall({ trajectory }) {
  const ref = useRef();

  useFrame((state) => {
    const t = state.clock.getElapsedTime();
    const frame = trajectory[Math.floor(t / 0.016) % trajectory.length];
    if (frame && ref.current) {
      ref.current.position.fromArray(frame.position);
      ref.current.quaternion.fromArray(frame.orientation);
    }
  });

  return (
    <mesh ref={ref}>
      <sphereGeometry args={[0.5]} />
      <meshStandardMaterial color="orange" />
    </mesh>
  );
}
```

---

## 🛠️ Installation

### Prerequisites
- Python 3.11+
- For simulations: Rapier service (see [RAPIER_SERVICE.md](RAPIER_SERVICE.md))
  - **Public service available at: https://rapier.chukai.io**
  - Or run locally with Docker (see below)

### Quick Start with uvx (Recommended)

The fastest way to try chuk-mcp-physics without installation:

```bash
# Run directly with uvx (no installation needed)
uvx chuk-mcp-physics

# With environment variables
uvx --with chuk-mcp-physics chuk-mcp-physics
```

### Installation Methods

#### Option 1: Install from PyPI (Recommended)

```bash
# Install globally
pip install chuk-mcp-physics

# Or with pipx (isolated environment)
pipx install chuk-mcp-physics

# Run the server
chuk-mcp-physics

# Or via python module
python -m chuk_mcp_physics.server
```

#### Option 2: Install from Source

```bash
# Clone repository
git clone https://github.com/yourusername/chuk-mcp-physics
cd chuk-mcp-physics

# Install in development mode
make dev-install

# Run the server
chuk-mcp-physics
```

### With Claude Desktop

Add to your Claude Desktop config (`~/Library/Application Support/Claude/claude_desktop_config.json` on macOS):

#### Option 1: Using uvx (Recommended - No Installation Required)

```json
{
  "mcpServers": {
    "physics": {
      "command": "uvx",
      "args": ["chuk-mcp-physics"],
      "env": {
        "PHYSICS_PROVIDER": "rapier",
        "RAPIER_SERVICE_URL": "https://rapier.chukai.io"
      }
    }
  }
}
```

#### Option 2: Using Installed Package

```json
{
  "mcpServers": {
    "physics": {
      "command": "python",
      "args": ["-m", "chuk_mcp_physics.server"],
      "env": {
        "PHYSICS_PROVIDER": "rapier",
        "RAPIER_SERVICE_URL": "https://rapier.chukai.io"
      }
    }
  }
}
```

#### Option 3: Analytic Only (No External Service)

```json
{
  "mcpServers": {
    "physics": {
      "command": "uvx",
      "args": ["chuk-mcp-physics"],
      "env": {
        "PHYSICS_PROVIDER": "analytic"
      }
    }
  }
}
```

---

## 📖 Available Tools

### Analytic Calculations (No Rapier Required)

| Tool | Description | Use Case |
|------|-------------|----------|
| `calculate_projectile_motion` | Ballistic trajectory calculation | Cannon shots, sports, games |
| `check_collision` | Predict if two objects will collide | Crash avoidance, game logic |
| `calculate_force` | F = ma calculations | Engineering, physics ed |
| `calculate_kinetic_energy` | KE = ½mv² | Impact analysis, safety |
| `calculate_momentum` | p = mv | Collision analysis, rockets |

### Simulations (Requires Rapier Service)

| Tool | Description | Use Case |
|------|-------------|----------|
| `create_simulation` | Initialize physics world | Start any simulation |
| `add_rigid_body` | Add objects to simulation | Build scene |
| `step_simulation` | Advance physics | Run simulation |
| `record_trajectory` | Record motion over time | R3F animations |
| `destroy_simulation` | Cleanup resources | End simulation |

---

## 🎓 Example Conversations

### Projectile Motion
```
User: A golf ball is hit at 70 m/s at 15 degrees. How far does it go and what's the flight time?

LLM: [calls calculate_projectile_motion(70, 15)]

     Range ≈ 249.7 m
     Time of flight ≈ 3.694 s
     Maximum height ≈ 16.73 m

     Here are 50 trajectory points for visualization...
```

```
User: What angle gives maximum range for a cannonball fired at 100 m/s?

LLM: [calls calculate_projectile_motion multiple times with different angles]

     The optimal angle is 45°, which gives:
     - Maximum range: 1019.4 m
     - Flight time: 14.4 s
     - Peak height: 254.8 m
```

### Collision Detection
```
User: Two asteroids: one at (0,0,0) moving at 1000 m/s in x direction,
      another at (50000, 100, 0) moving at -800 m/s in x. Will they collide?

LLM: [calls check_collision with both objects]

     Yes, they will collide!
     - Collision time: ~27.8 seconds
     - Impact location: (27,778, 100, 0)
     - Impact speed: 1,800 m/s
     - This would be a devastating impact given the closing speed.
```

```
User: Two cars 500m apart, one at 60 mph, other at 45 mph heading toward each other.
      When do they collide?

LLM: [calls check_collision]

     Collision in 10.7 seconds at position 250m from car 1's starting point.
     Combined approach speed is 105 mph (46.9 m/s).
```

### Force & Energy Calculations
```
User: What force is needed to accelerate a 1500 kg car at 3 m/s²?

LLM: [calls calculate_force(1500, 3)]

     Force required: 4,500 N (Newtons)
     This is about 1,011 pounds of force.
```

```
User: What's the kinetic energy of a 2000 kg car at 30 m/s (about 67 mph)?

LLM: [calls calculate_kinetic_energy(2000, 30)]

     Kinetic energy: 900,000 J (Joules)
     That's 900 kJ or 0.25 kWh of energy.
     For perspective, this is why car crashes at highway speeds are so dangerous.
```

### Momentum Analysis
```
User: Calculate momentum of a 70 kg runner at 10 m/s

LLM: [calls calculate_momentum(70, 10)]

     Momentum: 700 kg⋅m/s
     Direction: same as velocity vector
```

### Complex Multi-Step Problems
```
User: I'm designing a water fountain. Water shoots up at 15 m/s.
      How high does it go and when does it come back down?

LLM: [calls calculate_projectile_motion(15, 90)]

     The water will:
     - Rise to maximum height: 11.47 m
     - Take 1.53 seconds to reach peak
     - Return to ground level after 3.06 seconds total
     - Have the same 15 m/s speed when it lands (just downward)
```

```
User: A basketball player shoots from 2m high at 7 m/s toward a hoop
      3.05m high and 4.6m away. What angles work?

LLM: [calls calculate_projectile_motion with various angles]

     Two possible angles work:
     1. Low arc: 38° (faster, flatter shot)
     2. High arc: 52° (slower, higher shot)

     Most players use 50-55° for better chance of going in.
```

### Rigid Body Simulations
```
User: Create a simulation of a ball dropping from 10 meters and bouncing

LLM: [calls create_simulation(gravity_y=-9.81)]
     [calls add_rigid_body for ground plane (static)]
     [calls add_rigid_body for ball (dynamic, sphere, position=(0,10,0))]
     [calls record_trajectory(steps=200)]

     Simulation complete! The ball:
     - Hits ground at t=1.43s with velocity 14.0 m/s
     - Bounces to 7.5m (with restitution=0.8)
     - Makes 5 bounces before settling

     Here's the full trajectory data for R3F visualization...
```

```
User: Simulate 5 boxes stacked, then another box hits them from the side

LLM: [calls create_simulation]
     [calls add_rigid_body for ground]
     [calls add_rigid_body 5 times for stacked boxes]
     [calls add_rigid_body for projectile box with velocity]
     [calls step_simulation(300)]
     [calls record_trajectory for each box]

     The collision causes a realistic toppling effect!
     Boxes 1-2 fall left, boxes 3-5 scatter right.
     Peak chaos at t=0.8s. All settled by t=3.2s.

     Full trajectory data ready for 3D visualization...
```

---

## 💡 Try These Prompts

Copy and paste these into your LLM chat to see the physics tools in action:

### Projectile Motion
- `A golf ball is hit at 70 m/s at 15 degrees. How far does it go and what's the flight time?`
- `What angle gives maximum range for a cannonball fired at 100 m/s?`
- `If I throw a javelin at 28 m/s from 2 meters high, what angle gives maximum distance?`
- `A basketball player shoots from 2m high at 7 m/s toward a hoop 3.05m high and 4.6m away. What angles work?`

### Collision Detection
- `Two cars 500m apart, one at 60 mph, other at 45 mph heading toward each other. When do they collide?`
- `Two asteroids: one at (0,0,0) moving at 1000 m/s in x direction, another at (50000, 100, 0) moving at -800 m/s in x. Will they collide?`
- `Spaceship A at (10000,0,0) moving at (-50,0,0) m/s, spaceship B at (-10000,100,0) moving at (45,0,0) m/s. Collision check?`

### Force, Energy & Momentum
- `What force is needed to accelerate a 1500 kg car at 3 m/s²?`
- `What's the kinetic energy of a 2000 kg car traveling at 30 m/s?`
- `Calculate the momentum of a 70 kg runner sprinting at 10 m/s`
- `How much energy does a 0.145 kg baseball have when pitched at 45 m/s?`

### Real-World Applications
- `I'm designing a water fountain. Water shoots up at 15 m/s. How high does it go?`
- `A cannon on a 50 meter cliff fires horizontally at 200 m/s. How far from the base does the projectile land?`
- `Two cars crash: Car A (1500kg) at 30 m/s, Car B (1200kg) at 25 m/s. What's the total kinetic energy at impact?`

### Simulations (Requires Rapier Service)
- `Create a simulation of a ball dropping from 10 meters height and bouncing on the ground`
- `Simulate 5 boxes stacked on top of each other, then have another box hit them from the side`
- `Create a Newton's cradle with 5 spheres and record their motion`
- `Simulate a sphere rolling down a 30-degree ramp`

---

## 📝 Example Scripts

The `examples/` directory contains working demonstration scripts:

### Ready to Run (No External Services Required)

These examples use the built-in **analytic provider** and work immediately:

- **`00_quick_start.py`** - Quick demo of all 5 analytic tools
- **`01_simple_projectile.py`** - Cannonball trajectories, basketball shots, angle comparisons
- **`02_collision_detection.py`** - Car crashes, near misses, asteroid collisions
- **`03_force_energy_momentum.py`** - F=ma, kinetic energy, momentum conservation
- **`04_r3f_visualization.py`** - Generate React Three Fiber visualization data

```bash
# Run any example
python examples/00_quick_start.py
python examples/01_simple_projectile.py
# ... etc
```

### Requires Rapier Service

This example demonstrates rigid-body simulations but needs the Rapier service running:

- **`05_rapier_simulation.py`** - Bouncing balls, collisions, stacking boxes

```bash
# Start Rapier service first (see RAPIER_SERVICE.md)
docker run -p 9000:9000 chuk-rapier-service

# Then run example
python examples/05_rapier_simulation.py
```

**Note:** Examples 00-04 provide full functionality for calculations, collisions, and trajectory generation. Example 05 showcases advanced rigid-body physics when you have the Rapier service available.

---

## ⚙️ Configuration

### Environment Variables

```bash
# Provider selection
PHYSICS_PROVIDER=analytic          # or "rapier"

# Rapier service (only if using Rapier provider)
# Option 1: Public service (recommended for getting started)
RAPIER_SERVICE_URL=https://rapier.chukai.io

# Option 2: Local development
# RAPIER_SERVICE_URL=http://localhost:9000

# Optional configuration
RAPIER_TIMEOUT=30.0
RAPIER_MAX_RETRIES=3
RAPIER_RETRY_DELAY=1.0
```

### YAML Configuration

Create `physics.yaml` in your working directory or `~/.config/chuk-mcp-physics/`:

```yaml
default_provider: rapier

providers:
  # Override provider per tool type
  simulations: rapier
  projectile_motion: analytic

rapier:
  # Public service (recommended)
  service_url: https://rapier.chukai.io

  # Or local development
  # service_url: http://localhost:9000

  timeout: 30.0
  max_retries: 3
  retry_delay: 1.0
```

---

## 🔧 Development

```bash
# Clone repository
git clone https://github.com/yourusername/chuk-mcp-physics
cd chuk-mcp-physics

# Install in development mode
make dev-install

# Run tests
make test

# Run tests with coverage
make test-cov

# Format code
make format

# Run all checks
make check

# Build package
make build
```

---

## 🐳 Docker Deployment

```bash
# Build image
make docker-build

# Run container
make docker-run

# Access at http://localhost:8000
```

---

## ☁️ Production Deployment

### Live Public Services

**Current Production Services:**
- **Rapier Physics Engine**: https://rapier.chukai.io
  - Public API for physics simulations
  - No authentication required for basic usage
  - Rate limits may apply

**Quick Test:**
```bash
# Test the public Rapier service
curl https://rapier.chukai.io/health

# Use with chuk-mcp-physics
export RAPIER_SERVICE_URL=https://rapier.chukai.io
uvx chuk-mcp-physics
```

### Deploy Your Own Rapier Service

If you need your own private Rapier service instance:

#### 1. Deploy Rapier Service to Fly.io

```bash
cd rapier-service

# Login to Fly.io
fly auth login

# Create and deploy
fly apps create your-rapier-physics
fly deploy

# Add custom domain (optional)
fly certs add rapier.yourdomain.com -a your-rapier-physics

# Verify
curl https://your-rapier-physics.fly.dev/health
```

#### 2. Configure chuk-mcp-physics to Use Your Service

```bash
# Option 1: Environment variable
export RAPIER_SERVICE_URL=https://rapier.yourdomain.com
uvx chuk-mcp-physics

# Option 2: YAML config (physics.yaml)
# rapier:
#   service_url: https://rapier.yourdomain.com
```

**Why deploy your own?**
- 🔒 Private instance for production workloads
- 📈 Custom scaling and resource allocation
- 🌍 Deploy closer to your users (different regions)
- 💾 Persistent simulations and custom configurations

See **[DEPLOYMENT.md](DEPLOYMENT.md)** for complete deployment guide, scaling strategies, and CI/CD setup.

---

## 🦀 Rapier Service Setup

For full rigid-body simulations, you have several options:

### Option 1: Use Public Service (Easiest)

```bash
# No setup required - just configure the URL
export RAPIER_SERVICE_URL=https://rapier.chukai.io
uvx chuk-mcp-physics
```

### Option 2: Run Locally with Docker

```bash
# Using Docker
docker run -p 9000:9000 chuk-rapier-service

# Configure to use local service
export RAPIER_SERVICE_URL=http://localhost:9000
uvx chuk-mcp-physics
```

### Option 3: Build from Source

```bash
# Build and run the Rust service
cd rapier-service
cargo run --release

# In another terminal
export RAPIER_SERVICE_URL=http://localhost:9000
uvx chuk-mcp-physics
```

See **[RAPIER_SERVICE.md](RAPIER_SERVICE.md)** for:
- Complete API specification
- Rust implementation guide
- Docker deployment details
- Testing examples

---

## 📊 Comparison: Analytic vs Rapier

| Feature | Analytic Provider | Rapier Provider |
|---------|------------------|-----------------|
| **Projectile motion** | ✅ Exact (kinematic eqs) | ✅ Simulated |
| **Simple collisions** | ✅ Exact (sphere-sphere) | ✅ Simulated |
| **Force/energy/momentum** | ✅ Direct calculation | ✅ Can derive from sim |
| **Rigid-body dynamics** | ❌ Not supported | ✅ Full 3D/2D physics |
| **Complex shapes** | ❌ Spheres only | ✅ Box, capsule, mesh, etc |
| **Friction/restitution** | ❌ Not modeled | ✅ Full material properties |
| **Multi-body systems** | ❌ Not supported | ✅ Unlimited bodies |
| **Constraints/joints** | ❌ Not supported | ✅ Hinges, sliders, etc |
| **Performance** | ⚡ Instant | 🐇 Fast (Rust) |
| **Setup** | 📦 Built-in | 🦀 Requires Rapier service |

**Recommendation:**
- Use **Analytic** for simple calculations, education, quick answers
- Use **Rapier** for complex simulations, games, visualizations, multi-body dynamics

---

## 🤝 Contributing

Contributions welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md).

---

## 📄 License

MIT License - see [LICENSE](LICENSE) for details.

---

## 🙏 Acknowledgments

- [Rapier](https://rapier.rs/) - Fast 2D/3D physics engine in Rust
- [chuk-mcp-server](https://github.com/chrishayuk/chuk-mcp-server) - MCP server framework
- [Model Context Protocol](https://modelcontextprotocol.io/) - Protocol specification

---

## 📚 See Also

- [RAPIER_SERVICE.md](RAPIER_SERVICE.md) - Rapier microservice specification
- [Examples](examples/) - Example usage patterns
- [API Documentation](docs/API.md) - Detailed tool reference
