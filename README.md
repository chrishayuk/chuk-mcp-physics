# Physics MCP Server

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Test Coverage](https://img.shields.io/badge/coverage-98%25-brightgreen.svg)](#)
[![Tests](https://img.shields.io/badge/tests-515%20passing-brightgreen.svg)](#)

**Features:** 🌀 Magnus Force • 💨 Wind Effects • 🏔️ Altitude • 🌡️ Temperature • 🔄 Tumbling Drag • 🎮 Rigid-Body Sims • 📊 515 Tests

**A Model Context Protocol (MCP) server that brings comprehensive physics simulation and calculation capabilities to Large Language Models.**

Transform your LLM into a physics engine! This MCP server provides 55 specialized tools spanning classical mechanics, fluid dynamics, rotational motion, and rigid-body simulations. Built for seamless integration with Claude, ChatGPT, and any MCP-compatible AI system.

## 📚 Table of Contents

- [What is This?](#-what-is-this)
- [What's Included](#-whats-included)
- [Quick Start](#-quick-start-30-seconds)
- [Use Cases](#-use-cases)
- [Available Tools](#-available-tools)
  - [Common Drag Coefficients](#common-drag-coefficients-cd-reference)
  - [Advanced Projectile Features](#advanced-projectile-features) 🌀💨🏔️
- [Examples](#-examples)
- [Development](#-development)

## 🌟 What is This?

**Physics MCP Server** enables LLMs to perform sophisticated physics calculations and simulations through a standardized protocol. Instead of hallucinating physics formulas or making calculation errors, LLMs can now:

- **Calculate precisely**: Use validated physics formulas for exact results
- **Simulate realistically**: Run rigid-body physics with the Rapier engine
- **Visualize motion**: Generate trajectory data for React Three Fiber, Remotion, and other 3D frameworks
- **Teach interactively**: Answer physics questions with real calculations, not memorized facts
- **Design intelligently**: Analyze forces, collisions, and motion for engineering applications

### Why MCP?

The [Model Context Protocol](https://modelcontextprotocol.io/) provides a standardized way to extend LLM capabilities beyond text generation. This server implements MCP to give language models direct access to:

- ⚡ **Instant physics calculations** (projectile motion, forces, energy)
- 🎮 **Rigid-body simulations** (collisions, bounces, stacking)
- 📊 **Motion analysis** (trajectory fitting, kinematics, velocity profiles)
- 🌊 **Fluid dynamics** (drag, buoyancy, lift, Bernoulli)
- 🔄 **Rotational mechanics** (torque, angular momentum, gyroscopes)
- ⚖️ **Static analysis** (equilibrium, beam reactions, friction)
- 🔄 **Unit conversions** (62 unit types: velocity, distance, mass, time, acceleration, torque, frequency, data size, and more)

## 📦 What's Included

### 55 Physics Tools Across 10 Categories

| Category | Tools | Description |
|----------|-------|-------------|
| **Basic Mechanics** | 8 tools | Projectile motion, forces, energy, momentum, collisions |
| **Fluid Dynamics** | 10 tools | Drag, buoyancy, terminal velocity, lift, Magnus force, Bernoulli |
| **Rotational Dynamics** | 5 tools | Torque, moment of inertia, angular momentum, rotational KE |
| **Oscillations** | 5 tools | Springs, pendulums, harmonic motion, damping |
| **Circular Motion** | 5 tools | Centripetal force, orbits, banking angles, escape velocity |
| **Statics** | 7 tools | Force balance, torque balance, friction, beam reactions |
| **Kinematics** | 7 tools | Motion analysis, trajectory fitting, velocity calculations, projectile with drag |
| **Collisions** | 2 tools | Elastic and inelastic 3D collisions with energy loss |
| **Conservation Laws** | 4 tools | Energy, momentum, and angular momentum verification |
| **Unit Conversions** | 2 tools | 62 unit types across 16 categories (velocity, distance, mass, time, acceleration, torque, frequency, data, etc.) |

### Realistic vs Ideal Physics Comparison

See the dramatic difference when including real-world effects:

| Sport | Scenario | Ideal (No Drag) | Realistic (With Enhancements) | Difference |
|-------|----------|----------------|-------------------------------|------------|
| ⚾ Baseball | 90 mph fastball | 87.5m | 52.3m (with drag) | **-40%** range |
| ⛳ Golf | Pro drive at sea level | 251m | 129m (with drag) | **-49%** range |
| ⛳ Golf | Same drive in Denver | 251m | 181m (drag + altitude) | **-28%** range |
| ⚽ Soccer | Free kick with wind | 25m straight | 26m + 5.6m curve (wind + spin) | **Bends 5.6m!** |
| 🎾 Tennis | Serve on hot day | 27m | 26.8m (less drag) | **+2.3%** vs cold |

💡 **Key Insight**: Real physics matters! Air resistance can reduce range by 20-70% depending on the sport.

### Two Calculation Modes

1. **Analytic Mode** (Built-in, no setup)
   - Instant mathematical calculations
   - Perfect for education and quick answers
   - Exact solutions using physics formulas
   - **Now includes**: Advanced drag, spin (Magnus), wind, altitude effects

2. **Simulation Mode** (Requires Rapier service)
   - Full rigid-body dynamics
   - Complex multi-object interactions
   - Realistic material properties and constraints

## 🚀 Quick Start (30 seconds)

```bash
# Try it instantly with uvx (no installation needed)
uvx chuk-mcp-physics

# Or with the public Rapier service for simulations
RAPIER_SERVICE_URL=https://rapier.chukai.io uvx chuk-mcp-physics

# Or use the public hosted MCP server (no local installation)
# Add to Claude Desktop config with URL: https://physics.chukai.io/mcp
```

**For Claude Desktop:** Add to your config file:

**Option 1: Public Hosted MCP Server (Easiest - No Installation)**
```json
{
  "mcpServers": {
    "physics": {
      "command": "node",
      "args": ["-e", "require('https').get('physics.chukai.io/mcp')"]
    }
  }
}
```

**Option 2: Local uvx (Recommended)**
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
**Realistic Ball Trajectories with Air Resistance**
```
User: "How far does a 90 mph baseball fastball actually travel with air resistance?"
LLM: [calls calculate_projectile_with_drag with baseball parameters]
     "With drag (realistic): 52.3m range
      Without drag (vacuum): 87.5m range
      Air resistance reduces range by 40%! Energy lost to drag: 89.2 J..."
```

**Golf Drive Analysis**
```
User: "Pro golfer hits 70 m/s (155 mph) at 12° angle. How far with real air resistance?"
LLM: [calls calculate_projectile_with_drag with golf ball parameters]
     "With drag: 129.4m (142 yards)
      Without drag: 251.0m (274 yards)
      Dimples reduce drag coefficient from 0.47 to 0.25 - saves ~50% range loss!"
```

**Basketball 3-Pointer**
```
User: "What launch angle for a 7.5 m/s shot from 6.75m away (3-point line)?"
LLM: [calls calculate_projectile_with_drag iterating angles]
     "With air resistance, optimal angle is 48° (high arc).
      Range: 6.73m (close!), max height: 3.2m, flight time: 1.1s..."
```

**Shot Analysis Applications**
- **Baseball**: Pitch trajectories, drag reduces 90mph fastball range by 40%
- **Golf**: Drive distance with dimpled ball (Cd=0.25 vs smooth ball Cd=0.47)
- **Basketball**: Arc optimization for free throws and 3-pointers
- **Soccer**: Penalty kick trajectories, minimal drag at short distances
- **Track & field**: Javelin, shot put with realistic air resistance
- **Tennis**: Serve and groundstroke trajectory analysis

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

### 7. **Fluid Dynamics & Marine/Aerospace Engineering**
**Underwater Torpedo Simulation**
```
User: "A torpedo is launched underwater at 20 m/s. It weighs 100kg, has a
       streamlined shape (Cd=0.04), and cross-section of 0.03 m².
       How far does it travel in 30 seconds?"
LLM: [calls simulate_underwater_motion]
     "The torpedo travels 147.5 meters before drag and buoyancy slow it down.
      Final velocity: 0.2 m/s. Maximum depth: 729.9 m..."
```

**Terminal Velocity & Drag Analysis**
```
User: "What's the terminal velocity of a skydiver (70kg, 0.7m² area)?"
LLM: [calls calculate_terminal_velocity]
     "Terminal velocity is 40 m/s (90 mph) in belly-down position.
      Takes 12.2 seconds to reach 95% of terminal velocity..."
```

**Buoyancy & Float/Sink Predictions**
```
User: "Will a 1kg steel ball (10cm diameter) float in water?"
LLM: [calls calculate_buoyancy]
     "No, it will sink. Buoyant force is 5.14 N, but weight is 9.81 N.
      The ball is denser than water..."
```

**Applications:**
- **Marine engineering:** Submarine drag, torpedo trajectories, underwater vehicles
- **Aerospace:** Parachute descent, atmospheric re-entry, drag optimization
- **Sports science:** Swimming efficiency, diving trajectories
- **Product design:** Floatation devices, drag reduction, hydrodynamics
- **Environmental:** Particle settling rates, pollutant dispersion

---

### 8. **Film & VFX Pre-visualization**
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

## 📊 Trajectory Data Format

All trajectory recordings follow a canonical schema for maximum interoperability with R3F, Remotion, Three.js, and other animation systems.

### Schema Definition

```typescript
interface Trajectory {
  dt: number;              // Time step between frames (seconds)
  frames: Frame[];         // Ordered list of frames
  meta: {
    body_id: string;       // Fully qualified: "rapier://sim-123/body-1"
    total_time: number;    // Total duration (seconds)
    num_frames: number;    // Frame count
  };
}

interface Frame {
  t: number;                               // Absolute time (seconds)
  position: [number, number, number];      // [x, y, z] in meters
  rotation: [number, number, number, number];  // Quaternion [x, y, z, w]
  velocity?: [number, number, number];     // Optional: linear velocity (m/s)
  angular_velocity?: [number, number, number];  // Optional: angular velocity (rad/s)
}
```

### JSON Example

```json
{
  "dt": 0.016,
  "frames": [
    {
      "t": 0.0,
      "position": [0, 1, 0],
      "rotation": [0, 0, 0, 1],
      "velocity": [0, 0, 0]
    },
    {
      "t": 0.016,
      "position": [0.1, 1.01, 0],
      "rotation": [0, 0.01, 0, 0.9999],
      "velocity": [6.25, 0.61, 0]
    }
  ],
  "meta": {
    "body_id": "rapier://sim-abc123/ball",
    "total_time": 4.8,
    "num_frames": 300
  }
}
```

### Usage in React Three Fiber

```tsx
import { useRef } from "react";
import { useFrame } from "@react-three/fiber";

function AnimatedObject({ trajectory }) {
  const ref = useRef();

  useFrame((state) => {
    const elapsed = state.clock.getElapsedTime();
    const frameIdx = Math.floor(elapsed / trajectory.dt);
    const frame = trajectory.frames[frameIdx % trajectory.frames.length];

    if (frame && ref.current) {
      ref.current.position.fromArray(frame.position);
      ref.current.quaternion.fromArray(frame.rotation);
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

### Usage in Remotion

```tsx
import { useCurrentFrame } from "remotion";
import { ThreeCanvas } from "@remotion/three";

export const PhysicsAnimation = ({ trajectory }) => {
  const frame = useCurrentFrame();
  const frameData = trajectory.frames[frame];

  return (
    <AbsoluteFill>
      <ThreeCanvas>
        <mesh position={frameData.position}>
          <sphereGeometry />
        </mesh>
      </ThreeCanvas>
    </AbsoluteFill>
  );
};
```

### Design Notes

- **Quaternions for rotation:** More compact and interpolation-friendly than Euler angles
- **Absolute time:** Each frame has absolute time `t`, making scrubbing easier
- **Constant dt:** Frames are evenly spaced, simplifying playback
- **Optional velocities:** Include if needed for motion blur or physics visualization
- **Qualified body_id:** Format is `rapier://sim-{id}/{body_id}` for traceability

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
# Clone repository (from your source location)
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

### Tool Organization

Tools are organized into two tiers to help you choose the right abstraction:

#### 1️⃣ Analytic Primitives (No External Service)

**Best for:** Quick calculations, education, simple scenarios

Direct formula-based calculations that return instant results:

| Tool | What It Does | Example Use Case |
|------|--------------|------------------|
| `calculate_projectile_motion` | Ballistic trajectory using kinematic equations | "How far does a cannonball go?" |
| `check_collision` | Predict if two spheres will collide | "Will these asteroids hit?" |
| `calculate_force` | F = ma calculations | "What force accelerates this car?" |
| `calculate_kinetic_energy` | KE = ½mv² | "How much energy in this crash?" |
| `calculate_momentum` | p = mv | "What's the momentum transfer?" |
| `calculate_potential_energy` | PE = mgh | "What's the energy at height?" |
| `calculate_work_power` | Work (F·d) and power (W/t) | "How much work lifting this box?" |
| `calculate_elastic_collision` | 1D elastic collision (conserves energy & momentum) | "Pool balls colliding?" |
| `calculate_drag_force` | Air/water resistance (F = ½ρv²C_dA) | "What's the drag on this car?" |
| `calculate_buoyancy` | Will it float? (Archimedes) | "Does a steel ball float in water?" |
| `calculate_terminal_velocity` | Maximum fall speed | "How fast does a skydiver fall?" |
| `simulate_underwater_motion` | Underwater trajectory with drag & buoyancy | "How far does a torpedo travel?" |

**Characteristics:**
- ⚡ Instant execution (< 1ms)
- 📦 No external dependencies
- 🎯 Exact mathematical solutions
- ✅ Always available (no service required)

**Limitations:**
- Only spherical objects (for collisions)
- No complex shapes
- No multi-body interactions
- No friction or material properties

#### 2️⃣ Simulation Primitives (Requires Rapier Service)

**Best for:** Complex physics, multi-body dynamics, visualization data

Low-level building blocks for rigid-body simulations:

| Tool | What It Does | When to Use |
|------|--------------|-------------|
| `create_simulation` | Initialize physics world | Start any simulation |
| `add_rigid_body` | Add objects (box, sphere, capsule, etc.) | Build scene |
| `step_simulation` | Advance time | Run physics |
| `record_trajectory` | Capture motion for R3F/Remotion | Generate animation data |
| `destroy_simulation` | Cleanup resources | End simulation |

**Characteristics:**
- 🦀 Requires Rapier service
- 🔄 Stateful (track simulation ID)
- 🧩 Composable (combine for complex scenarios)
- 💪 Full rigid-body dynamics

**Typical workflow:**
```python
# 1. Create world
sim = create_simulation(gravity_y=-9.81)

# 2. Add objects
add_rigid_body(sim.sim_id, "ground", type="static", shape="plane")
add_rigid_body(sim.sim_id, "ball", type="dynamic", shape="sphere",
               radius=0.5, position=[0, 5, 0])

# 3. Run simulation
step_simulation(sim.sim_id, steps=300)

# 4. Record for visualization
trajectory = record_trajectory(sim.sim_id, "ball", steps=300)

# 5. Cleanup
destroy_simulation(sim.sim_id)
```

---

## 📋 Complete Tool Reference

### Basic Mechanics (8 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_projectile_motion` | Ballistic trajectories | "How far does a cannonball travel?" |
| `check_collision` | Predict sphere collisions | "Will these asteroids hit?" |
| `calculate_force` | F = ma calculations | "What force accelerates this car?" |
| `calculate_kinetic_energy` | KE = ½mv² | "Energy in moving object?" |
| `calculate_momentum` | p = mv | "Momentum of moving object?" |
| `calculate_potential_energy` | PE = mgh | "Energy at height?" |
| `calculate_work_power` | Work (F·d) and power (W/t) | "Work done lifting object?" |
| `calculate_elastic_collision` | 1D elastic collision | "Pool ball velocities after impact?" |

### Fluid Dynamics (10 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_drag_force` | Air/water resistance | "Drag on car at speed?" |
| `calculate_buoyancy` | Will it float? (Archimedes) | "Does steel ball float?" |
| `calculate_terminal_velocity` | Maximum fall speed | "Skydiver terminal velocity?" |
| `simulate_underwater_motion` | Underwater trajectory | "How far does torpedo travel?" |
| `calculate_lift_force` | Aerodynamic lift | "Lift on aircraft wing?" |
| `calculate_magnus_force` | Force on spinning ball | "Why does curveball curve?" |
| `calculate_bernoulli` | Pressure in flowing fluid | "Pressure in pipe constriction?" |
| `calculate_pressure_at_depth` | Hydrostatic pressure | "Pressure at 30m depth?" |
| `calculate_reynolds_number` | Flow regime classification | "Is flow turbulent?" |
| `calculate_venturi_effect` | Flow through constriction | "Velocity in throat?" |

### Rotational Dynamics (5 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_torque` | τ = r × F (cross product) | "Torque from wrench?" |
| `calculate_moment_of_inertia` | Rotational inertia | "MOI of spinning disk?" |
| `calculate_angular_momentum` | L = Iω | "Angular momentum of wheel?" |
| `calculate_rotational_kinetic_energy` | Rotational KE = ½Iω² | "Energy in flywheel?" |
| `calculate_angular_acceleration` | α = τ/I | "How fast does it spin up?" |

### Oscillations & Waves (5 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_hookes_law` | Spring force F = -kx | "Force in compressed spring?" |
| `calculate_spring_mass_period` | Oscillation period | "How fast does mass oscillate?" |
| `calculate_simple_harmonic_motion` | Position at time t | "Where is mass at t=2s?" |
| `calculate_damped_oscillation` | Motion with damping | "How quickly does it settle?" |
| `calculate_pendulum_period` | Pendulum swing time | "Period of 1m pendulum?" |

### Circular Motion & Orbits (5 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_centripetal_force` | F_c = mv²/r | "Force for circular motion?" |
| `calculate_orbital_period` | Kepler's 3rd law | "Satellite orbital period?" |
| `calculate_banking_angle` | Optimal curve angle | "Banking for highway curve?" |
| `calculate_escape_velocity` | Escape from gravity | "Earth escape velocity?" |
| `analyze_circular_orbit` | Complete orbital analysis | "Analyze ISS orbit?" |

### Statics & Equilibrium (7 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `check_force_balance` | Verify ΣF = 0 | "Are forces balanced?" |
| `check_torque_balance` | Verify Στ = 0 | "Will seesaw balance?" |
| `calculate_center_of_mass` | Balance point | "Where is center of mass?" |
| `calculate_static_friction` | Max friction force | "Will object slip?" |
| `calculate_normal_force` | Force on incline | "Normal force on ramp?" |
| `check_equilibrium` | Force + torque balance | "Is structure stable?" |
| `calculate_beam_reactions` | Support forces | "Reaction forces on beam?" |

### Kinematics Analysis (7 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_acceleration_from_position` | Derive from position data | "Acceleration from motion capture?" |
| `calculate_jerk` | Rate of acceleration change | "How jerky is motion?" |
| `fit_trajectory` | Find trajectory equation | "Fit parabola to data?" |
| `generate_motion_graph` | Position/velocity/accel graphs | "Generate motion graphs?" |
| `calculate_average_speed` | Speed along path | "Average speed on route?" |
| `calculate_instantaneous_velocity` | Velocity at exact time | "Speed at t=2.5s?" |
| `calculate_projectile_with_drag` | Realistic projectile with air resistance | "How far does baseball actually go?" |

### Advanced Collisions (2 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `calculate_elastic_collision_3d` | 3D perfect collision | "Pool balls in 3D?" |
| `calculate_inelastic_collision_3d` | 3D collision with energy loss | "Car crash analysis?" |

### Conservation Laws (4 tools)

| Tool | Purpose | Example |
|------|---------|---------|
| `check_energy_conservation` | Verify energy conserved | "Is collision realistic?" |
| `check_momentum_conservation` | Verify momentum conserved | "Is momentum preserved?" |
| `check_angular_momentum_conservation` | Verify L conserved | "Is rotation valid?" |
| `track_energy_dissipation` | Energy loss over time | "Where did energy go?" |

### Unit Conversions (2 tools)

| Tool | Purpose | Supported Units |
|------|---------|-----------------|
| `convert_unit` | Convert between units | **Velocity**: m/s, km/h, mph, ft/s, knots<br/>**Distance**: m, km, mi, ft, yd, in<br/>**Mass**: kg, g, lb, oz<br/>**Force**: N, kN, lbf<br/>**Energy**: J, kJ, cal, BTU, kWh<br/>**Power**: W, kW, hp<br/>**Temperature**: K, C, F<br/>**Angle**: rad, deg<br/>**Pressure**: Pa, kPa, bar, psi, atm<br/>**Area**: m², km², ft², acre<br/>**Volume**: m³, L, gal, ft³<br/>**Time**: s, min, hr, day<br/>**Acceleration**: m/s², g, ft/s²<br/>**Torque**: N·m, lb·ft, lb·in<br/>**Frequency**: Hz, kHz, MHz, GHz<br/>**Data Size**: B, KB, MB, GB |
| `list_unit_conversions` | Get all supported units | Returns complete list of conversions |

**Examples:**
```python
# Natural language queries work perfectly
"Convert 60 mph to m/s"  # → 26.82 m/s
"How fast is 100 km/h in mph?"  # → 62.14 mph
"What's 10 kg in pounds?"  # → 22.05 lb
"Convert 100 feet to meters"  # → 30.48 m
"What's 98.6°F in Celsius?"  # → 37°C
"Convert 3 g-force to m/s²"  # → 29.42 m/s²
"What's 300 N·m in lb·ft?"  # → 221.27 lb·ft
"Convert 1 hour to seconds"  # → 3600 s
```

**Features:**
- ⚡ Instant conversions (no external service needed)
- 🔄 Automatic indirect conversions (e.g., mph → km/h via m/s)
- 📐 **70+ unit types across 16 categories**
- 🎯 Perfect for natural language physics queries
- 🚀 Includes engineering units (torque, acceleration, frequency)

---

### Common Drag Coefficients (Cd) Reference

For use with `calculate_projectile_with_drag` tool:

| Object | Drag Coefficient (Cd) | Notes |
|--------|----------------------|-------|
| **Sports Balls** | | |
| Baseball | 0.4 | Stitched surface |
| Golf ball (dimpled) | 0.25 | Dimples reduce drag by ~50% |
| Golf ball (smooth) | 0.47 | Without dimples (don't use!) |
| Basketball | 0.55 | Large, textured surface |
| Soccer ball | 0.25 | Modern, smooth panels |
| Tennis ball | 0.55 | Fuzzy surface |
| Football (American) | 0.05-0.15 | Highly streamlined, orientation-dependent |
| **Generic Shapes** | | |
| Sphere (smooth) | 0.47 | Default reference |
| Flat plate (perpendicular) | 1.28 | Maximum drag |
| Streamlined body | 0.04 | Teardrop/airfoil |
| Cylinder (perpendicular) | 1.15 | Like a pole |
| **Vehicles** | | |
| Car (modern) | 0.25-0.35 | Aerodynamic design |
| Truck | 0.6-0.9 | Boxy shape |
| Bicycle + rider | 0.9 | Upright position |
| **Human Body** | | |
| Skydiver (belly-down) | 1.0-1.3 | Maximum drag |
| Skydiver (head-down) | 0.7 | Streamlined |
| **Projectiles** | | |
| Bullet (supersonic) | 0.295 | Pointed nose |
| Artillery shell | 0.15-0.25 | Streamlined |

**Basic Usage Example:**
```python
# Baseball pitch with realistic drag
result = await calculate_projectile_with_drag(
    initial_velocity=40.23,  # 90 mph
    angle_degrees=10,
    mass=0.145,
    cross_sectional_area=0.0043,  # π × (0.037m)²
    drag_coefficient=0.4  # Baseball Cd
)
```

---

### Advanced Projectile Features

The `calculate_projectile_with_drag` tool supports optional enhancements for ultra-realistic simulations:

#### 🌀 Magnus Force (Spin Effects)

Spin creates a pressure differential that deflects the ball's path. Essential for:
- **Baseball**: Curveballs (topspin drops), fastballs (backspin lifts)
- **Golf**: Backspin increases carry, sidespin causes slices/hooks
- **Soccer**: Bending free kicks around defensive walls
- **Tennis**: Topspin brings ball down faster

```python
# Baseball curveball with topspin
result = await calculate_projectile_with_drag(
    initial_velocity=35,
    angle_degrees=0,
    mass=0.145,
    cross_sectional_area=0.0043,
    drag_coefficient=0.4,
    spin_rate=261.8,  # 2500 rpm = 261.8 rad/s
    spin_axis=[0, 0, -1]  # Topspin (negative z-axis)
)
# Returns lateral_deflection and magnus_force_max
```

**Spin Parameters:**
- `spin_rate`: Rotation speed in rad/s (convert from RPM: rpm × 2π/60)
- `spin_axis`: Unit vector [x, y, z] indicating spin direction
  - `[0, 0, 1]` = Backspin (lifts)
  - `[0, 0, -1]` = Topspin (drops)
  - `[0, 1, 0]` = Sidespin (hooks/slices)

#### 💨 Wind Effects

Constant wind vector affects trajectory throughout flight:

```python
# Soccer free kick with 5 m/s crosswind
result = await calculate_projectile_with_drag(
    initial_velocity=28,
    angle_degrees=12,
    mass=0.43,
    cross_sectional_area=0.0388,
    drag_coefficient=0.25,
    wind_velocity=[5.0, 0.0]  # [horizontal, vertical] in m/s
)
# Returns wind_drift showing total deflection
```

**Wind Types:**
- **Tailwind**: `[+X, 0]` - increases range
- **Headwind**: `[-X, 0]` - decreases range
- **Crosswind**: `[X, 0]` - lateral deflection
- **Updraft**: `[0, +Y]` - increases height and flight time
- **Downdraft**: `[0, -Y]` - decreases height

#### 🏔️ Altitude & Temperature Effects

Air density varies with elevation and temperature, dramatically affecting drag:

```python
# Golf drive in Denver (1600m elevation, 20°C)
result = await calculate_projectile_with_drag(
    initial_velocity=70,
    angle_degrees=12,
    mass=0.0459,
    cross_sectional_area=0.00143,
    drag_coefficient=0.25,
    altitude=1600,  # meters above sea level
    temperature=20  # Celsius
)
# Returns effective_air_density showing actual density used
```

**Real-World Impact:**
- **Denver (1600m)**: ~10% longer drives than sea level
- **Hot day (+20°C)**: ~2-3% less drag than cold day
- **Everest Base Camp (5300m)**: ~50% less air density!

**Air Density Formula:**
```
ρ(h,T) = ρ₀ × exp(-Mgh/RT₀) × (T₀/T)
```
Where:
- ρ₀ = sea level density (1.225 kg/m³)
- h = altitude (meters)
- T = temperature (Kelvin)
- M = molar mass of air (0.029 kg/mol)
- g = gravity (9.81 m/s²)
- R = gas constant (8.314 J/(mol·K))

#### 🌟 Combined Effects Example

```python
# Golf ball with ALL effects (spin + wind + altitude)
result = await calculate_projectile_with_drag(
    initial_velocity=70,
    angle_degrees=12,
    mass=0.0459,
    cross_sectional_area=0.00143,
    drag_coefficient=0.25,
    spin_rate=200,  # Backspin
    spin_axis=[0, 0, 1],
    wind_velocity=[3, 0],  # Tailwind
    altitude=1000,  # Moderate elevation
    temperature=25  # Warm day
)
# All effects combine for maximum realism!
```

**See Examples:**
- `examples/sports_projectiles_with_drag.py` - Basic drag effects
- `examples/advanced_projectile_effects.py` - Magnus force, wind, altitude

---

### 🔄 Orientation-Dependent Drag (Rapier Simulations)

For tumbling objects like footballs, frisbees, and javelins, drag varies dramatically based on orientation. A football in a perfect spiral has **3-6× less drag** than when tumbling end-over-end!

**Available via Rapier rigid-body simulations** using the `add_rigid_body` tool with orientation-dependent drag parameters.

#### How It Works

Objects moving through air experience drag that depends on their orientation:
- **Football spiral**: Streamlined along flight path → low drag (~0.1 Cd)
- **Football tumbling**: Broadside to airflow → high drag (~0.6 Cd)
- **Frisbee flat**: Minimal cross-section → low drag (~0.08 Cd)
- **Frisbee tilted**: Larger cross-section → higher drag

**New `add_rigid_body` Parameters:**
```python
drag_coefficient: float      # Base Cd value
drag_area: float             # Reference cross-sectional area (m²)
drag_axis_ratios: [x, y, z]  # Drag variation along body axes
fluid_density: float         # Fluid density (air=1.225, water=1000)
```

#### Example: Football Spiral vs Tumble

```python
# Perfect spiral (low drag along Y-axis)
await add_rigid_body(
    sim_id,
    id="spiral",
    shape="capsule",
    size=[0.17, 0.28],  # diameter, length
    mass=0.42,
    position=[0, 2, 0],
    velocity=[vx, vy, 0],
    angular_velocity=[0, 126, 0],  # 20 rev/s spin
    # Orientation-dependent drag
    drag_coefficient=0.1,
    drag_area=0.023,  # End-on area
    drag_axis_ratios=[1.0, 0.2, 1.0],  # 5× less drag along Y
    fluid_density=1.225
)

# Tumbling (higher drag, averaging all orientations)
await add_rigid_body(
    sim_id,
    id="tumble",
    shape="capsule",
    size=[0.17, 0.28],
    mass=0.42,
    position=[0, 2, 0],
    velocity=[vx, vy, 0],
    angular_velocity=[31.4, 0, 0],  # Tumbling rotation
    # Orientation-dependent drag
    drag_coefficient=0.6,  # Higher base Cd
    drag_area=0.048,  # Broadside area
    drag_axis_ratios=[0.8, 1.0, 0.8],  # Less streamlining
    fluid_density=1.225
)
```

**Result**: Spiral can travel **20-40% farther** than tumble!

#### Common `drag_axis_ratios` Patterns

The `drag_axis_ratios` parameter specifies how drag varies along each body-local axis [X, Y, Z]:

| Object | Ratios | Streamlined Axis | Use Case |
|--------|--------|-----------------|----------|
| Football spiral | `[1.0, 0.2, 1.0]` | Y (length) | Perfect pass |
| Javelin | `[1.5, 0.2, 1.5]` | Y (length) | Optimal flight |
| Frisbee flat | `[1.0, 0.1, 1.0]` | Y (vertical) | Stable throw |
| Sphere | `[1.0, 1.0, 1.0]` | None | Basketball, etc. |
| Disc tumbling | `[0.8, 1.0, 0.8]` | Less variation | Wobbly throw |

**Physical Meaning:**
- `0.2` = 20% of base drag along that axis (very streamlined)
- `1.0` = 100% of base drag (normal)
- `1.5` = 150% of base drag (higher resistance)

#### Real-World Examples

**⚽ Football Throw** (spiral vs tumble):
```python
# Spiral: 45-50 yards typical
# Tumble: 30-35 yards (30-40% loss)
```

**🥏 Frisbee** (stable vs wobbling):
```python
# Stable (600 rpm spin): 60-80 meters
# Wobbling (slow spin): 30-40 meters (50% loss)
```

**🏹 Javelin** (optimal vs poor technique):
```python
# Optimal angle: 70-90 meters (Olympic level)
# Poor release: 40-50 meters (45% loss)
```

#### Important Notes

⚠️ **Requires Rapier Service**: Orientation-dependent drag calculations are performed by the Rapier physics service (Rust implementation). The Python MCP server defines the API and passes parameters to Rapier.

🎯 **When to Use**:
- Sports simulations (football, frisbee, discus)
- Projectile accuracy (javelin, arrows, darts)
- Aerospace applications (rocket tumbling, debris)

🔬 **Physics**:
The drag force is calculated in the Rapier service using the body's current orientation (quaternion) to transform body-local drag coefficients into world-space drag forces.

**Hybrid Drag Implementation**:
Rapier uses a hybrid approach to handle extreme drag cases:
- **Normal drag** (ratio < 2.0): Force-based orientation-dependent drag with full anisotropic behavior
- **Extreme drag** (ratio ≥ 2.0): Damping-based drag for stability when drag-to-weight ratio is very high
  - Automatically activates for objects like ping pong balls (high drag, low mass)
  - Prevents numerical instabilities while maintaining realistic energy dissipation
  - Logged as INFO when triggered: `"Body 'name' has extreme drag (ratio=X.XX), using damping"`

Where `drag_ratio = 0.5 * fluid_density * drag_coefficient * drag_area * v_typical^2 / (mass * g)`

**Measurement Notes**:
When analyzing trajectories, use `max(x_positions)` instead of `final_x` to measure range. Rapier's solver may occasionally jitter backward slightly near ground impact, but this is a measurement artifact, not a physics error. The drag forces always oppose motion correctly.

**See Example:**
- `examples/tumbling_projectiles.py` - Football, frisbee, and javelin orientation effects

---

## ✨ Phase 1 Features (Production Ready)

All Phase 1 features are **complete, tested (98% coverage), and deployed** to production!

### Phase 1.1: Bounce Detection 🏀

**Automatically detect and analyze bounces** in ball trajectories with energy loss calculations.

**What it does:**
- Detects bounce events from trajectory data (velocity reversals near ground)
- Calculates energy loss percentage for each bounce
- Provides before/after velocities and heights
- Perfect for answering "how many bounces?" and "when does it stop?"

**Tool:** `record_trajectory_with_events`

**Example:**
```python
# Record a bouncing ball
traj = await record_trajectory_with_events(
    sim_id=sim_id,
    body_id="ball",
    steps=300,
    detect_bounces=True,
    bounce_height_threshold=0.01  # 1cm = "on ground"
)

print(f"Detected {len(traj.bounces)} bounces")
for bounce in traj.bounces:
    print(f"Bounce #{bounce.bounce_number}:")
    print(f"  Time: {bounce.time:.2f}s")
    print(f"  Height: {bounce.height_at_bounce:.3f}m")
    print(f"  Energy loss: {bounce.energy_loss_percent:.1f}%")
    print(f"  Speed before: {bounce.speed_before:.2f} m/s")
    print(f"  Speed after: {bounce.speed_after:.2f} m/s")
```

**Output:**
```
Detected 5 bounces
Bounce #1: Time: 1.43s, Height: 0.00m, Energy loss: 36.0%, Speed: 14.0→9.0 m/s
Bounce #2: Time: 2.51s, Height: 0.00m, Energy loss: 36.0%, Speed: 8.9→5.7 m/s
Bounce #3: Time: 3.23s, Height: 0.00m, Energy loss: 36.0%, Speed: 5.7→3.6 m/s
...
```

**Use Cases:**
- Sports analytics (basketball arc, tennis serve bounces)
- Product testing (phone drop tests, durability simulations)
- Game development (realistic ball physics)
- Education (demonstrate energy conservation)

**See:** `examples/06_bounce_detection.py` for full demo

---

### Phase 1.2: Contact Events 📊

**Real-time collision tracking** with detailed contact information from the physics engine.

**What it does:**
- Tracks all contact events between bodies during simulation
- Reports contact start, ongoing, and end events
- Provides impulse magnitudes, normals, and relative velocities
- Essential for collision analysis and force calculations

**Included in:** All trajectory recordings (`record_trajectory`, `record_trajectory_with_events`)

**Contact Event Data:**
```python
ContactEvent(
    time=1.43,                          # When contact occurred
    body_a="ball",                      # First body
    body_b="ground",                    # Second body
    contact_point=[0.0, 0.05, 0.0],    # World space position
    normal=[0.0, 1.0, 0.0],            # Contact normal (from A to B)
    impulse_magnitude=14.2,             # Collision impulse (N⋅s)
    relative_velocity=[0.0, -14.0, 0.0], # Relative velocity
    event_type="started"                # "started", "ongoing", or "ended"
)
```

**Example:**
```python
traj = await record_trajectory(sim_id, "ball", steps=300)

# Analyze contacts
for event in traj.contact_events:
    if event.event_type == "started":
        print(f"Collision at t={event.time:.2f}s")
        print(f"  Bodies: {event.body_a} ↔ {event.body_b}")
        print(f"  Impulse: {event.impulse_magnitude:.1f} N⋅s")
        print(f"  Impact speed: {abs(event.relative_velocity[1]):.1f} m/s")
```

**Use Cases:**
- Collision analysis (car crashes, sports impacts)
- Force calculations (derive forces from impulses)
- Interaction tracking (which objects touched what)
- VFX triggers (spark effects on collisions)

**See:** `examples/07_contact_events.py` for full demo

---

### Phase 1.3: Joints & Constraints 🔗

**Connect rigid bodies** with realistic joints for complex mechanical systems.

**What it does:**
- Create constraints between bodies (hinges, sliders, ball-and-socket, fixed)
- Build complex systems (pendulums, chains, ragdolls, machinery)
- Realistic mechanical motion (doors, wheels, linkages)
- Perfect for simulating articulated structures

**Tool:** `add_joint`

**Joint Types:**

| Type | Description | Example Uses |
|------|-------------|--------------|
| **FIXED** | Rigid connection (glue) | Attach hat to head, weld joints |
| **REVOLUTE** | Hinge rotation around axis | Doors, pendulums, wheels |
| **SPHERICAL** | Ball-and-socket rotation | Ragdoll shoulders, gimbal mounts |
| **PRISMATIC** | Sliding along axis | Pistons, elevators, sliders |

**Example - Simple Pendulum:**
```python
# Create anchor point
await add_rigid_body(
    sim_id=sim_id,
    body_id="anchor",
    body_type="static",
    shape="sphere",
    size=[0.05],
    position=[0.0, 3.0, 0.0]
)

# Create pendulum bob
await add_rigid_body(
    sim_id=sim_id,
    body_id="bob",
    body_type="dynamic",
    shape="sphere",
    size=[0.2],
    mass=1.0,
    position=[1.5, 1.5, 0.0]  # Start displaced
)

# Connect with revolute joint (hinge)
await add_joint(
    sim_id=sim_id,
    joint=JointDefinition(
        id="hinge",
        joint_type=JointType.REVOLUTE,
        body_a="anchor",
        body_b="bob",
        anchor_a=[0.0, 0.0, 0.0],    # Center of anchor
        anchor_b=[0.0, 0.2, 0.0],    # Top of bob
        axis=[0.0, 0.0, 1.0]         # Rotate around Z-axis
    )
)

# Simulate and see realistic pendulum motion!
traj = await record_trajectory(sim_id, "bob", steps=300)
```

**Example - Multi-Link Chain:**
```python
# Create anchor
await add_rigid_body(sim_id, "anchor", body_type="static", ...)

# Create 3 chain links
for i in range(3):
    await add_rigid_body(
        sim_id,
        f"link{i}",
        body_type="dynamic",
        shape="box",
        size=[0.1, 0.4, 0.1],
        position=[0.0, 2.5 - i*0.5, 0.0]
    )

# Connect with spherical joints (ball-and-socket)
await add_joint(sim_id, JointDefinition(
    id="joint0",
    joint_type=JointType.SPHERICAL,
    body_a="anchor",
    body_b="link0",
    anchor_a=[0.0, 0.0, 0.0],
    anchor_b=[0.0, 0.2, 0.0]
))

await add_joint(sim_id, JointDefinition(
    id="joint1",
    joint_type=JointType.SPHERICAL,
    body_a="link0",
    body_b="link1",
    anchor_a=[0.0, -0.2, 0.0],
    anchor_b=[0.0, 0.2, 0.0]
))

# ... and so on
```

**Use Cases:**
- Mechanical systems (engines, gears, levers)
- Character animation (ragdolls, inverse kinematics)
- Vehicle suspension (wheels, shocks)
- Architectural simulations (doors, drawbridges)

**See:** `examples/08_pendulum.py` for full demo

---

### Phase 1.4: Damping & Advanced Controls 🌬️

**Realistic energy dissipation** through linear and angular damping.

**What it does:**
- Simulates air resistance (linear damping)
- Simulates rotational friction (angular damping)
- Makes simulations more realistic and stable
- Perfect for settling physics and reducing "floaty" motion

**Parameters:** Added to `add_rigid_body` tool

**Damping Parameters:**
```python
await add_rigid_body(
    sim_id=sim_id,
    body_id="damped_ball",
    body_type="dynamic",
    shape="sphere",
    size=[0.5],
    mass=1.0,
    position=[0.0, 5.0, 0.0],

    # Phase 1.4: Damping
    linear_damping=0.5,   # 0.0 (none) to 1.0 (high) - like air resistance
    angular_damping=0.3   # 0.0 (none) to 1.0 (high) - like rotational friction
)
```

**Effect of Linear Damping:**
- `0.0` = No air resistance (vacuum physics)
- `0.1-0.3` = Light damping (tennis ball in air)
- `0.5-0.7` = Moderate damping (underwater motion)
- `0.9+` = Heavy damping (very viscous fluid)

**Effect of Angular Damping:**
- `0.0` = Spins forever (vacuum)
- `0.1-0.3` = Realistic friction (rolling ball)
- `0.5-0.7` = High friction (rough surface)
- `0.9+` = Almost no rotation (sticky surface)

**Comparison:**
```python
# Without damping - bounces forever
await add_rigid_body(..., linear_damping=0.0)
# Result: Ball bounces 20+ times, takes 30+ seconds to settle

# With damping - realistic settling
await add_rigid_body(..., linear_damping=0.5)
# Result: Ball bounces 5 times, settles in ~5 seconds
```

**Use Cases:**
- Realistic object motion (not "floaty" game physics)
- Faster settling (less simulation time needed)
- Underwater simulations (high damping)
- Space simulations (zero damping)

**See:** `examples/09_phase1_complete.py` for all Phase 1 features combined

---

### Phase 1.5: Fluid Dynamics 🌊

**Analytical fluid calculations** for drag, buoyancy, and underwater motion.

**What it does:**
- Calculates drag forces (quadratic air/water resistance)
- Computes buoyancy using Archimedes' principle
- Determines terminal velocity for falling objects
- Simulates underwater projectile motion with drag and buoyancy

**Tools Available:**

#### 1. `calculate_drag_force` - Air/Water Resistance

Calculate the force opposing motion through a fluid.

```python
# Ball falling through water
result = await calculate_drag_force(
    velocity=[0, -5.0, 0],          # 5 m/s downward
    cross_sectional_area=0.00785,   # π*r² for 10cm diameter
    fluid_density=1000,              # water (air=1.225)
    drag_coefficient=0.47,           # sphere (streamlined=0.04)
    viscosity=1.002e-3               # optional: water viscosity for accurate Re
)

print(f"Drag force: {result['magnitude']:.1f} N (upward)")
print(f"Reynolds number: {result['reynolds_number']:.0f}")
```

**Common drag coefficients:**
- Sphere: 0.47
- Streamlined (torpedo): 0.04
- Flat plate: 1.28
- Human (standing): 1.0-1.3
- Car: 0.25-0.35

**Optional viscosity parameter** (for accurate Reynolds number):
- Water at 20°C: `1.002e-3` Pa·s
- Air at 20°C: `1.825e-5` Pa·s
- Motor oil: `0.1` Pa·s
- If omitted, estimated from density (water-like if >100 kg/m³, else air-like)

#### 2. `calculate_buoyancy` - Will it Float?

Determine buoyant force and whether objects float or sink.

```python
# Check if 1kg steel ball floats
volume = (4/3) * π * (0.05)**3  # 10cm diameter sphere
result = await calculate_buoyancy(
    volume=0.000524,          # m³
    fluid_density=1000        # water
)

weight = 1.0 * 9.81  # 9.81 N
buoyancy = result['buoyant_force']  # 5.14 N

# weight > buoyancy → SINKS
```

#### 3. `calculate_terminal_velocity` - Maximum Fall Speed

Calculate the speed where drag equals weight.

```python
# Skydiver terminal velocity
result = await calculate_terminal_velocity(
    mass=70,                      # kg
    cross_sectional_area=0.7,     # m² (belly-down)
    fluid_density=1.225,          # air
    drag_coefficient=1.0          # human
)

print(f"Terminal velocity: {result['terminal_velocity']:.1f} m/s")
# Result: ~40 m/s (90 mph)
print(f"Time to 95%: {result['time_to_95_percent']:.1f}s")
```

#### 4. `simulate_underwater_motion` - Full Fluid Simulation

Simulate motion through fluids with drag and buoyancy forces.

```python
# Torpedo launched underwater
result = await simulate_underwater_motion(
    initial_velocity=[20, 0, 0],   # 20 m/s forward
    mass=100,                       # kg
    volume=0.05,                    # m³
    cross_sectional_area=0.03,      # m²
    fluid_density=1000,             # water
    drag_coefficient=0.04,          # streamlined
    duration=30.0
)

print(f"Distance traveled: {result['total_distance']:.1f}m")
print(f"Final velocity: {result['final_velocity']}")
print(f"Max depth: {result['max_depth']:.1f}m")
```

**Use Cases:**
- **Marine engineering:** Torpedo trajectories, submarine drag
- **Aerospace:** Skydiving, parachute descent, atmospheric re-entry
- **Sports:** Swimming, diving, underwater ballistics
- **Product design:** Drag optimization, floatation devices
- **Environmental:** Particle settling, pollutant dispersion

**Physics Models:**
- Quadratic drag: F_drag = 0.5 * ρ * v² * C_d * A
- Buoyancy: F_b = ρ_fluid * V * g (Archimedes)
- Terminal velocity: v_t = √(2mg / ρC_dA)
- Numerical integration for complex underwater motion

**See:** `examples/10_fluid_dynamics.py` for comprehensive demonstrations

---

### 🎉 Phase 1 Complete Summary

**Status:** ✅ **All features production-ready**

| Feature | Status | Tool | Coverage |
|---------|--------|------|----------|
| **Bounce Detection** | ✅ Shipped | `record_trajectory_with_events` | 100% |
| **Contact Events** | ✅ Shipped | All trajectory tools | 100% |
| **Joints & Constraints** | ✅ Shipped | `add_joint` | 100% |
| **Damping Controls** | ✅ Shipped | `add_rigid_body` | 100% |
| **Fluid Dynamics** | ✅ Shipped | `calculate_drag_force`, `calculate_buoyancy`, `calculate_terminal_velocity`, `simulate_underwater_motion` | 100% |

**Test Coverage:** 98% (350 tests passing)

**Deployment:**
- 🌐 MCP Server: https://physics.chukai.io/mcp
- 🦀 Rapier Service: https://rapier.chukai.io

**Examples:** See `examples/06_bounce_detection.py` through `examples/10_fluid_dynamics.py`

---

## 🚀 Phase 2 Features (Production Ready)

All Phase 2 features are **complete, tested (98% coverage), and deployed** to production!

### Phase 2.1: Rotational Dynamics 🔄

**Complete rotational motion calculations** including torque, moment of inertia, angular momentum, and rotational kinetic energy.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `calculate_torque` | Calculate torque from force and position (τ = r × F) | "What torque does this wrench apply?" |
| `calculate_moment_of_inertia` | Moment of inertia for common shapes (disk, sphere, rod, etc.) | "What's the rotational inertia?" |
| `calculate_angular_momentum` | Angular momentum (L = Iω) | "How much rotational momentum?" |
| `calculate_rotational_kinetic_energy` | Rotational KE (½Iω²) | "Energy in spinning flywheel?" |
| `calculate_angular_acceleration` | Angular acceleration (α = τ/I) | "How fast does it spin up?" |

**Example - Calculate Torque:**
```python
result = await calculate_torque(
    force_x=50.0,
    force_y=0.0,
    force_z=0.0,
    position_x=0.0,
    position_y=0.0,
    position_z=0.8  # 80cm wrench
)
# torque magnitude = 40 N⋅m
```

**Use Cases:**
- Mechanical engineering (gear systems, engines)
- Robotics (joint torques, motor sizing)
- Sports science (bat swings, golf clubs)
- Aerospace (satellite attitude control)

---

### Phase 2.2: Oscillations & Waves 🌊

**Harmonic motion and spring systems** with damping effects.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `calculate_hookes_law` | Spring force and potential energy (F = -kx) | "How much force in compressed spring?" |
| `calculate_spring_mass_period` | Period and frequency of spring-mass system | "How fast does it oscillate?" |
| `calculate_simple_harmonic_motion` | Position, velocity, acceleration at time t | "Where is the mass at t=2s?" |
| `calculate_damped_oscillation` | Damped harmonic motion (underdamped, critically damped, overdamped) | "How quickly does it settle?" |
| `calculate_pendulum_period` | Period of simple pendulum | "How long is one swing?" |

**Example - Spring-Mass System:**
```python
result = await calculate_spring_mass_period(
    mass=0.5,           # 500g mass
    spring_constant=20.0  # N/m
)
# period ≈ 0.99s, frequency ≈ 1.01 Hz
```

**Use Cases:**
- Mechanical design (suspension systems, vibration isolation)
- Seismology (earthquake oscillations)
- Electronics (LC circuits, resonance)
- Horology (pendulum clocks)

---

### Phase 2.3: Circular Motion & Orbits 🌍

**Circular motion, orbital mechanics, and centripetal forces**.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `calculate_centripetal_force` | Force required for circular motion | "What force keeps car on curve?" |
| `calculate_orbital_period` | Period and velocity for circular orbit | "How long is satellite orbit?" |
| `calculate_banking_angle` | Optimal banking for curved road | "What angle for this turn?" |
| `calculate_escape_velocity` | Minimum velocity to escape gravity | "Can rocket escape Earth?" |
| `analyze_circular_orbit` | Complete orbital analysis (altitude, period, velocity) | "Analyze ISS orbit" |

**Example - Satellite Orbit:**
```python
result = await analyze_circular_orbit(
    altitude=400000.0,       # 400 km above surface
    planet_mass=5.972e24,    # Earth mass
    planet_radius=6.371e6    # Earth radius
)
# orbital_velocity ≈ 7670 m/s
# period ≈ 5530 seconds (92 minutes)
```

**Use Cases:**
- Space missions (orbital calculations, satellite deployment)
- Astrophysics (planetary motion, binary stars)
- Transportation (highway curve design)
- Amusement parks (loop-the-loop, centrifuges)

---

### Phase 2.4: Advanced Collisions 💥

**3D collision calculations** with elastic and inelastic collisions.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `calculate_elastic_collision_3d` | 3D elastic collision (energy conserved) | "Pool ball collisions in 3D" |
| `calculate_inelastic_collision_3d` | 3D inelastic collision with restitution | "Car crash with energy loss" |

**Example - Car Crash:**
```python
result = await calculate_inelastic_collision_3d(
    mass1=1500.0,
    velocity1=[20.0, 0.0, 0.0],
    mass2=1200.0,
    velocity2=[-15.0, 0.0, 0.0],
    coefficient_of_restitution=0.0  # Perfectly inelastic
)
# final_velocity1 = [1.11, 0, 0]
# final_velocity2 = [1.11, 0, 0]  # Stick together
# energy_loss > 0 (deformation energy)
```

---

### Phase 2.5: Conservation Laws ⚖️

**Verify and track conservation** of energy, momentum, and angular momentum.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `check_energy_conservation` | Verify total energy is conserved | "Is this collision realistic?" |
| `check_momentum_conservation` | Verify momentum is conserved | "Does this violate physics?" |
| `check_angular_momentum_conservation` | Verify angular momentum conserved | "Is rotation energy conserved?" |
| `track_energy_dissipation` | Track energy loss over trajectory | "Where did the energy go?" |

**Example - Validate Collision:**
```python
result = await check_energy_conservation(
    initial_kinetic_energy=100.0,
    final_kinetic_energy=50.0,
    initial_potential_energy=0.0,
    final_potential_energy=50.0
)
# is_conserved = True (100 = 50 + 50)
# energy_difference ≈ 0
```

---

### Phase 2.6: Statics & Equilibrium ⚖️

**Static equilibrium analysis** for structures and forces.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `check_force_balance` | Verify ΣF = 0 (force equilibrium) | "Are these forces balanced?" |
| `check_torque_balance` | Verify Στ = 0 (torque equilibrium) | "Will this seesaw balance?" |
| `calculate_center_of_mass` | Find center of mass for system | "Where is the balance point?" |
| `calculate_static_friction` | Maximum friction force, will object slip? | "Will box slide down ramp?" |
| `calculate_normal_force` | Normal force on inclined plane | "What force on ramp?" |
| `check_equilibrium` | Complete equilibrium check (force + torque) | "Is structure stable?" |
| `calculate_beam_reactions` | Reaction forces for simply supported beam | "What are support forces?" |

**Example - Beam Analysis:**
```python
result = await calculate_beam_reactions(
    beam_length=10.0,
    loads=[1000, 500],  # Two point loads
    load_positions=[3.0, 7.0]  # Positions along beam
)
# reaction_left = 800 N
# reaction_right = 700 N
# is_balanced = True
```

**Use Cases:**
- Structural engineering (bridges, buildings)
- Mechanical design (levers, balances)
- Architecture (load analysis)
- Safety analysis (stability checks)

---

### Phase 2.7: Kinematics Analysis 📊

**Analyze motion data** to extract velocities, accelerations, and trajectories.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `calculate_acceleration_from_position` | Derive velocity and acceleration from position data | "Analyze motion capture data" |
| `calculate_jerk` | Calculate jerk (rate of change of acceleration) | "How jerky is this motion?" |
| `fit_trajectory` | Fit polynomial to trajectory (linear, quadratic, cubic) | "Find trajectory equation" |
| `generate_motion_graph` | Generate position/velocity/acceleration graphs | "Visualize kinematics" |
| `calculate_average_speed` | Average speed along path | "What's average speed?" |
| `calculate_instantaneous_velocity` | Velocity at specific time with interpolation | "Speed at exact moment?" |

**Example - Motion Analysis:**
```python
result = await calculate_acceleration_from_position(
    times=[0, 1, 2, 3, 4],
    positions=[[0,0,0], [5,0,0], [10,0,0], [15,0,0], [20,0,0]]
)
# velocities = [[5,0,0], [5,0,0], ...]  # Constant 5 m/s
# average_acceleration ≈ [0,0,0]  # No acceleration
```

**Use Cases:**
- Motion capture analysis (sports, biomechanics)
- Robotics (trajectory planning, motion smoothness)
- Autonomous vehicles (trajectory optimization)
- Scientific research (particle tracking)

---

### Phase 2.8: Advanced Fluid Dynamics 💨

**Extended fluid calculations** including lift, Magnus force, Bernoulli, and viscous flow.

**Tools Available:**

| Tool | Description | Example Use |
|------|-------------|-------------|
| `calculate_lift_force` | Aerodynamic lift (L = ½ρv²C_LA) | "What lift on wing?" |
| `calculate_magnus_force` | Force on spinning ball | "Why does curveball curve?" |
| `calculate_bernoulli` | Bernoulli's equation for flowing fluids | "Pressure in pipe constriction?" |
| `calculate_pressure_at_depth` | Hydrostatic pressure | "Pressure at 30m depth?" |
| `calculate_reynolds_number` | Flow regime (laminar/turbulent) | "Is flow turbulent?" |
| `calculate_venturi_effect` | Flow through constriction | "Velocity in throat?" |

**Example - Aircraft Wing:**
```python
result = await calculate_lift_force(
    velocity=70,          # m/s (~250 km/h)
    wing_area=20.0,       # m²
    lift_coefficient=1.2,
    fluid_density=1.225   # air
)
# lift_force ≈ 73,500 N
```

**Use Cases:**
- Aerospace engineering (aircraft design, aerodynamics)
- Marine engineering (hull design, submarine motion)
- Sports science (ball trajectories, swimming efficiency)
- HVAC design (airflow optimization)

---

## 🎉 Phase 2 Complete Summary

**Status:** ✅ **All features production-ready**

| Feature Area | Tools | Coverage |
|--------------|-------|----------|
| **Rotational Dynamics** | 5 tools | 100% |
| **Oscillations** | 5 tools | 100% |
| **Circular Motion** | 5 tools | 100% |
| **Advanced Collisions** | 2 tools | 100% |
| **Conservation Laws** | 4 tools | 100% |
| **Statics** | 7 tools | 100% |
| **Kinematics Analysis** | 6 tools | 100% |
| **Advanced Fluids** | 6 tools | 100% |

**Total Tools:** 79 tools across all categories
**Test Coverage:** 98% overall (350 tests passing)

---

**Next Up:** Phase 3 - Performance Optimization & Visualization
- Batch simulation API for ML training data
- Parameter sweep utilities
- Enhanced visualization exports
- Performance profiling and optimization

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

These examples demonstrate rigid-body simulations and Phase 1 features. They need the Rapier service running:

- **`05_rapier_simulation.py`** - Bouncing balls, collisions, stacking boxes
- **`06_bounce_detection.py`** - Phase 1.1: Automatic bounce detection and energy analysis
- **`07_contact_events.py`** - Phase 1.2: Real-time contact tracking and collision events
- **`08_pendulum.py`** - Phase 1.3: Joints and constraints (pendulums, chains)
- **`09_phase1_complete.py`** - Phase 1.4: All Phase 1 features (damping, bounces, contacts, joints)
- **`10_fluid_dynamics.py`** - Phase 1.5: Fluid calculations (drag, buoyancy, terminal velocity)
- **`11_rotational_dynamics.py`** - Phase 2.1: Torque, angular momentum, gyroscopes
- **`12_oscillations.py`** - Phase 2.2: Springs, pendulums, harmonic motion, damping
- **`13_circular_motion.py`** - Phase 2.3: Orbital mechanics, centripetal force
- **`14_statics.py`** - Phase 2.6: Static equilibrium, force balance, beam analysis
- **`15_kinematics_analysis.py`** - Phase 2.7: Motion analysis, trajectory fitting
- **`16_roulette_simulation.py`** - 🎰 **Casino Roulette** - Complete showcase of multi-body physics, collisions, and energy dissipation

```bash
# Option 1: Use public Rapier service (easiest)
export RAPIER_SERVICE_URL=https://rapier.chukai.io
python examples/05_rapier_simulation.py
python examples/06_bounce_detection.py
# ... etc

# Option 2: Run local Rapier service with Docker
docker run -p 9000:9000 chuk-rapier-service
export RAPIER_SERVICE_URL=http://localhost:9000
python examples/05_rapier_simulation.py
```

**Note:**
- Examples 00-04 work instantly (no external services)
- Examples 05-09 showcase advanced rigid-body physics with Rapier
- Use the public Rapier service at `https://rapier.chukai.io` or run your own

---

## ⚙️ Configuration

### Environment Variables

```bash
# Provider selection
PHYSICS_PROVIDER=analytic          # or "rapier"

# Rapier service (only if using Rapier provider)
# The default is automatically determined:
# - On Fly.io: uses https://rapier.chukai.io (public service)
# - Locally: uses http://localhost:9000
#
# Override with:
RAPIER_SERVICE_URL=https://rapier.chukai.io  # or http://localhost:9000

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

## 🛡️ Safety & Limits

### Recommended Ranges

Understanding these limits helps prevent timeouts, instabilities, and confusion:

| Parameter | Recommended | Maximum | Notes |
|-----------|-------------|---------|-------|
| **Units** | meters, kg, seconds | - | SI units throughout |
| **dt** | 0.008 - 0.033 | 0.001 - 0.1 | <0.008 = overkill, >0.033 = unstable |
| **steps** | 100 - 5000 | 10,000 | Depends on dt and complexity |
| **bodies** | 1 - 100 | 1,000 | Performance degrades >100 |
| **gravity** | -20 to 0 m/s² | -100 to +100 | Earth = -9.81 |
| **velocity** | 0 - 100 m/s | 1,000 m/s | Very high speeds may cause tunneling |
| **mass** | 0.1 - 10,000 kg | 1e-6 - 1e6 | Extreme ratios cause instability |

### Public Service Limits

The public Rapier service at `https://rapier.chukai.io` has these limits:

- **Max steps per call:** 5,000
- **Max bodies per simulation:** 100
- **Max concurrent simulations:** 10 per IP
- **Request timeout:** 30 seconds
- **Max simulation lifetime:** 1 hour (auto-cleanup)

For larger simulations, run your own Rapier service (see [RAPIER_SERVICE.md](RAPIER_SERVICE.md)).

### Common Pitfalls

#### ❌ Simulation explodes or bodies fly away

**Symptoms:**
- Bodies gain extreme velocities
- Objects disappear from view
- NaN values in positions

**Causes:**
- `dt` too large for the forces involved
- Very high mass ratios (1g object hitting 1000kg object)
- Extreme initial velocities

**Solutions:**
- Reduce `dt` to 0.008 or lower
- Use more similar masses (within 2-3 orders of magnitude)
- Limit initial velocities to <100 m/s

#### ❌ Simulation runs very slowly

**Symptoms:**
- Request takes >10 seconds
- Timeout errors
- High CPU usage

**Causes:**
- Too many bodies (>100)
- Very small `dt` (<0.005)
- Complex mesh colliders
- Too many steps (>5000)

**Solutions:**
- Reduce body count or simplify shapes
- Increase `dt` (balance accuracy vs. speed)
- Break large step counts into multiple calls
- Use primitive shapes (sphere, box) instead of meshes

#### ❌ Objects tunnel through each other

**Symptoms:**
- Fast-moving objects pass through walls
- Collisions not detected
- Objects appear inside each other

**Causes:**
- Very high velocities + large `dt`
- Thin colliders (<0.1m)
- Disabled continuous collision detection (CCD)

**Solutions:**
- Reduce `dt` for high-speed scenarios
- Thicken colliders (minimum 0.1m recommended)
- Reduce velocities
- Enable CCD if available (future feature)

### Best Practices

1. **Start simple:** Test with 2-3 bodies before scaling up
2. **Validate inputs:** Check for NaN, Inf, extreme values before simulation
3. **Monitor performance:** Track step time, adjust `dt`/steps accordingly
4. **Cleanup:** Always `destroy_simulation` when done (prevent memory leaks)
5. **Use analytic when possible:** For simple scenarios, analytic is faster and exact

---

## 🔧 Development

```bash
# Navigate to project directory
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
- **MCP Physics Server**: https://physics.chukai.io/mcp
  - Public hosted MCP server endpoint
  - No local installation required
  - Pre-configured with Rapier service
  - Ready to use in Claude Desktop

- **Rapier Physics Engine**: https://rapier.chukai.io
  - Public API for physics simulations
  - No authentication required for basic usage
  - Rate limits may apply

**Quick Test:**
```bash
# Test the public Rapier service
curl https://rapier.chukai.io/health

# Use with chuk-mcp-physics locally
export RAPIER_SERVICE_URL=https://rapier.chukai.io
uvx chuk-mcp-physics
```

### Deploy Your Own Rapier Service

If you need your own private Rapier service instance:

#### 1. Deploy Rapier Service to Fly.io with Redis

```bash
cd rapier-service

# Login to Fly.io
fly auth login

# Create Redis instance for distributed storage
fly redis create
# Choose: your-rapier-redis, region sjc (or your preferred region), plan 256MB

# Set Redis URL secret (use URL from previous step)
fly secrets set REDIS_URL="redis://default:password@fly-your-rapier-redis.upstash.io"

# Create and deploy the service
fly apps create your-rapier-physics
fly deploy

# Verify Redis connection in logs
fly logs
# Look for: "📦 Initialized RedisStorage backend"

# Add custom domain (optional)
fly certs add rapier.yourdomain.com -a your-rapier-physics

# Verify service is running
curl https://your-rapier-physics.fly.dev/health
```

**Redis Benefits:**
- ✅ Horizontal scaling (multiple service instances share state)
- ✅ Automatic cleanup (TTL-based simulation expiration)
- ✅ Distributed coordination across instances
- ✅ Session persistence within TTL window

**Configuration:**
The service uses Redis automatically on Fly.io (see `rapier-service/fly.toml`). For local development, it defaults to in-memory storage.

See **[rapier-service/FLY_REDIS_SETUP.md](rapier-service/FLY_REDIS_SETUP.md)** for detailed Redis setup guide, monitoring, and troubleshooting.

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
| **Simple collisions** | ✅ Exact (sphere-sphere, elastic) | ✅ Simulated |
| **Force/energy/momentum** | ✅ F=ma, KE, PE, momentum, work/power | ✅ Can derive from sim |
| **Fluid dynamics** | ✅ Drag, buoyancy, terminal velocity | ❌ Not supported |
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

Apache License 2.0 - see [LICENSE](LICENSE) for details.

> This is a demonstration project provided as-is for learning and testing purposes.

---

## 🙏 Acknowledgments

- [Rapier](https://rapier.rs/) - Fast 2D/3D physics engine in Rust
- [Model Context Protocol](https://modelcontextprotocol.io/) - Protocol specification

---

## 📚 See Also

- [RAPIER_SERVICE.md](RAPIER_SERVICE.md) - Rapier microservice specification
- [Examples](examples/) - Example usage patterns
- [API Documentation](docs/API.md) - Detailed tool reference
