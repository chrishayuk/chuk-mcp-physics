# Physics MCP Server - Examples

This directory contains example scripts demonstrating various use cases for the physics MCP server.

## Running Examples

### Phase 1 Examples (Core Physics)

Examples 00-04 use the **analytic provider** (no external dependencies):

```bash
# Core physics examples
python examples/00_quick_start.py
python examples/01_simple_projectile.py
python examples/02_collision_detection.py
python examples/03_force_energy_momentum.py
python examples/04_r3f_visualization.py
```

Examples 05-10 require the **Rapier service**:

```bash
# Set up Rapier service
export RAPIER_SERVICE_URL=https://rapier.chukai.io

# Simulation examples
python examples/05_rapier_simulation.py
python examples/06_bounce_detection.py
python examples/07_contact_events.py
python examples/08_pendulum.py
python examples/09_phase1_complete.py
python examples/10_fluid_dynamics.py
```

### Phase 2 Examples (Advanced Physics)

Examples 11-17 use the **analytic provider** (no external dependencies):

```bash
# Advanced physics examples
python examples/11_rotational_dynamics.py
python examples/12_oscillations.py
python examples/13_circular_motion.py
python examples/14_statics.py
python examples/15_kinematics_analysis.py
python examples/17_viscosity_and_reynolds_number.py
```

## Examples Overview

### Phase 1: Core Physics (Examples 00-10)

#### 1. Simple Projectile Motion (`01_simple_projectile.py`)

Demonstrates basic projectile motion calculations:
- **Cannonball trajectory** at 45° (maximum range)
- **Basketball free throw** simulation with height checking
- **Angle comparison** showing how launch angle affects range

**Output:**
- Maximum height, range, time of flight
- Trajectory sample points
- Comparison table for different angles

**Use cases:**
- Educational physics demonstrations
- Game development (ballistics)
- Sports analysis

---

### 2. Collision Detection (`02_collision_detection.py`)

Shows collision prediction between moving objects:
- **Head-on car collision** with impact prediction
- **Near miss scenario** in parallel lanes
- **Asteroid collision** tracking (space scale)
- **Multiple scenarios** comparison table

**Output:**
- Collision yes/no prediction
- Time to impact
- Collision point coordinates
- Impact speed
- Closest approach distance

**Use cases:**
- Crash avoidance systems
- Accident reconstruction
- Game physics
- Space debris tracking

---

### 3. Force, Energy, and Momentum (`03_force_energy_momentum.py`)

Fundamental physics calculations:
- **Force calculations** (F = ma) for cars, rockets, braking
- **Kinetic energy** (KE = ½mv²) for various speeds
- **Momentum** (p = mv) comparisons
- **Collision analysis** with energy loss calculation

**Output:**
- Force magnitudes in Newtons
- Energy values in Joules
- Momentum conservation demonstration
- Energy vs. speed relationship (quadratic)

**Use cases:**
- Engineering analysis
- Crash testing
- Physics education
- Safety calculations

---

### 4. React Three Fiber Visualization (`04_r3f_visualization.py`)

Generate 3D visualization data for React Three Fiber:
- **Basketball shot** with R3F component code
- **Multiple trajectories** comparison (different angles)
- **Trail markers** for time-based visualization
- **JSON export** for easy import into React

**Output:**
- Trajectory data in R3F-compatible format
- Complete React component examples
- JSON file (`basketball_trajectory.json`) ready to import
- Trail marker positions for path visualization

**Use cases:**
- 3D web visualizations
- Educational animations
- Game prototyping
- Scientific visualization

---

## Output Examples

### Projectile Motion
```
CANNONBALL TRAJECTORY CALCULATOR
================================================================

1. Cannonball fired at 45° (maximum range angle)
----------------------------------------------------------------
Initial velocity: 50.0 m/s
Launch angle: 45.0°

RESULTS:
  Maximum height: 63.7 m
  Range: 254.6 m
  Time of flight: 7.2 s
  Trajectory points: 51 samples
```

### Collision Detection
```
COLLISION DETECTION EXAMPLES
================================================================

1. Head-on car collision
----------------------------------------------------------------
Car A: Position (0, 0, 0), Velocity (20, 0, 0) m/s
Car B: Position (100, 0, 0), Velocity (-25, 0, 0) m/s

RESULTS:
  Will collide: True
  ⚠️  COLLISION ALERT!
  Time to impact: 2.11 seconds
  Impact speed: 45.0 m/s (100.7 mph)
```

### R3F Data
```json
{
  "time": 0.0,
  "position": [0.0, 2.0, 0.0],
  "index": 0
},
{
  "time": 0.02,
  "position": [0.12, 2.06, 0.0],
  "index": 1
}
```

---

---

### Phase 2: Advanced Physics (Examples 11-15)

#### 11. Rotational Dynamics (`11_rotational_dynamics.py`)

Rotational motion and angular mechanics:
- **Torque calculations** - Wrenches, forces at distance
- **Moment of inertia** - Various shapes (disk, sphere, rod)
- **Angular momentum** - Conservation (figure skater example)
- **Rotational KE** - Flywheel energy storage
- **Angular acceleration** - Motor startup
- **Real-world** - Bicycle wheel gyroscopic effects

**Output:**
- Torque magnitudes and directions
- Moment of inertia for different shapes
- Angular momentum conservation demonstration
- Energy storage in rotating systems

**Use cases:**
- Mechanical engineering
- Robotics (joint torques)
- Sports science
- Aerospace (satellite attitude)

---

#### 12. Oscillations (`12_oscillations.py`)

Harmonic motion and spring systems:
- **Hooke's law** - Spring forces and energy
- **Spring-mass period** - Natural frequencies
- **Simple harmonic motion** - Position vs. time
- **Damped oscillations** - Underdamped, critical, overdamped
- **Pendulum motion** - Period calculations
- **Energy analysis** - KE ↔ PE oscillation

**Output:**
- Spring forces and potential energy
- Oscillation periods and frequencies
- Damping regime classification
- Energy transfer visualization

**Use cases:**
- Suspension design
- Seismology
- Clock mechanisms
- Vibration isolation

---

#### 13. Circular Motion (`13_circular_motion.py`)

Circular motion and orbital mechanics:
- **Centripetal force** - Car on curved road
- **Orbital mechanics** - ISS orbit analysis
- **Banking angles** - Highway curve design
- **Escape velocity** - Various celestial bodies
- **Geostationary orbits** - 24-hour satellites

**Output:**
- Required centripetal forces
- Orbital periods and velocities
- Optimal banking angles for speeds
- Escape velocities for planets

**Use cases:**
- Space mission planning
- Highway design
- Astrophysics
- Amusement park rides

---

#### 14. Statics & Equilibrium (`14_statics.py`)

Static equilibrium and structural analysis:
- **Force balance** - Bridge supports
- **Torque balance** - Seesaws and levers
- **Center of mass** - Multi-body systems
- **Static friction** - Will it slide?
- **Normal forces** - Inclined planes
- **Complete equilibrium** - Force + torque
- **Beam reactions** - Simply supported beams

**Output:**
- Force and torque equilibrium checks
- Center of mass locations
- Friction limits and slip predictions
- Support reactions for beams

**Use cases:**
- Structural engineering
- Architecture
- Mechanical design
- Safety analysis

---

#### 15. Kinematics Analysis (`15_kinematics_analysis.py`)

Motion data analysis and trajectory processing:
- **Acceleration from position** - Numerical differentiation
- **Jerk analysis** - Motion smoothness
- **Trajectory fitting** - Polynomial regression
- **Motion graphs** - Position/velocity/acceleration
- **Average speed** - Path length vs. displacement
- **Instantaneous velocity** - With interpolation

**Output:**
- Derived velocities and accelerations
- Jerk magnitudes (smoothness metrics)
- Fitted trajectory equations
- Graph data for visualization
- Speed and velocity calculations

**Use cases:**
- Motion capture analysis
- Robotics trajectory planning
- Sports biomechanics
- Autonomous vehicle testing

---

#### 17. Viscosity & Reynolds Number (`17_viscosity_and_reynolds_number.py`)

Demonstrates the viscosity parameter for accurate Reynolds number calculations:
- **Viscosity comparison** - Same ball through different fluids
- **Motor oil accuracy** - Why explicit viscosity matters (100x error without it!)
- **Temperature effects** - How temp changes flow behavior
- **Industrial fluids** - Hydraulic oil, glycerin, honey, molasses
- **Backwards compatibility** - Old code still works

**Output:**
- Reynolds numbers for various fluids
- Flow regime classification (laminar/transitional/turbulent)
- Comparison tables showing viscosity impact
- Temperature-dependent behavior
- Engineering insights on flow regimes

**Use cases:**
- Industrial process design
- Lubrication engineering
- Food processing (honey, syrup flows)
- Temperature-sensitive fluid analysis
- Accurate flow regime determination

**Key Insight:**
Without explicit viscosity, motor oil Reynolds number is **100x too high**, leading to completely wrong flow regime classification! This example shows how the new viscosity parameter fixes this limitation.

---

## Extending Examples

All Phase 1 examples (00-10) use the analytic provider. For **simulation-based examples** (Rapier), see:
- `05_rapier_simulation.py` (requires Rapier service)
- `06_bounce_detection.py` (requires Rapier service)
- `07_contact_events.py` (requires Rapier service)
- `08_pendulum.py` (requires Rapier service)
- `09_phase1_complete.py` (requires Rapier service)

All Phase 2 examples (11-17) use the analytic provider (no external dependencies).

---

## Integration with MCP

These examples call providers directly. In a real MCP scenario:

```python
# Instead of direct provider calls:
from chuk_mcp_physics.providers.analytic import AnalyticProvider
provider = AnalyticProvider()
result = await provider.calculate_projectile_motion(request)

# LLM calls via MCP tools:
# User: "How far does a ball go at 30 m/s, 45°?"
# LLM calls: calculate_projectile_motion(30, 45)
# Returns: ProjectileMotionResponse
```

---

## Additional Resources

- **Main README**: `../README.md` - Complete documentation
- **Rapier Service**: `../RAPIER_SERVICE.md` - Simulation setup
- **API Reference**: See tool docstrings in `../src/chuk_mcp_physics/server.py`

---

### 16. Casino Roulette Simulation (`16_roulette_simulation.py`) 🎰

**🌟 SHOWCASE EXAMPLE - Complete Physics Engine Demonstration**

A comprehensive casino roulette wheel simulation that demonstrates the full capabilities of the physics engine:

**Features Demonstrated:**
- **Spinning wheel** with rotational dynamics
- **Ball launch** with tangential velocity
- **Deflector bounces** (8 diamond obstacles)
- **Pocket divisions** (37 numbered pockets - European roulette)
- **Energy dissipation** through friction and damping
- **Realistic settling** behavior
- **Complete trajectory** recording with bounce detection
- **Result determination** based on final ball position

**Physics Concepts:**
- Rotational motion (spinning wheel at 40-60 RPM)
- Collision detection (ball hitting deflectors)
- Friction (wheel, rim, and pocket surfaces)
- Damping (air resistance, rotational friction)
- Energy loss per bounce
- Multi-body interactions (46+ bodies: wheel, rim, 8 deflectors, 37 pockets, ball)
- Complex geometry (cylinders, spheres, boxes)

**Output:**
- Winning number (0-36) and color (RED/BLACK/GREEN)
- Even/odd classification
- High/low range (1-18 or 19-36)
- Physics statistics (bounces, contacts, distance traveled)
- Energy dissipation analysis
- Full trajectory data for visualization

**Use Cases:**
- Game development (realistic casino simulations)
- Physics education (complex system dynamics)
- Visualization (3D casino table rendering)
- Probability analysis (physics + statistics)
- Engine capability demonstration

**Requires:**
- Rapier service
- ~12 seconds simulation time
- Multi-body physics support

```bash
export PHYSICS_PROVIDER=rapier
export RAPIER_SERVICE_URL=https://rapier.chukai.io
python examples/16_roulette_simulation.py
```

**Why This Example?**
This is the perfect showcase for demonstrating what the physics engine can do:
- ✅ Complex real-world scenario everyone understands
- ✅ Combines multiple physics concepts (rotation, collision, friction, damping)
- ✅ Visually impressive (46+ interacting bodies)
- ✅ Realistic behavior (ball bounces, slows, settles)
- ✅ Practical application (actual casino physics)
- ✅ Generates quantifiable results (specific number outcome)
- ✅ Ready for 3D visualization (full trajectory data)

---

## Tips

1. **Modify parameters** in examples to explore different scenarios
2. **Save trajectory data** to JSON for visualization
3. **Combine examples** to create complex analyses
4. **Add visualization** using matplotlib or R3F
5. **Scale values** for different domains (space, microscopic, etc.)

Enjoy exploring physics with the MCP server! 🚀
