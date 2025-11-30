# Physics MCP Server - Examples

This directory contains example scripts demonstrating various use cases for the physics MCP server.

## Running Examples

All examples use the **analytic provider** (no external dependencies required):

```bash
# Run individual examples
python examples/01_simple_projectile.py
python examples/02_collision_detection.py
python examples/03_force_energy_momentum.py
python examples/04_r3f_visualization.py
```

## Examples Overview

### 1. Simple Projectile Motion (`01_simple_projectile.py`)

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

## Extending Examples

These examples use the analytic provider directly. For **simulation-based examples** (Rapier), see:
- `05_rapier_simulation.py` (requires Rapier service)
- `06_multi_body_system.py` (requires Rapier service)

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

## Tips

1. **Modify parameters** in examples to explore different scenarios
2. **Save trajectory data** to JSON for visualization
3. **Combine examples** to create complex analyses
4. **Add visualization** using matplotlib or R3F
5. **Scale values** for different domains (space, microscopic, etc.)

Enjoy exploring physics with the MCP server! 🚀
