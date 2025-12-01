# Example Prompts for LLM Physics Simulations

This document demonstrates that an LLM can build complex physics simulations using only the MCP tools, without needing example code.

## 🎰 Roulette Wheel Simulation

### User Prompt (What You'd Say)

```
Create a realistic casino roulette wheel simulation. The wheel should have:
- A spinning wheel base (about 35cm radius)
- An outer rim to contain the ball
- Some deflector obstacles that the ball bounces off
- Numbered pockets (European roulette has 37 numbers: 0-36)
- A small ball (about 8mm, 6 grams) launched with initial velocity

The ball should:
- Start at the rim moving tangentially
- Bounce off deflectors as it spirals inward
- Slow down due to friction and air resistance
- Eventually settle into a numbered pocket

Run the simulation for about 12 seconds and tell me which number the ball lands on.
```

### What the LLM Would Do

**Step 1: Plan the structure**
```
I'll need to:
1. Create a physics world with gravity
2. Build the wheel structure (ground, base, rim)
3. Add deflector obstacles in a circle
4. Add pocket dividers in a smaller circle
5. Launch the ball with tangential velocity
6. Record the trajectory with bounce detection
7. Analyze the final position to determine the number
```

**Step 2: Create simulation and ground**
```python
# LLM would call these tools:
sim = await create_simulation(gravity_x=0.0, gravity_y=-9.81, gravity_z=0.0)

await add_rigid_body(
    sim_id=sim.sim_id,
    body_id="ground",
    body_type="static",
    shape="plane",
    position=[0.0, 0.0, 0.0],
    friction=0.8,
    restitution=0.3
)
```

**Step 3: Build wheel base**
```python
await add_rigid_body(
    sim_id=sim.sim_id,
    body_id="wheel_base",
    body_type="static",
    shape="cylinder",
    size=[0.35, 0.02],  # radius, height
    position=[0.0, 0.01, 0.0],
    friction=0.5,
    restitution=0.4
)
```

**Step 4: Create rim (LLM would calculate circle positions)**
```python
import math

num_rim_segments = 32
rim_radius = 0.42

for i in range(num_rim_segments):
    angle = (2 * math.pi * i) / num_rim_segments
    x = rim_radius * math.cos(angle)
    z = rim_radius * math.sin(angle)

    await add_rigid_body(
        sim_id=sim.sim_id,
        body_id=f"rim_{i}",
        body_type="static",
        shape="box",
        size=[0.04, 0.10, 0.04],
        position=[x, 0.05, z],
        friction=0.4,
        restitution=0.6
    )
```

**Step 5: Add deflectors**
```python
num_deflectors = 8
deflector_radius = 0.30

for i in range(num_deflectors):
    angle = (2 * math.pi * i) / num_deflectors
    x = deflector_radius * math.cos(angle)
    z = deflector_radius * math.sin(angle)

    await add_rigid_body(
        sim_id=sim.sim_id,
        body_id=f"deflector_{i}",
        body_type="static",
        shape="sphere",
        size=[0.015],
        position=[x, 0.03, z],
        friction=0.3,
        restitution=0.8  # Bouncy!
    )
```

**Step 6: Add pocket dividers**
```python
num_pockets = 37
pocket_radius = 0.20

for i in range(num_pockets):
    angle = (2 * math.pi * i) / num_pockets
    x = pocket_radius * math.cos(angle)
    z = pocket_radius * math.sin(angle)

    await add_rigid_body(
        sim_id=sim.sim_id,
        body_id=f"pocket_{i}",
        body_type="static",
        shape="box",
        size=[0.02, 0.03, 0.02],
        position=[x, 0.025, z],
        friction=0.9,  # High friction to trap ball
        restitution=0.2
    )
```

**Step 7: Launch ball**
```python
import random

# Random starting position on rim
initial_angle = random.uniform(0, 2 * math.pi)
initial_radius = 0.38

x = initial_radius * math.cos(initial_angle)
z = initial_radius * math.sin(initial_angle)

# Tangential velocity (perpendicular to radius)
speed = 1.5  # m/s
tangent_angle = initial_angle + math.pi / 2
vx = speed * math.cos(tangent_angle)
vz = speed * math.sin(tangent_angle)

await add_rigid_body(
    sim_id=sim.sim_id,
    body_id="ball",
    body_type="dynamic",
    shape="sphere",
    size=[0.008],  # 8mm diameter
    mass=0.006,     # 6 grams
    position=[x, 0.05, z],
    velocity=[vx, 0.0, vz],
    linear_damping=0.5,   # Air resistance
    angular_damping=0.3,  # Rotational friction
    friction=0.6,
    restitution=0.7
)
```

**Step 8: Run simulation and record**
```python
trajectory = await record_trajectory_with_events(
    sim_id=sim.sim_id,
    body_id="ball",
    steps=750,  # 12 seconds @ 60 FPS
    dt=0.016,
    detect_bounces=True
)
```

**Step 9: Analyze result**
```python
# Get final position
final_pos = trajectory.frames[-1].position
x, z = final_pos[0], final_pos[2]

# Calculate angle from center
angle = math.atan2(z, x)
if angle < 0:
    angle += 2 * math.pi

# Map to pocket number (37 pockets evenly distributed)
pocket_index = int((angle / (2 * math.pi)) * 37) % 37

# European roulette number sequence
NUMBERS = [0, 32, 15, 19, 4, 21, 2, 25, 17, 34, 6, 27, 13, 36, 11, 30, 8, 23, 10,
           5, 24, 16, 33, 1, 20, 14, 31, 9, 22, 18, 29, 7, 28, 12, 35, 3, 26]

winning_number = NUMBERS[pocket_index]

print(f"🎰 The ball landed on: {winning_number}")
print(f"Number of bounces: {len(trajectory.bounces)}")
print(f"Total distance: {calculate_distance(trajectory):.2f} meters")
```

**Step 10: Cleanup**
```python
await destroy_simulation(sim.sim_id)
```

### LLM Response

```
I've created a realistic roulette simulation with 79 rigid bodies!

🎰 THE BALL LANDED ON: 15

Physics Details:
- Simulation time: 12.0 seconds
- Ball bounces: 2 detected
- Average energy loss per bounce: 30.7%
- Total distance traveled: 1.90 meters
- Final position: Pocket 15 (RED, ODD, Low 1-18)

The simulation includes:
✓ Ground plane (prevents fall-through)
✓ Wheel base (35cm cylinder)
✓ 32 rim segments (circular wall)
✓ 8 deflectors (bounce obstacles)
✓ 37 pocket dividers (numbered slots)
✓ Realistic physics (friction, damping, gravity)

The ball started at the rim moving at 1.5 m/s, bounced off deflectors,
spiraled inward, and settled into pocket 15.

Would you like to run another spin or visualize the trajectory?
```

## Why This Works

### 1. **Composable Primitives**
The tools provide basic shapes (sphere, box, cylinder, plane) that can be combined to build complex structures.

### 2. **Physics Properties as Parameters**
Friction, restitution, damping, mass, velocity—all controllable per object.

### 3. **Stateless API**
No hidden state. Each tool call is explicit about what it does.

### 4. **Built-in Analysis**
Bounce detection and trajectory recording are built into the tools.

### 5. **Clear Documentation**
Tool parameters have obvious meanings (friction, restitution, position, velocity).

## Other Examples LLM Could Build

### Bowling Pin Strike
```
"Simulate a bowling ball hitting 10 pins arranged in a triangle.
Calculate which pins fall over."
```

LLM would:
- Create ground plane
- Add 10 cylindrical pins in triangle formation
- Add spherical bowling ball with velocity toward pins
- Record trajectories of all pins
- Analyze which pins tipped over (angle > threshold)

### Domino Chain Reaction
```
"Create a domino chain with 20 dominoes. Push the first one and
see if they all fall."
```

LLM would:
- Create 20 box-shaped dominoes in a line
- Tip the first domino slightly (initial rotation)
- Record trajectories
- Analyze contact events to see which dominoes hit which

### Basketball Free Throw
```
"Simulate a basketball free throw. The ball starts 2m high,
4.6m from the hoop (which is at 3.05m). What velocity and
angle should I use?"
```

LLM would:
- Use analytic `calculate_projectile_motion` tool
- Try various angles and velocities
- Find combinations that pass through (4.6, 3.05)
- Report optimal launch parameters

### Marble Maze
```
"Create a tilted board with obstacles. Drop a marble and see
where it exits."
```

LLM would:
- Create angled plane (ground)
- Add box obstacles
- Add sphere (marble) at starting position
- Record trajectory
- Report exit position

### Newton's Cradle
```
"Create a Newton's cradle with 5 balls. Pull back the first
ball and release it."
```

LLM would:
- Create 5 spheres in a line
- Use joints to suspend them (pendulums)
- Give first ball initial velocity
- Record all trajectories
- Show momentum transfer visualization

## Key Insight

**The LLM doesn't need example code!**

It only needs:
1. Understanding of the physical scenario
2. Basic geometry (circles, angles, vectors)
3. Tool documentation (parameters and what they do)
4. Physics intuition (friction slows things, bouncy = high restitution)

The tools are **general enough** that the LLM can figure out how to combine them for any physics scenario, yet **specific enough** that the LLM knows what each parameter controls.

This is the power of a well-designed API! 🚀
