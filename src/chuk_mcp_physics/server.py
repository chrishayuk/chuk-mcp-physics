"""Physics MCP Server.

Provides physics calculations and simulations through configurable providers:
- Analytic: Closed-form equations for simple physics problems
- Rapier: Full rigid-body physics simulation using Rapier engine

Features:
- Projectile motion calculations
- Collision detection and prediction
- Force, energy, and momentum calculations
- Full 3D rigid-body simulations with Rapier
- Trajectory recording and R3F-compatible output

All responses use Pydantic models for type safety and validation.
No dictionary goop, no magic strings - everything is strongly typed with enums and constants.
"""

import logging
import sys
from typing import Optional

from chuk_mcp_server import run, tool

from .models import (
    CollisionCheckRequest,
    CollisionCheckResponse,
    ForceCalculationRequest,
    ForceCalculationResponse,
    KineticEnergyRequest,
    KineticEnergyResponse,
    MomentumRequest,
    MomentumResponse,
    ProjectileMotionRequest,
    ProjectileMotionResponse,
    RigidBodyDefinition,
    SimulationConfig,
    SimulationCreateResponse,
    SimulationStepResponse,
    TrajectoryResponse,
)
from .providers.factory import get_provider_for_tool

# Configure logging
# In STDIO mode, we need to be quiet to avoid polluting the JSON-RPC stream
# Only log to stderr, and only warnings/errors
logging.basicConfig(
    level=logging.WARNING, format="%(levelname)s:%(name)s:%(message)s", stream=sys.stderr
)
logger = logging.getLogger(__name__)


# ============================================================================
# Analytic Calculation Tools
# ============================================================================


@tool  # type: ignore[arg-type]
async def calculate_projectile_motion(
    initial_velocity: float,
    angle_degrees: float,
    initial_height: float = 0.0,
    gravity: float = 9.81,
) -> ProjectileMotionResponse:
    """Calculate projectile motion trajectory using kinematic equations.

    Computes the complete trajectory of a projectile launched at an angle,
    including maximum height, range, time of flight, and sample trajectory points.
    Perfect for ballistics, sports analysis, or educational demonstrations.

    Args:
        initial_velocity: Initial velocity in meters per second (m/s). Must be positive.
        angle_degrees: Launch angle in degrees from horizontal (0-90).
            0° = horizontal, 45° = maximum range, 90° = straight up
        initial_height: Initial height above ground in meters. Default 0.0 (ground level).
        gravity: Gravitational acceleration in m/s². Default 9.81 (Earth surface).
            Use 1.62 for Moon, 3.71 for Mars, etc.

    Returns:
        ProjectileMotionResponse containing:
            - max_height: Maximum height reached (meters)
            - range: Horizontal distance traveled (meters)
            - time_of_flight: Total time in air (seconds)
            - trajectory_points: List of [x, y] sample points for plotting

    Tips for LLMs:
        - 45° gives maximum range on flat ground (no air resistance)
        - For R3F visualization: convert trajectory_points to 3D by adding z=0
        - trajectory_points are evenly spaced in time (50 samples)
        - Air resistance is NOT modeled - this is ideal ballistic motion
        - Use for: cannon balls, baseballs, basketball shots, water fountains

    Example:
        # Calculate trajectory of a cannonball fired at 50 m/s at 30°
        result = await calculate_projectile_motion(
            initial_velocity=50.0,
            angle_degrees=30.0,
            initial_height=2.0
        )
        print(f"Range: {result.range:.1f}m, Max height: {result.max_height:.1f}m")
    """
    request = ProjectileMotionRequest(
        initial_velocity=initial_velocity,
        angle_degrees=angle_degrees,
        initial_height=initial_height,
        gravity=gravity,
    )
    provider = get_provider_for_tool("projectile_motion")
    return await provider.calculate_projectile_motion(request)


@tool  # type: ignore[arg-type]
async def check_collision(
    body1_position: list[float],
    body1_velocity: list[float],
    body1_radius: float,
    body2_position: list[float],
    body2_velocity: list[float],
    body2_radius: float,
    max_time: float = 10.0,
) -> CollisionCheckResponse:
    """Check if two moving spherical objects will collide.

    Predicts whether two moving spheres will collide within a time window,
    and if so, calculates when and where the collision occurs. Uses analytic
    relative motion to find exact collision time (if any).

    Args:
        body1_position: Position of first object [x, y, z] in meters
        body1_velocity: Velocity of first object [x, y, z] in m/s
        body1_radius: Radius of first object in meters (must be positive)
        body2_position: Position of second object [x, y, z] in meters
        body2_velocity: Velocity of second object [x, y, z] in m/s
        body2_radius: Radius of second object in meters (must be positive)
        max_time: Maximum time to check in seconds. Default 10.0.

    Returns:
        CollisionCheckResponse containing:
            - will_collide: True if collision will occur
            - collision_time: Time until collision in seconds (if collision occurs)
            - collision_point: Approximate collision location [x, y, z] (if collision occurs)
            - impact_speed: Relative velocity at impact in m/s (if collision occurs)
            - closest_approach_distance: Minimum distance between objects
            - closest_approach_time: Time of closest approach

    Tips for LLMs:
        - Objects are modeled as spheres (point masses with radius)
        - Collision detection is exact for constant velocity motion
        - Returns earliest collision time if multiple intersections
        - If no collision, check closest_approach_distance to see how close they get
        - Use for: asteroid tracking, car crash prediction, sports ball interactions
        - For complex shapes or forces, use create_simulation instead

    Example:
        # Check if two cars will collide
        result = await check_collision(
            body1_position=[0.0, 0.0, 0.0],
            body1_velocity=[10.0, 0.0, 0.0],
            body1_radius=2.0,
            body2_position=[50.0, 1.0, 0.0],
            body2_velocity=[-8.0, 0.0, 0.0],
            body2_radius=2.0
        )
        if result.will_collide:
            print(f"Collision in {result.collision_time:.2f} seconds at {result.impact_speed:.1f} m/s")
    """
    request = CollisionCheckRequest(
        body1_position=body1_position,
        body1_velocity=body1_velocity,
        body1_radius=body1_radius,
        body2_position=body2_position,
        body2_velocity=body2_velocity,
        body2_radius=body2_radius,
        max_time=max_time,
    )
    provider = get_provider_for_tool("collision_check")
    return await provider.check_collision(request)


@tool  # type: ignore[arg-type]
async def calculate_force(
    mass: float,
    acceleration_x: float,
    acceleration_y: float,
    acceleration_z: float,
) -> ForceCalculationResponse:
    """Calculate force from mass and acceleration using Newton's Second Law (F = ma).

    Computes the force vector required to produce a given acceleration on a mass.
    Fundamental for dynamics, engineering, and understanding motion.

    Args:
        mass: Mass in kilograms (must be positive)
        acceleration_x: X component of acceleration in m/s²
        acceleration_y: Y component of acceleration in m/s²
        acceleration_z: Z component of acceleration in m/s²

    Returns:
        ForceCalculationResponse containing:
            - force: Force vector [x, y, z] in Newtons
            - magnitude: Force magnitude in Newtons

    Tips for LLMs:
        - 1 Newton = force to accelerate 1 kg at 1 m/s²
        - On Earth, weight force = mass × 9.81 N (vertical)
        - Use magnitude to compare total force regardless of direction
        - Common accelerations: car braking ~10 m/s², elevator ~2 m/s²

    Example:
        # Force to accelerate a 1500kg car at 3 m/s² forward
        result = await calculate_force(
            mass=1500.0,
            acceleration_x=3.0,
            acceleration_y=0.0,
            acceleration_z=0.0
        )
        print(f"Required force: {result.magnitude:.0f} N")
    """
    request = ForceCalculationRequest(
        mass=mass,
        acceleration=[acceleration_x, acceleration_y, acceleration_z],
    )
    provider = get_provider_for_tool("force_calculation")
    return await provider.calculate_force(request)


@tool  # type: ignore[arg-type]
async def calculate_kinetic_energy(
    mass: float,
    velocity_x: float,
    velocity_y: float,
    velocity_z: float,
) -> KineticEnergyResponse:
    """Calculate kinetic energy from mass and velocity (KE = ½mv²).

    Computes the energy of motion for a moving object. Energy is scalar
    (direction doesn't matter, only speed). Useful for collision analysis,
    vehicle safety, and understanding energy transfer.

    Args:
        mass: Mass in kilograms (must be positive)
        velocity_x: X component of velocity in m/s
        velocity_y: Y component of velocity in m/s
        velocity_z: Z component of velocity in m/s

    Returns:
        KineticEnergyResponse containing:
            - kinetic_energy: Energy in Joules (J)
            - speed: Velocity magnitude in m/s

    Tips for LLMs:
        - 1 Joule = 1 kg⋅m²/s² = energy to lift 102g by 1m on Earth
        - Kinetic energy doubles mass → doubles energy, doubles speed → 4× energy
        - Car at highway speed (~30 m/s, 1500 kg) ≈ 675,000 J
        - Use to compare impact severity or stopping distances

    Example:
        # Energy of a 0.145kg baseball at 40 m/s
        result = await calculate_kinetic_energy(
            mass=0.145,
            velocity_x=40.0,
            velocity_y=0.0,
            velocity_z=0.0
        )
        print(f"Kinetic energy: {result.kinetic_energy:.1f} J")
    """
    request = KineticEnergyRequest(
        mass=mass,
        velocity=[velocity_x, velocity_y, velocity_z],
    )
    provider = get_provider_for_tool("kinetic_energy")
    return await provider.calculate_kinetic_energy(request)


@tool  # type: ignore[arg-type]
async def calculate_momentum(
    mass: float,
    velocity_x: float,
    velocity_y: float,
    velocity_z: float,
) -> MomentumResponse:
    """Calculate momentum from mass and velocity (p = mv).

    Computes the momentum vector, which represents "quantity of motion."
    Momentum is conserved in collisions, making it crucial for analyzing
    impacts, explosions, and rocket propulsion.

    Args:
        mass: Mass in kilograms (must be positive)
        velocity_x: X component of velocity in m/s
        velocity_y: Y component of velocity in m/s
        velocity_z: Z component of velocity in m/s

    Returns:
        MomentumResponse containing:
            - momentum: Momentum vector [x, y, z] in kg⋅m/s
            - magnitude: Momentum magnitude in kg⋅m/s

    Tips for LLMs:
        - Momentum is a vector (has direction), unlike kinetic energy
        - Total momentum before collision = total momentum after (conservation)
        - Large mass × small velocity can equal small mass × large velocity
        - Use to analyze: collisions, recoil, rocket thrust

    Example:
        # Momentum of a 70kg person running at 5 m/s
        result = await calculate_momentum(
            mass=70.0,
            velocity_x=5.0,
            velocity_y=0.0,
            velocity_z=0.0
        )
        print(f"Momentum: {result.magnitude:.1f} kg⋅m/s")
    """
    request = MomentumRequest(
        mass=mass,
        velocity=[velocity_x, velocity_y, velocity_z],
    )
    provider = get_provider_for_tool("momentum")
    return await provider.calculate_momentum(request)


# ============================================================================
# Simulation Tools (Rapier Provider Required)
# ============================================================================


@tool  # type: ignore[arg-type]
async def create_simulation(
    gravity_x: float = 0.0,
    gravity_y: float = -9.81,
    gravity_z: float = 0.0,
    dimensions: int = 3,
    dt: float = 0.016,
    integrator: str = "verlet",
) -> SimulationCreateResponse:
    """Create a new physics simulation using Rapier engine.

    Initializes a new rigid-body physics world with configurable gravity and
    timestep. Returns a simulation ID used for all subsequent operations.

    Args:
        gravity_x: X component of gravity vector (m/s²). Default 0.0
        gravity_y: Y component of gravity vector (m/s²). Default -9.81 (Earth down)
        gravity_z: Z component of gravity vector (m/s²). Default 0.0
        dimensions: 2 or 3 for 2D/3D simulation. Default 3.
        dt: Simulation timestep in seconds. Default 0.016 (60 FPS).
            Smaller = more accurate but slower, larger = faster but less stable
        integrator: Integration method. Options: "euler", "verlet", "rk4". Default "verlet".

    Returns:
        SimulationCreateResponse containing:
            - sim_id: Unique simulation identifier (use for all other sim calls)
            - config: Echo of the configuration used

    Tips for LLMs:
        - Keep simulation IDs in memory for the conversation session
        - Default gravity is Earth standard (9.81 m/s² down = -Y direction)
        - dt=0.016 ≈ 60 FPS, dt=0.008 ≈ 120 FPS (higher accuracy)
        - "verlet" integrator is good default (stable, energy-conserving)
        - Remember to destroy_simulation when done to free resources

    Requires:
        - Rapier provider must be configured (see config.py)
        - Rapier service must be running (see RAPIER_SERVICE.md)

    Example:
        # Create simulation with Earth gravity
        sim = await create_simulation(
            gravity_y=-9.81,
            dt=0.016
        )
        # Use sim.sim_id for add_body, step_simulation, etc.
    """
    config = SimulationConfig(
        gravity=[gravity_x, gravity_y, gravity_z],
        dimensions=dimensions,
        dt=dt,
        integrator=integrator,  # type: ignore
    )
    provider = get_provider_for_tool("create_simulation")
    return await provider.create_simulation(config)


@tool  # type: ignore[arg-type]
async def add_rigid_body(
    sim_id: str,
    body_id: str,
    body_type: str,
    shape: str,
    mass: float = 1.0,
    size: Optional[list[float]] = None,
    radius: Optional[float] = None,
    half_height: Optional[float] = None,
    position: Optional[list[float]] = None,
    orientation: Optional[list[float]] = None,
    velocity: Optional[list[float]] = None,
    angular_velocity: Optional[list[float]] = None,
    restitution: float = 0.5,
    friction: float = 0.5,
    is_sensor: bool = False,
) -> str:
    """Add a rigid body to an existing simulation.

    Creates a new physics body (static, dynamic, or kinematic) with specified
    shape, mass, and initial conditions. Bodies interact via collisions.

    Args:
        sim_id: Simulation ID from create_simulation
        body_id: Unique identifier for this body (user-defined string)
        body_type: "static", "dynamic", or "kinematic"
            - static: Never moves (ground, walls)
            - dynamic: Affected by forces and collisions
            - kinematic: Moves but not affected by forces (scripted motion)
        shape: Collider shape: "box", "sphere", "capsule", "cylinder", "plane"
        mass: Mass in kilograms (for dynamic bodies). Default 1.0
        size: For box: [width, height, depth]. For other shapes, use radius/half_height
        radius: Radius for sphere/capsule/cylinder
        half_height: Half-height for capsule/cylinder
        position: Initial position [x, y, z]. Default [0, 0, 0]
        orientation: Initial orientation quaternion [x, y, z, w]. Default [0, 0, 0, 1] (identity)
        velocity: Initial linear velocity [x, y, z]. Default [0, 0, 0]
        angular_velocity: Initial angular velocity [x, y, z]. Default [0, 0, 0]
        restitution: Bounciness (0.0 = no bounce, 1.0 = perfect bounce). Default 0.5
        friction: Surface friction (0.0 = ice, 1.0 = rubber). Default 0.5
        is_sensor: If true, detects collisions but doesn't respond physically. Default false

    Returns:
        body_id (echo of the input ID)

    Tips for LLMs:
        - Create ground first: body_type="static", shape="plane"
        - Box size is full width/height/depth (not half-extents)
        - Quaternions: identity = [0, 0, 0, 1] (no rotation)
        - Common restitution: steel=0.8, wood=0.5, clay=0.1
        - Common friction: ice=0.05, wood=0.4, rubber=1.0

    Example:
        # Add a ground plane
        await add_rigid_body(
            sim_id=sim_id,
            body_id="ground",
            body_type="static",
            shape="plane"
        )

        # Add a falling box
        await add_rigid_body(
            sim_id=sim_id,
            body_id="box",
            body_type="dynamic",
            shape="box",
            size=[1.0, 1.0, 1.0],
            mass=10.0,
            position=[0.0, 5.0, 0.0]
        )
    """
    body = RigidBodyDefinition(
        id=body_id,
        kind=body_type,  # type: ignore
        shape=shape,  # type: ignore
        mass=mass,
        size=size,
        radius=radius,
        half_height=half_height,
        position=position or [0.0, 0.0, 0.0],
        orientation=orientation or [0.0, 0.0, 0.0, 1.0],
        velocity=velocity or [0.0, 0.0, 0.0],
        angular_velocity=angular_velocity or [0.0, 0.0, 0.0],
        restitution=restitution,
        friction=friction,
        is_sensor=is_sensor,
    )
    provider = get_provider_for_tool("add_body")
    return await provider.add_body(sim_id, body)


@tool  # type: ignore[arg-type]
async def step_simulation(
    sim_id: str,
    steps: int = 1,
    dt: Optional[float] = None,
) -> SimulationStepResponse:
    """Step the simulation forward in time.

    Advances the physics simulation by running the integrator for N steps.
    Returns the complete state of all bodies after stepping.

    Args:
        sim_id: Simulation ID
        steps: Number of timesteps to simulate. Default 1.
            Example: steps=600 with dt=0.016 = 9.6 seconds of simulation
        dt: Optional timestep override (seconds). If None, uses config default.

    Returns:
        SimulationStepResponse containing:
            - sim_id: Simulation identifier
            - time: Current simulation time in seconds
            - bodies: List of all body states with positions, velocities, contacts

    Tips for LLMs:
        - Each body state includes position, orientation (quaternion), velocities
        - contacts array shows active collisions with impulse magnitudes
        - For real-time preview: steps=1, call repeatedly
        - For final result: steps=1000+, call once
        - Large step counts may timeout - limit to ~10,000 steps per call

    Example:
        # Simulate 10 seconds at 60 FPS
        result = await step_simulation(
            sim_id=sim_id,
            steps=600  # 600 steps × 0.016s = 9.6s
        )
        for body in result.bodies:
            print(f"{body.id}: position={body.position}")
    """
    provider = get_provider_for_tool("step_simulation")
    return await provider.step_simulation(sim_id, steps, dt)


@tool  # type: ignore[arg-type]
async def record_trajectory(
    sim_id: str,
    body_id: str,
    steps: int,
    dt: Optional[float] = None,
) -> TrajectoryResponse:
    """Record the trajectory of a specific body over time.

    Steps the simulation and records position/orientation/velocity at each
    timestep for one body. Perfect for generating animation data for R3F.

    Args:
        sim_id: Simulation ID
        body_id: ID of the body to track
        steps: Number of timesteps to record
        dt: Optional timestep override. If None, uses config default.

    Returns:
        TrajectoryResponse containing:
            - body_id: Tracked body identifier
            - frames: List of trajectory frames with time, position, orientation, velocity
            - total_time: Total simulated time in seconds
            - num_frames: Number of frames recorded

    Tips for LLMs:
        - Each frame has: time, position [x,y,z], orientation [x,y,z,w], velocity [x,y,z]
        - Frames are evenly spaced in time (every dt seconds)
        - Output is R3F-compatible: use position/orientation directly in Three.js
        - For 60 FPS video: record at dt=1/60 ≈ 0.0167
        - Typical recording: 100-1000 frames (1.6-16 seconds at 60 FPS)

    Example:
        # Record 5 seconds of a falling ball
        traj = await record_trajectory(
            sim_id=sim_id,
            body_id="ball",
            steps=300  # 300 × 0.016 ≈ 5 seconds
        )
        # Use traj.frames in React Three Fiber for animation
    """
    provider = get_provider_for_tool("record_trajectory")
    return await provider.record_trajectory(sim_id, body_id, steps, dt)


@tool  # type: ignore[arg-type]
async def destroy_simulation(sim_id: str) -> str:
    """Destroy a simulation and free resources.

    Cleanup when done with a simulation. Important for long-running servers
    to avoid memory leaks.

    Args:
        sim_id: Simulation ID to destroy

    Returns:
        Success message

    Tips for LLMs:
        - Always destroy simulations when conversation ends or changes topic
        - Rapier service keeps simulations in memory until explicitly destroyed
        - Good practice: destroy after recording trajectory or final state

    Example:
        await destroy_simulation(sim_id)
    """
    provider = get_provider_for_tool("destroy_simulation")
    await provider.destroy_simulation(sim_id)
    return f"Simulation {sim_id} destroyed"


# ============================================================================
# Main Entry Point
# ============================================================================


def main() -> None:
    """Run the Physics MCP server.

    Supports two transport modes:
    - stdio: For MCP clients (default) - Claude Desktop, mcp-cli
    - http: For web/REST access (use --http or http argument)
    """
    # Check if transport is specified in command line args
    # Default to stdio for MCP compatibility (Claude Desktop, mcp-cli)
    transport = "stdio"

    # Allow HTTP mode via command line
    if len(sys.argv) > 1 and sys.argv[1] in ["http", "--http"]:
        transport = "http"
        # Only log in HTTP mode
        logger.warning("Starting Physics MCP Server in HTTP mode")

    # Suppress chuk_mcp_server logging in STDIO mode
    if transport == "stdio":
        # Set chuk_mcp_server loggers to ERROR only
        logging.getLogger("chuk_mcp_server").setLevel(logging.ERROR)
        logging.getLogger("chuk_mcp_server.core").setLevel(logging.ERROR)
        logging.getLogger("chuk_mcp_server.stdio_transport").setLevel(logging.ERROR)
        # Suppress httpx logging (API calls to Rapier service)
        logging.getLogger("httpx").setLevel(logging.ERROR)

    # For HTTP mode, bind to 0.0.0.0 for container/cloud deployment
    if transport == "http":
        run(transport=transport, host="0.0.0.0", port=8000)
    else:
        run(transport=transport)


if __name__ == "__main__":
    main()
