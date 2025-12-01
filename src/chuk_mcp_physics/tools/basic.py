"""Basic physics MCP tool endpoints (force, energy, momentum, work, projectile)."""

import json
from typing import Union

from chuk_mcp_server import tool

from ..models import (
    ProjectileMotionRequest,
    ProjectileMotionResponse,
    CollisionCheckRequest,
    CollisionCheckResponse,
    ForceCalculationRequest,
    ForceCalculationResponse,
    KineticEnergyRequest,
    KineticEnergyResponse,
    MomentumRequest,
    MomentumResponse,
)
from ..providers.factory import get_provider_for_tool


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


@tool  # type: ignore[arg-type]
async def calculate_potential_energy(
    mass: float,
    height: float,
    gravity: float = 9.81,
) -> dict:
    """Calculate gravitational potential energy.

    Computes PE = mgh (mass × gravity × height).
    Also returns the equivalent velocity if the object falls from that height.

    Args:
        mass: Object mass in kilograms
        height: Height above reference point in meters
        gravity: Gravitational acceleration in m/s² (default 9.81 for Earth)

    Returns:
        Dict containing:
            - potential_energy: PE in Joules
            - equivalent_kinetic_velocity: Speed if dropped from height (m/s)

    Example - Object at 10m height:
        result = await calculate_potential_energy(mass=2.0, height=10.0)
        # PE = 196.2 J
        # Velocity if dropped = 14.0 m/s
    """
    provider = get_provider_for_tool("potential_energy")
    response = await provider.calculate_potential_energy(mass, height, gravity)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_work_power(
    force: Union[list[float], str],
    displacement: Union[list[float], str],
    time: float | None = None,
) -> dict:
    """Calculate work done by a force and optionally power.

    Work is the dot product: W = F · d
    Power (if time given): P = W / t

    Args:
        force: Force vector [x, y, z] in Newtons (or JSON string)
        displacement: Displacement vector [x, y, z] in meters (or JSON string)
        time: Time taken in seconds (optional, for power calculation)

    Returns:
        Dict containing:
            - work: Work done in Joules
            - power: Power in Watts (if time provided, else None)

    Example - Pushing box 5m with 100N force:
        result = await calculate_work_power(
            force=[100, 0, 0],
            displacement=[5, 0, 0],
            time=10.0
        )
        # Work = 500 J, Power = 50 W
    """
    # Parse inputs
    parsed_force = json.loads(force) if isinstance(force, str) else force
    parsed_disp = json.loads(displacement) if isinstance(displacement, str) else displacement

    provider = get_provider_for_tool("work_power")
    response = await provider.calculate_work_power(parsed_force, parsed_disp, time)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_elastic_collision(
    mass1: float,
    velocity1: float,
    mass2: float,
    velocity2: float,
) -> dict:
    """Calculate final velocities after a 1D elastic collision.

    Uses conservation of momentum and energy to solve for final velocities.
    Assumes perfectly elastic collision (no energy loss).

    Args:
        mass1: Mass of first object in kg
        velocity1: Initial velocity of first object in m/s (1D)
        mass2: Mass of second object in kg
        velocity2: Initial velocity of second object in m/s (1D)

    Returns:
        Dict containing:
            - final_velocity1: Final velocity of object 1 in m/s
            - final_velocity2: Final velocity of object 2 in m/s
            - initial_kinetic_energy: Total KE before (J)
            - final_kinetic_energy: Total KE after (J) - should equal initial
            - initial_momentum: Total momentum before (kg⋅m/s)
            - final_momentum: Total momentum after (kg⋅m/s) - should equal initial

    Example - Pool ball collision:
        result = await calculate_elastic_collision(
            mass1=0.17,      # kg (pool ball)
            velocity1=2.0,   # m/s (moving right)
            mass2=0.17,      # kg (pool ball)
            velocity2=0.0    # m/s (stationary)
        )
        # Result: ball 1 stops, ball 2 moves at 2.0 m/s
    """
    provider = get_provider_for_tool("elastic_collision")
    response = await provider.calculate_elastic_collision(mass1, velocity1, mass2, velocity2)
    return response.model_dump()
