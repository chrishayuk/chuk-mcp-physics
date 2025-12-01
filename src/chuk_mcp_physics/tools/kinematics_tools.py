"""Kinematics analysis MCP tool endpoints (Phase 3)."""

import json
from typing import Union

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def calculate_acceleration_from_position(
    times: Union[list[float], str],
    positions: Union[list[list[float]], str],
) -> dict:
    """Calculate acceleration by numerical differentiation of position data.

    Uses central differences for numerical differentiation:
    v[i] ≈ (r[i+1] - r[i-1]) / (2Δt)
    a[i] ≈ (v[i+1] - v[i-1]) / (2Δt)

    Args:
        times: Time values in seconds (or JSON string)
        positions: Position vectors [[x,y,z], ...] in meters (or JSON string)

    Returns:
        Dict containing:
            - velocities: Velocity vectors [[x,y,z], ...] in m/s
            - accelerations: Acceleration vectors [[x,y,z], ...] in m/s²
            - average_velocity: Average velocity [x,y,z] in m/s
            - average_acceleration: Average acceleration [x,y,z] in m/s²

    Example - Analyze recorded position data:
        result = await calculate_acceleration_from_position(
            times=[0, 1, 2, 3],
            positions=[[0,0,0], [5,0,0], [10,0,0], [15,0,0]]
        )
    """
    from ..kinematics import (
        AccelerationFromPositionRequest,
        calculate_acceleration_from_position as calc_accel,
    )

    # Parse inputs
    parsed_times = json.loads(times) if isinstance(times, str) else times
    parsed_positions = json.loads(positions) if isinstance(positions, str) else positions

    request = AccelerationFromPositionRequest(
        times=parsed_times,
        positions=parsed_positions,
    )
    response = calc_accel(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_jerk(
    times: Union[list[float], str],
    accelerations: Union[list[list[float]], str],
) -> dict:
    """Calculate jerk (rate of change of acceleration).

    Jerk = da/dt is important for comfort in vehicles and mechanical design.

    Args:
        times: Time values in seconds (or JSON string)
        accelerations: Acceleration vectors [[x,y,z], ...] in m/s² (or JSON string)

    Returns:
        Dict containing:
            - jerks: Jerk vectors [[x,y,z], ...] in m/s³
            - average_jerk: Average jerk [x,y,z] in m/s³
            - max_jerk_magnitude: Maximum jerk magnitude in m/s³

    Example:
        result = await calculate_jerk(
            times=[0, 1, 2, 3],
            accelerations=[[0,0,0], [2,0,0], [4,0,0], [6,0,0]]
        )
        # jerk_x ≈ 2 m/s³ (constant)
    """
    from ..kinematics import JerkCalculationRequest, calculate_jerk as calc_jerk

    # Parse inputs
    parsed_times = json.loads(times) if isinstance(times, str) else times
    parsed_accelerations = (
        json.loads(accelerations) if isinstance(accelerations, str) else accelerations
    )

    request = JerkCalculationRequest(
        times=parsed_times,
        accelerations=parsed_accelerations,
    )
    response = calc_jerk(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def fit_trajectory(
    times: Union[list[float], str],
    positions: Union[list[list[float]], str],
    fit_type: str = "quadratic",
) -> dict:
    """Fit polynomial to trajectory data.

    Useful for smoothing noisy data or finding trajectory equations.
    Default fit_type="quadratic" fits parabolic trajectory (constant acceleration).

    Args:
        times: Time values in seconds (or JSON string)
        positions: Position vectors [[x,y,z], ...] in meters (or JSON string)
        fit_type: Polynomial type - "linear", "quadratic", or "cubic" (default "quadratic")

    Returns:
        Dict containing:
            - coefficients_x: Polynomial coefficients for x(t)
            - coefficients_y: Polynomial coefficients for y(t)
            - coefficients_z: Polynomial coefficients for z(t)
            - r_squared: R² goodness of fit (0-1)
            - predicted_positions: Fitted positions [[x,y,z], ...]

    Example - Projectile motion:
        result = await fit_trajectory(
            times=[0, 1, 2, 3],
            positions=[[0,0,0], [10,15,0], [20,20,0], [30,15,0]],
            fit_type="quadratic"
        )
        # Fits x(t) = c0 + c1*t + c2*t²
    """
    from ..kinematics import TrajectoryFitRequest, fit_trajectory as fit_traj

    # Parse inputs
    parsed_times = json.loads(times) if isinstance(times, str) else times
    parsed_positions = json.loads(positions) if isinstance(positions, str) else positions

    request = TrajectoryFitRequest(
        times=parsed_times,
        positions=parsed_positions,
        fit_type=fit_type,  # type: ignore
    )
    response = fit_traj(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def generate_motion_graph(
    times: Union[list[float], str],
    positions: Union[list[list[float]], str],
    component: str = "magnitude",
) -> dict:
    """Generate motion graph data (position, velocity, acceleration vs time).

    Calculates velocity and acceleration from position data and extracts
    the specified component for graphing.

    Args:
        times: Time values in seconds (or JSON string)
        positions: Position vectors [[x,y,z], ...] in meters (or JSON string)
        component: Which component to analyze - "x", "y", "z", or "magnitude" (default)

    Returns:
        Dict containing:
            - times: Time values
            - positions: Position values (selected component)
            - velocities: Velocity values (selected component)
            - accelerations: Acceleration values (selected component)
            - max_velocity: Maximum velocity magnitude
            - max_acceleration: Maximum acceleration magnitude
            - component: Which component was analyzed

    Example:
        result = await generate_motion_graph(
            times=[0, 1, 2, 3],
            positions=[[0,0,0], [5,0,0], [20,0,0], [45,0,0]],
            component="x"
        )
        # Automatically calculates v and a
    """
    from ..kinematics import MotionGraphRequest, generate_motion_graph as gen_graph

    # Parse inputs
    parsed_times = json.loads(times) if isinstance(times, str) else times
    parsed_positions = json.loads(positions) if isinstance(positions, str) else positions

    request = MotionGraphRequest(
        times=parsed_times,
        positions=parsed_positions,
        component=component,  # type: ignore
    )
    response = gen_graph(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_average_speed(
    positions: Union[list[list[float]], str],
    times: Union[list[float], str],
) -> dict:
    """Calculate average speed along a path.

    Average speed = total distance / total time
    (Distance is path length, not displacement)

    Args:
        positions: Position vectors [[x,y,z], ...] in meters (or JSON string)
        times: Time values in seconds (or JSON string)

    Returns:
        Dict containing:
            - average_speed: Average speed in m/s
            - total_distance: Total path length in meters
            - total_time: Total elapsed time in seconds
            - displacement_magnitude: Straight-line displacement in meters
            - displacement: Displacement vector [x,y,z] in meters

    Example - Car on winding road:
        result = await calculate_average_speed(
            positions=[[0,0,0], [10,5,0], [20,10,0], [15,20,0]],
            times=[0, 10, 20, 30]
        )
    """
    from ..kinematics import AverageSpeedRequest, calculate_average_speed as calc_avg

    # Parse inputs
    parsed_positions = json.loads(positions) if isinstance(positions, str) else positions
    parsed_times = json.loads(times) if isinstance(times, str) else times

    request = AverageSpeedRequest(
        positions=parsed_positions,
        times=parsed_times,
    )
    response = calc_avg(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_instantaneous_velocity(
    positions: Union[list[list[float]], str],
    times: Union[list[float], str],
    target_time: float,
) -> dict:
    """Calculate instantaneous velocity at a specific time.

    Uses interpolation if target_time is between data points,
    otherwise uses numerical differentiation.

    Args:
        positions: Position vectors [[x,y,z], ...] in meters (or JSON string)
        times: Time values in seconds (or JSON string)
        target_time: Time at which to calculate velocity in seconds

    Returns:
        Dict containing:
            - velocity: Velocity vector [x,y,z] in m/s
            - speed: Speed magnitude in m/s
            - interpolated: Whether interpolation was used
            - time: Target time (echo)

    Example:
        result = await calculate_instantaneous_velocity(
            positions=[[0,0,0], [3,4,0], [6,8,0]],
            times=[0, 1, 2],
            target_time=1.0
        )
        # speed = 5 m/s
    """
    from ..kinematics import (
        InstantaneousVelocityRequest,
        calculate_instantaneous_velocity as calc_inst_vel,
    )

    # Parse inputs
    parsed_positions = json.loads(positions) if isinstance(positions, str) else positions
    parsed_times = json.loads(times) if isinstance(times, str) else times

    request = InstantaneousVelocityRequest(
        positions=parsed_positions,
        times=parsed_times,
        target_time=target_time,
    )
    response = calc_inst_vel(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_projectile_with_drag(
    initial_velocity: float,
    angle_degrees: float,
    mass: float,
    cross_sectional_area: float,
    initial_height: float = 0.0,
    drag_coefficient: float = 0.47,
    fluid_density: float = 1.225,
    gravity: float = 9.81,
    time_step: float = 0.01,
    max_time: float = 30.0,
    spin_rate: float = 0.0,
    spin_axis: Union[list[float], str] = "[0, 0, 1]",
    wind_velocity: Union[list[float], str] = "[0, 0]",
    altitude: float = 0.0,
    temperature: float = 15.0,
) -> dict:
    """Calculate projectile motion including air resistance (drag).

    Uses numerical integration (RK4) to solve motion equations with:
    - Quadratic drag force: F_drag = 0.5 * ρ * v² * Cd * A
    - Magnus force (spin effects): F_magnus = 0.5 * ρ * Cl * A * ω * r * v
    - Wind effects (constant wind vector)
    - Variable air density (altitude and temperature effects)

    This provides REALISTIC trajectories for sports balls, projectiles,
    and other objects moving through air or water. Compare with
    calculate_projectile_motion (no drag) to see dramatic differences!

    Common drag coefficients (Cd):
        - Sphere: 0.47 (default)
        - Baseball: 0.4
        - Golf ball: 0.25 (dimples reduce drag)
        - Football (American): 0.05-0.15 (orientation-dependent)
        - Basketball: 0.55
        - Soccer ball: 0.25
        - Skydiver (belly-down): 1.0-1.3
        - Streamlined car: 0.25-0.35

    Args:
        initial_velocity: Launch velocity in m/s
        angle_degrees: Launch angle in degrees (0-90)
        mass: Object mass in kg
        cross_sectional_area: Cross-section perpendicular to motion in m²
        initial_height: Launch height in meters (default 0)
        drag_coefficient: Drag coefficient Cd (default 0.47 for sphere)
        fluid_density: Fluid density in kg/m³ (air=1.225, water=1000)
        gravity: Gravitational acceleration m/s² (default 9.81)
        time_step: Integration time step in seconds (default 0.01)
        max_time: Maximum simulation time in seconds (default 30)
        spin_rate: Spin rate in rad/s for Magnus force (default 0, no spin)
        spin_axis: Spin axis unit vector [x, y, z] (default [0, 0, 1] = vertical)
        wind_velocity: Wind velocity [vx, vy] in m/s (default [0, 0], no wind)
        altitude: Altitude above sea level in meters (default 0, affects air density)
        temperature: Air temperature in Celsius (default 15, affects air density)

    Returns:
        Dict containing:
            - max_height: Maximum altitude reached (m)
            - range: Horizontal distance traveled (m)
            - time_of_flight: Total flight time (s)
            - impact_velocity: Speed at landing (m/s)
            - impact_angle: Angle at landing (degrees below horizontal)
            - trajectory_points: [[x, y], ...] for plotting
            - energy_lost_to_drag: Energy dissipated by drag (J)
            - initial_kinetic_energy: Initial KE (J)
            - final_kinetic_energy: Final KE (J)
            - lateral_deflection: Lateral deflection from spin/wind (m)
            - magnus_force_max: Maximum Magnus force magnitude (N)
            - wind_drift: Total wind drift (m)
            - effective_air_density: Effective air density used (kg/m³)

    Example - Baseball curveball (2500 rpm backspin):
        result = await calculate_projectile_with_drag(
            initial_velocity=40.23,  # 90 mph
            angle_degrees=10,
            mass=0.145,
            cross_sectional_area=0.0043,
            drag_coefficient=0.4,
            spin_rate=261.8,  # 2500 rpm = 261.8 rad/s
            spin_axis=[0, 0, 1]  # Backspin (vertical axis)
        )
        # Backspin increases range and height!

    Example - Golf ball at altitude (Denver, 1600m):
        result = await calculate_projectile_with_drag(
            initial_velocity=70,
            angle_degrees=12,
            mass=0.0459,
            cross_sectional_area=0.00143,
            drag_coefficient=0.25,
            altitude=1600,  # Denver elevation
            temperature=20  # Summer day
        )
        # Less air resistance = longer drive!

    Example - Soccer free kick with wind:
        result = await calculate_projectile_with_drag(
            initial_velocity=25,
            angle_degrees=15,
            mass=0.43,
            cross_sectional_area=0.0388,
            drag_coefficient=0.25,
            wind_velocity=[5, 0],  # 5 m/s tailwind
            spin_rate=50,  # Sidespin for curve
            spin_axis=[0, 1, 0]  # Horizontal axis
        )
        # Wind drift + Magnus curve!
    """
    from ..models import ProjectileWithDragRequest
    from ..kinematics import calculate_projectile_with_drag as calc_drag

    # Parse JSON strings if provided
    import json

    parsed_spin_axis = json.loads(spin_axis) if isinstance(spin_axis, str) else spin_axis
    parsed_wind_velocity = (
        json.loads(wind_velocity) if isinstance(wind_velocity, str) else wind_velocity
    )

    request = ProjectileWithDragRequest(
        initial_velocity=initial_velocity,
        angle_degrees=angle_degrees,
        initial_height=initial_height,
        mass=mass,
        drag_coefficient=drag_coefficient,
        cross_sectional_area=cross_sectional_area,
        fluid_density=fluid_density,
        gravity=gravity,
        time_step=time_step,
        max_time=max_time,
        spin_rate=spin_rate,
        spin_axis=parsed_spin_axis,
        wind_velocity=parsed_wind_velocity,
        altitude=altitude,
        temperature=temperature,
    )

    response = calc_drag(request)
    return response.model_dump()
