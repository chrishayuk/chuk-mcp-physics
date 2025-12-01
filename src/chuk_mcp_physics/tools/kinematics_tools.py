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
