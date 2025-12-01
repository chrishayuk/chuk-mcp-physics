"""Conservation law verification MCP tool endpoints."""

import json
from typing import Union

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def check_energy_conservation(
    initial_kinetic_energy: float,
    final_kinetic_energy: float,
    initial_potential_energy: float,
    final_potential_energy: float,
    expected_energy_loss: float = 0.0,
    tolerance: float = 0.01,
) -> dict:
    """Verify conservation of energy in a physics process.

    Checks whether total mechanical energy is conserved (or correctly dissipated).
    Useful for validating simulation results and understanding energy transfer.

    Args:
        initial_kinetic_energy: Initial KE in Joules
        final_kinetic_energy: Final KE in Joules
        initial_potential_energy: Initial PE in Joules
        final_potential_energy: Final PE in Joules
        expected_energy_loss: Expected energy loss (from friction, etc.) in Joules
        tolerance: Tolerance for conservation check (fraction, default 0.01 = 1%)

    Returns:
        Dict containing:
            - initial_total_energy: Initial total energy in Joules
            - final_total_energy: Final total energy in Joules
            - energy_difference: Energy difference in Joules
            - energy_difference_percent: % difference
            - is_conserved: Whether energy is conserved within tolerance
            - expected_loss: Expected energy loss in Joules
            - actual_loss: Actual energy loss in Joules

    Tips for LLMs:
        - In isolated systems, total energy is conserved
        - With friction/damping, expect energy loss
        - Small numerical errors are normal in simulations
        - Use to validate simulation accuracy

    Example - Bouncing ball with energy loss:
        result = await check_energy_conservation(
            initial_kinetic_energy=0,
            final_kinetic_energy=0,
            initial_potential_energy=10,  # J (at 1m height)
            final_potential_energy=6.4,  # J (bounced to 0.64m)
            expected_energy_loss=3.6,  # 36% loss (e=0.8)
            tolerance=0.01
        )
    """
    from ..conservation import EnergyConservationCheckRequest, check_energy_conservation as check_E

    request = EnergyConservationCheckRequest(
        initial_kinetic_energy=initial_kinetic_energy,
        final_kinetic_energy=final_kinetic_energy,
        initial_potential_energy=initial_potential_energy,
        final_potential_energy=final_potential_energy,
        expected_energy_loss=expected_energy_loss,
        tolerance=tolerance,
    )
    response = check_E(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def check_momentum_conservation(
    initial_momentum: Union[list[float], str],
    final_momentum: Union[list[float], str],
    tolerance: float = 0.01,
) -> dict:
    """Verify conservation of momentum.

    Checks whether total momentum is conserved in a collision or interaction.
    Momentum should be conserved in isolated systems (no external forces).

    Args:
        initial_momentum: Initial total momentum [x, y, z] in kg⋅m/s (or JSON string)
        final_momentum: Final total momentum [x, y, z] in kg⋅m/s (or JSON string)
        tolerance: Tolerance for conservation check (fraction, default 0.01 = 1%)

    Returns:
        Dict containing:
            - initial_momentum_magnitude: Initial |p| in kg⋅m/s
            - final_momentum_magnitude: Final |p| in kg⋅m/s
            - momentum_difference: Difference [x, y, z]
            - momentum_difference_magnitude: |Δp|
            - momentum_difference_percent: % difference
            - is_conserved: Whether momentum is conserved within tolerance

    Tips for LLMs:
        - Momentum is ALWAYS conserved in isolated systems
        - Vector quantity - direction matters
        - Use to validate collision calculations
        - External forces (friction, etc.) can change total momentum

    Example - Collision verification:
        result = await check_momentum_conservation(
            initial_momentum=[3000, 0, 0],  # kg⋅m/s
            final_momentum=[2995, 5, 0],  # slightly off
            tolerance=0.01
        )
    """
    # Parse inputs
    parsed_p_i = (
        json.loads(initial_momentum) if isinstance(initial_momentum, str) else initial_momentum
    )
    parsed_p_f = json.loads(final_momentum) if isinstance(final_momentum, str) else final_momentum

    from ..conservation import (
        MomentumConservationCheckRequest,
        check_momentum_conservation as check_p,
    )

    request = MomentumConservationCheckRequest(
        initial_momentum=parsed_p_i,
        final_momentum=parsed_p_f,
        tolerance=tolerance,
    )
    response = check_p(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def check_angular_momentum_conservation(
    initial_angular_momentum: Union[list[float], str],
    final_angular_momentum: Union[list[float], str],
    tolerance: float = 0.01,
) -> dict:
    """Verify conservation of angular momentum.

    Checks whether total angular momentum is conserved. Angular momentum
    is conserved when no external torques act on the system.

    Args:
        initial_angular_momentum: Initial L [x, y, z] in kg⋅m²/s (or JSON string)
        final_angular_momentum: Final L [x, y, z] in kg⋅m²/s (or JSON string)
        tolerance: Tolerance (fraction, default 0.01 = 1%)

    Returns:
        Dict containing:
            - initial_L_magnitude: Initial |L| in kg⋅m²/s
            - final_L_magnitude: Final |L| in kg⋅m²/s
            - L_difference: Difference [x, y, z]
            - L_difference_magnitude: |ΔL|
            - L_difference_percent: % difference
            - is_conserved: Whether L is conserved within tolerance

    Tips for LLMs:
        - Conserved when no external torques (isolated rotation)
        - Ice skater spinning: pull arms in → I decreases → ω increases (L constant)
        - Gyroscope: resists changes to L direction
        - Planets orbiting: L conserved → elliptical orbits

    Example - Figure skater:
        # Arms extended → Arms pulled in
        result = await check_angular_momentum_conservation(
            initial_angular_momentum=[0, 15, 0],  # kg⋅m²/s
            final_angular_momentum=[0, 15.05, 0],
            tolerance=0.01
        )
    """
    # Parse inputs
    parsed_L_i = (
        json.loads(initial_angular_momentum)
        if isinstance(initial_angular_momentum, str)
        else initial_angular_momentum
    )
    parsed_L_f = (
        json.loads(final_angular_momentum)
        if isinstance(final_angular_momentum, str)
        else final_angular_momentum
    )

    from ..conservation import (
        AngularMomentumConservationCheckRequest,
        check_angular_momentum_conservation as check_L,
    )

    request = AngularMomentumConservationCheckRequest(
        initial_angular_momentum=parsed_L_i,
        final_angular_momentum=parsed_L_f,
        tolerance=tolerance,
    )
    response = check_L(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def track_energy_dissipation(
    trajectory_data: dict,
    mass: float,
    gravity: float = 9.81,
    reference_height: float = 0.0,
) -> dict:
    """Track energy dissipation over a trajectory.

    Analyzes how energy changes over time in a recorded trajectory.
    Useful for understanding damping, bounces, and energy loss mechanisms.

    Args:
        trajectory_data: Trajectory data dict with 'frames' field
        mass: Object mass in kg
        gravity: Gravitational acceleration in m/s² (default 9.81)
        reference_height: Reference height for PE in meters (default 0.0)

    Returns:
        Dict containing:
            - frames: Energy data for each frame (time, KE, PE, total E)
            - initial_total_energy: Initial total energy in Joules
            - final_total_energy: Final total energy in Joules
            - total_energy_loss: Total energy dissipated in Joules
            - total_energy_loss_percent: % of energy lost
            - average_power_dissipated: Average power in Watts (J/s)

    Tips for LLMs:
        - Use after record_trajectory or record_trajectory_with_events
        - Visualize energy vs time to see where energy is lost
        - Identifies bounces, friction effects, air resistance
        - Power = rate of energy dissipation

    Example - Bouncing ball energy analysis:
        traj = await record_trajectory_with_events(sim_id, "ball", 600)
        result = await track_energy_dissipation(
            trajectory_data=traj.model_dump(),
            mass=0.5,  # 500g ball
            gravity=9.81
        )
        # See how energy decreases with each bounce
    """
    from ..conservation import EnergyDissipationTrackingRequest, track_energy_dissipation as track_E
    from ..models import TrajectoryFrame

    # Extract frames from trajectory data
    frames_data = trajectory_data.get("frames", [])
    frames = [TrajectoryFrame.model_validate(f) for f in frames_data]

    request = EnergyDissipationTrackingRequest(
        frames=frames,
        mass=mass,
        gravity=gravity,
        reference_height=reference_height,
    )
    response = track_E(request)
    return response.model_dump()
