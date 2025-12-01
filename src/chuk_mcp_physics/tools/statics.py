"""Statics and equilibrium MCP tools."""

import json

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def check_force_balance(
    forces: list[list[float]] | str,
    tolerance: float = 0.01,
) -> dict:
    """Check if forces are in equilibrium: ΣF = 0.

    Verifies whether a system of forces is balanced (net force = 0).
    Essential for statics problems and structural analysis.

    Args:
        forces: List of force vectors [[x,y,z], ...] in Newtons (or JSON string)
        tolerance: Tolerance for equilibrium check (fraction, default 0.01)

    Returns:
        Dict containing:
            - net_force: Net force vector [x, y, z] in Newtons
            - net_force_magnitude: Net force magnitude in Newtons
            - is_balanced: Whether forces are in equilibrium
            - individual_magnitudes: Magnitude of each force

    Example - Bridge support forces:
        result = await check_force_balance(
            forces=[[0, 1000, 0], [0, 500, 0], [0, -1500, 0]],
            tolerance=0.01
        )
        # is_balanced = True if net force ≈ 0
    """
    from ..statics import ForceBalanceRequest
    from ..statics import check_force_balance as check_forces

    # Parse forces if string
    if isinstance(forces, str):
        forces = json.loads(forces)

    request = ForceBalanceRequest(forces=forces, tolerance=tolerance)
    response = check_forces(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def check_torque_balance(
    torques: list[list[float]] | str,
    tolerance: float = 0.01,
) -> dict:
    """Check if torques are in equilibrium: Στ = 0.

    Verifies whether a system of torques is balanced (net torque = 0).
    Essential for rotational equilibrium and lever problems.

    Args:
        torques: List of torque vectors [[x,y,z], ...] in N⋅m (or JSON string)
        tolerance: Tolerance for equilibrium check (fraction, default 0.01)

    Returns:
        Dict containing:
            - net_torque: Net torque vector [x, y, z] in N⋅m
            - net_torque_magnitude: Net torque magnitude in N⋅m
            - is_balanced: Whether torques are in equilibrium
            - individual_magnitudes: Magnitude of each torque

    Example - Seesaw balance:
        result = await check_torque_balance(
            torques=[[0, 0, 100], [0, 0, -100]],
            tolerance=0.01
        )
    """
    from ..statics import TorqueBalanceRequest
    from ..statics import check_torque_balance as check_torques

    if isinstance(torques, str):
        torques = json.loads(torques)

    request = TorqueBalanceRequest(torques=torques, tolerance=tolerance)
    response = check_torques(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_center_of_mass(
    masses: list[float] | str,
    positions: list[list[float]] | str,
) -> dict:
    """Calculate center of mass for a system of point masses.

    Formula: r_cm = Σ(m_i × r_i) / Σm_i

    Args:
        masses: List of masses in kg (or JSON string)
        positions: List of positions [[x,y,z], ...] in meters (or JSON string)

    Returns:
        Dict containing:
            - center_of_mass: Position [x, y, z] in meters
            - total_mass: Total system mass in kg

    Example - Three-mass system:
        result = await calculate_center_of_mass(
            masses=[1.0, 2.0, 3.0],
            positions=[[0,0,0], [1,0,0], [2,0,0]]
        )
        # center_of_mass ≈ [1.5, 0, 0]
    """
    from ..statics import CenterOfMassRequest
    from ..statics import calculate_center_of_mass as calc_cm

    if isinstance(masses, str):
        masses = json.loads(masses)
    if isinstance(positions, str):
        positions = json.loads(positions)

    request = CenterOfMassRequest(masses=masses, positions=positions)
    response = calc_cm(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_static_friction(
    normal_force: float,
    coefficient_static_friction: float,
    applied_force: float | None = None,
) -> dict:
    """Calculate maximum static friction force: f_s,max = μ_s × N.

    Determines whether an object will slip under applied force.

    Args:
        normal_force: Normal force in Newtons
        coefficient_static_friction: Coefficient of static friction μ_s
        applied_force: Applied horizontal force in Newtons (optional)

    Returns:
        Dict containing:
            - max_static_friction: Maximum static friction in Newtons
            - will_slip: Whether object will slip (if applied_force provided)
            - friction_force: Actual friction force (if applied_force provided)

    Example - Box on floor:
        result = await calculate_static_friction(
            normal_force=100,
            coefficient_static_friction=0.5,
            applied_force=40
        )
        # will_slip = False (40N < 50N max)
    """
    from ..statics import StaticFrictionRequest
    from ..statics import calculate_static_friction as calc_friction

    request = StaticFrictionRequest(
        normal_force=normal_force,
        coefficient_static_friction=coefficient_static_friction,
        applied_force=applied_force,
    )
    response = calc_friction(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_normal_force(
    mass: float,
    gravity: float = 9.81,
    angle_degrees: float = 0.0,
    additional_force: float | None = None,
) -> dict:
    """Calculate normal force on an inclined plane.

    On an incline at angle θ:
    - N = mg cos(θ) + F_additional
    - Weight component perpendicular: mg cos(θ)
    - Weight component parallel: mg sin(θ)

    Args:
        mass: Object mass in kg
        gravity: Gravitational acceleration in m/s² (default 9.81)
        angle_degrees: Incline angle in degrees (0 = horizontal)
        additional_force: Additional perpendicular force in Newtons (optional)

    Returns:
        Dict containing:
            - normal_force: Normal force in Newtons
            - weight_component_perpendicular: Weight component ⊥ to surface
            - weight_component_parallel: Weight component ∥ to surface

    Example - Box on 30° ramp:
        result = await calculate_normal_force(
            mass=10.0,
            angle_degrees=30.0
        )
        # normal_force ≈ 84.9 N
    """
    from ..statics import NormalForceRequest
    from ..statics import calculate_normal_force as calc_normal

    request = NormalForceRequest(
        mass=mass,
        gravity=gravity,
        angle_degrees=angle_degrees,
        additional_force=additional_force,
    )
    response = calc_normal(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def check_equilibrium(
    forces: list[list[float]] | str,
    force_positions: list[list[float]] | str,
    pivot_point: list[float] | str | None = None,
    tolerance: float = 0.01,
) -> dict:
    """Check complete static equilibrium: ΣF = 0 and Στ = 0.

    For static equilibrium, both force and torque must be balanced.

    Args:
        forces: List of force vectors [[x,y,z], ...] in N (or JSON string)
        force_positions: Positions where forces applied [[x,y,z], ...] (or JSON string)
        pivot_point: Pivot point for torque calculation [x,y,z] (default [0,0,0])
        tolerance: Tolerance for equilibrium check (default 0.01)

    Returns:
        Dict containing:
            - force_balanced: Whether ΣF = 0
            - torque_balanced: Whether Στ = 0
            - in_equilibrium: Whether system is in static equilibrium
            - net_force: Net force [x, y, z] in N
            - net_torque: Net torque [x, y, z] in N⋅m

    Example - Beam with two forces:
        result = await check_equilibrium(
            forces=[[0, 100, 0], [0, -100, 0]],
            force_positions=[[1, 0, 0], [2, 0, 0]]
        )
    """
    from ..statics import EquilibriumCheckRequest
    from ..statics import check_equilibrium as check_eq

    if isinstance(forces, str):
        forces = json.loads(forces)
    if isinstance(force_positions, str):
        force_positions = json.loads(force_positions)

    # Handle pivot_point default
    if pivot_point is None:
        pivot_point = [0.0, 0.0, 0.0]
    elif isinstance(pivot_point, str):
        pivot_point = json.loads(pivot_point)

    request = EquilibriumCheckRequest(
        forces=forces,
        force_positions=force_positions,
        pivot_point=pivot_point,
        tolerance=tolerance,
    )
    response = check_eq(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_beam_reactions(
    beam_length: float,
    loads: list[float] | str,
    load_positions: list[float] | str,
) -> dict:
    """Calculate reaction forces for a simply supported beam.

    Uses moment equilibrium about supports to find reaction forces.

    Args:
        beam_length: Beam length in meters
        loads: Point loads in Newtons (downward positive) (or JSON string)
        load_positions: Positions of loads from left end in meters (or JSON string)

    Returns:
        Dict containing:
            - reaction_left: Reaction force at left support in Newtons
            - reaction_right: Reaction force at right support in Newtons
            - total_load: Total downward load in Newtons
            - is_balanced: Whether reactions balance loads

    Example - Beam with two loads:
        result = await calculate_beam_reactions(
            beam_length=10.0,
            loads=[1000, 500],
            load_positions=[3.0, 7.0]
        )
    """
    from ..statics import BeamReactionRequest
    from ..statics import calculate_beam_reactions as calc_beam

    if isinstance(loads, str):
        loads = json.loads(loads)
    if isinstance(load_positions, str):
        load_positions = json.loads(load_positions)

    request = BeamReactionRequest(
        beam_length=beam_length,
        loads=loads,
        load_positions=load_positions,
    )
    response = calc_beam(request)
    return response.model_dump()
