"""Statics and equilibrium calculations.

Implements force balance, torque balance, center of mass, and static friction.
"""

import math
from typing import Optional

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class ForceBalanceRequest(BaseModel):
    """Request for force balance verification."""

    forces: list[list[float]] = Field(..., description="List of force vectors [x, y, z] in Newtons")
    tolerance: float = Field(
        default=0.01, description="Tolerance for equilibrium check (fraction)", ge=0.0
    )


class ForceBalanceResponse(BaseModel):
    """Response for force balance verification."""

    net_force: list[float] = Field(..., description="Net force vector [x, y, z] in Newtons")
    net_force_magnitude: float = Field(..., description="Net force magnitude in Newtons")
    is_balanced: bool = Field(..., description="Whether forces are in equilibrium")
    individual_magnitudes: list[float] = Field(
        ..., description="Magnitude of each input force in Newtons"
    )


class TorqueBalanceRequest(BaseModel):
    """Request for torque balance verification."""

    torques: list[list[float]] = Field(..., description="List of torque vectors [x, y, z] in N⋅m")
    tolerance: float = Field(
        default=0.01, description="Tolerance for equilibrium check (fraction)", ge=0.0
    )


class TorqueBalanceResponse(BaseModel):
    """Response for torque balance verification."""

    net_torque: list[float] = Field(..., description="Net torque vector [x, y, z] in N⋅m")
    net_torque_magnitude: float = Field(..., description="Net torque magnitude in N⋅m")
    is_balanced: bool = Field(..., description="Whether torques are in equilibrium")
    individual_magnitudes: list[float] = Field(
        ..., description="Magnitude of each input torque in N⋅m"
    )


class CenterOfMassRequest(BaseModel):
    """Request for center of mass calculation."""

    masses: list[float] = Field(..., description="List of masses in kg")
    positions: list[list[float]] = Field(..., description="List of positions [x, y, z] in meters")


class CenterOfMassResponse(BaseModel):
    """Response for center of mass calculation."""

    center_of_mass: list[float] = Field(
        ..., description="Center of mass position [x, y, z] in meters"
    )
    total_mass: float = Field(..., description="Total system mass in kg")


class StaticFrictionRequest(BaseModel):
    """Request for static friction calculation."""

    normal_force: float = Field(..., description="Normal force in Newtons", gt=0.0)
    coefficient_static_friction: float = Field(
        ..., description="Coefficient of static friction μ_s", ge=0.0
    )
    applied_force: Optional[float] = Field(
        None, description="Applied horizontal force in Newtons (optional)"
    )


class StaticFrictionResponse(BaseModel):
    """Response for static friction calculation."""

    max_static_friction: float = Field(..., description="Maximum static friction force in Newtons")
    will_slip: Optional[bool] = Field(
        None, description="Whether object will slip (if applied_force provided)"
    )
    friction_force: Optional[float] = Field(
        None, description="Actual friction force in Newtons (if applied_force provided)"
    )


class NormalForceRequest(BaseModel):
    """Request for normal force calculation."""

    mass: float = Field(..., description="Object mass in kg", gt=0.0)
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)
    angle_degrees: float = Field(
        default=0.0, description="Incline angle in degrees (0 = horizontal)", ge=0.0, le=90.0
    )
    additional_force: Optional[float] = Field(
        None, description="Additional perpendicular force in Newtons (optional)"
    )


class NormalForceResponse(BaseModel):
    """Response for normal force calculation."""

    normal_force: float = Field(..., description="Normal force in Newtons")
    weight_component_perpendicular: float = Field(
        ..., description="Weight component perpendicular to surface in Newtons"
    )
    weight_component_parallel: float = Field(
        ..., description="Weight component parallel to surface in Newtons"
    )


class EquilibriumCheckRequest(BaseModel):
    """Request for complete equilibrium verification."""

    forces: list[list[float]] = Field(..., description="List of force vectors [x, y, z] in N")
    force_positions: list[list[float]] = Field(
        ..., description="Positions where forces are applied [x, y, z] in meters"
    )
    pivot_point: list[float] = Field(
        default=[0.0, 0.0, 0.0], description="Pivot point for torque calculation [x, y, z]"
    )
    tolerance: float = Field(default=0.01, description="Tolerance for equilibrium check", ge=0.0)


class EquilibriumCheckResponse(BaseModel):
    """Response for equilibrium verification."""

    force_balanced: bool = Field(..., description="Whether ΣF = 0")
    torque_balanced: bool = Field(..., description="Whether Στ = 0")
    in_equilibrium: bool = Field(..., description="Whether system is in static equilibrium")
    net_force: list[float] = Field(..., description="Net force [x, y, z] in N")
    net_torque: list[float] = Field(..., description="Net torque [x, y, z] in N⋅m")


class BeamReactionRequest(BaseModel):
    """Request for beam reaction force calculation (simple supported beam)."""

    beam_length: float = Field(..., description="Beam length in meters", gt=0.0)
    loads: list[float] = Field(..., description="Point loads in Newtons (downward positive)")
    load_positions: list[float] = Field(
        ..., description="Positions of loads from left end in meters"
    )


class BeamReactionResponse(BaseModel):
    """Response for beam reaction forces."""

    reaction_left: float = Field(..., description="Reaction force at left support in Newtons")
    reaction_right: float = Field(..., description="Reaction force at right support in Newtons")
    total_load: float = Field(..., description="Total downward load in Newtons")
    is_balanced: bool = Field(..., description="Whether reactions balance loads")


# ============================================================================
# Helper Functions
# ============================================================================


def _vector_magnitude(v: list[float]) -> float:
    """Calculate magnitude of a vector."""
    return math.sqrt(sum(x * x for x in v))


def _vector_add(vectors: list[list[float]]) -> list[float]:
    """Sum multiple vectors."""
    if not vectors:
        return [0.0, 0.0, 0.0]

    result = [0.0] * len(vectors[0])
    for vec in vectors:
        for i in range(len(vec)):
            result[i] += vec[i]
    return result


def _cross_product(a: list[float], b: list[float]) -> list[float]:
    """Calculate cross product a × b."""
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _vector_subtract(a: list[float], b: list[float]) -> list[float]:
    """Subtract vector b from vector a."""
    return [x - y for x, y in zip(a, b)]


# ============================================================================
# Calculation Functions
# ============================================================================


def check_force_balance(request: ForceBalanceRequest) -> ForceBalanceResponse:
    """Check if forces are in equilibrium: ΣF = 0.

    Args:
        request: Force balance request

    Returns:
        Force balance analysis
    """
    net_force = _vector_add(request.forces)
    net_magnitude = _vector_magnitude(net_force)

    individual_magnitudes = [_vector_magnitude(f) for f in request.forces]
    total_magnitude = sum(individual_magnitudes)

    is_balanced = (
        net_magnitude <= request.tolerance * total_magnitude if total_magnitude > 0 else True
    )

    return ForceBalanceResponse(
        net_force=net_force,
        net_force_magnitude=net_magnitude,
        is_balanced=is_balanced,
        individual_magnitudes=individual_magnitudes,
    )


def check_torque_balance(request: TorqueBalanceRequest) -> TorqueBalanceResponse:
    """Check if torques are in equilibrium: Στ = 0.

    Args:
        request: Torque balance request

    Returns:
        Torque balance analysis
    """
    net_torque = _vector_add(request.torques)
    net_magnitude = _vector_magnitude(net_torque)

    individual_magnitudes = [_vector_magnitude(t) for t in request.torques]
    total_magnitude = sum(individual_magnitudes)

    is_balanced = (
        net_magnitude <= request.tolerance * total_magnitude if total_magnitude > 0 else True
    )

    return TorqueBalanceResponse(
        net_torque=net_torque,
        net_torque_magnitude=net_magnitude,
        is_balanced=is_balanced,
        individual_magnitudes=individual_magnitudes,
    )


def calculate_center_of_mass(request: CenterOfMassRequest) -> CenterOfMassResponse:
    """Calculate center of mass for a system of point masses.

    Formula: r_cm = Σ(m_i * r_i) / Σm_i

    Args:
        request: Center of mass request

    Returns:
        Center of mass position and total mass
    """
    if len(request.masses) != len(request.positions):
        raise ValueError("Number of masses must equal number of positions")

    total_mass = sum(request.masses)

    if total_mass == 0:
        raise ValueError("Total mass cannot be zero")

    # Calculate weighted sum of positions
    weighted_sum = [0.0, 0.0, 0.0]
    for mass, pos in zip(request.masses, request.positions):
        for i in range(3):
            weighted_sum[i] += mass * pos[i]

    # Divide by total mass
    center_of_mass = [x / total_mass for x in weighted_sum]

    return CenterOfMassResponse(
        center_of_mass=center_of_mass,
        total_mass=total_mass,
    )


def calculate_static_friction(request: StaticFrictionRequest) -> StaticFrictionResponse:
    """Calculate maximum static friction force: f_s,max = μ_s × N.

    Args:
        request: Static friction request

    Returns:
        Maximum static friction and slip prediction
    """
    max_static_friction = request.coefficient_static_friction * request.normal_force

    will_slip = None
    friction_force = None

    if request.applied_force is not None:
        will_slip = request.applied_force > max_static_friction
        # Actual friction force equals applied force up to the maximum
        friction_force = min(request.applied_force, max_static_friction)

    return StaticFrictionResponse(
        max_static_friction=max_static_friction,
        will_slip=will_slip,
        friction_force=friction_force,
    )


def calculate_normal_force(request: NormalForceRequest) -> NormalForceResponse:
    """Calculate normal force on an inclined plane.

    On an incline at angle θ:
    - N = mg cos(θ) + F_additional
    - Weight component perpendicular: mg cos(θ)
    - Weight component parallel: mg sin(θ)

    Args:
        request: Normal force request

    Returns:
        Normal force and weight components
    """
    weight = request.mass * request.gravity
    angle_rad = math.radians(request.angle_degrees)

    weight_perpendicular = weight * math.cos(angle_rad)
    weight_parallel = weight * math.sin(angle_rad)

    normal_force = weight_perpendicular
    if request.additional_force is not None:
        normal_force += request.additional_force

    return NormalForceResponse(
        normal_force=normal_force,
        weight_component_perpendicular=weight_perpendicular,
        weight_component_parallel=weight_parallel,
    )


def check_equilibrium(request: EquilibriumCheckRequest) -> EquilibriumCheckResponse:
    """Check complete static equilibrium: ΣF = 0 and Στ = 0.

    For static equilibrium, both force and torque must be balanced.

    Args:
        request: Equilibrium check request

    Returns:
        Complete equilibrium analysis
    """
    if len(request.forces) != len(request.force_positions):
        raise ValueError("Number of forces must equal number of positions")

    # Check force balance
    net_force = _vector_add(request.forces)
    net_force_magnitude = _vector_magnitude(net_force)

    total_force_magnitude = sum(_vector_magnitude(f) for f in request.forces)
    force_balanced = (
        net_force_magnitude <= request.tolerance * total_force_magnitude
        if total_force_magnitude > 0
        else True
    )

    # Calculate torques about pivot point
    torques = []
    for force, position in zip(request.forces, request.force_positions):
        r = _vector_subtract(position, request.pivot_point)
        torque = _cross_product(r, force)
        torques.append(torque)

    net_torque = _vector_add(torques)
    net_torque_magnitude = _vector_magnitude(net_torque)

    total_torque_magnitude = sum(_vector_magnitude(t) for t in torques)
    torque_balanced = (
        net_torque_magnitude <= request.tolerance * total_torque_magnitude
        if total_torque_magnitude > 0
        else True
    )

    in_equilibrium = force_balanced and torque_balanced

    return EquilibriumCheckResponse(
        force_balanced=force_balanced,
        torque_balanced=torque_balanced,
        in_equilibrium=in_equilibrium,
        net_force=net_force,
        net_torque=net_torque,
    )


def calculate_beam_reactions(request: BeamReactionRequest) -> BeamReactionResponse:
    """Calculate reaction forces for a simply supported beam with point loads.

    Uses moment equilibrium about left support:
    Σ M_left = 0: R_right × L - Σ(P_i × x_i) = 0

    Then force equilibrium:
    Σ F_y = 0: R_left + R_right - Σ P_i = 0

    Args:
        request: Beam reaction request

    Returns:
        Reaction forces at supports
    """
    if len(request.loads) != len(request.load_positions):
        raise ValueError("Number of loads must equal number of positions")

    # Check that all positions are within beam length
    for pos in request.load_positions:
        if pos < 0 or pos > request.beam_length:
            raise ValueError(f"Load position {pos}m is outside beam length {request.beam_length}m")

    total_load = sum(request.loads)

    # Moment about left support
    moment_about_left = sum(load * pos for load, pos in zip(request.loads, request.load_positions))

    # R_right from moment equilibrium
    reaction_right = moment_about_left / request.beam_length

    # R_left from force equilibrium
    reaction_left = total_load - reaction_right

    # Check if balanced
    is_balanced = abs((reaction_left + reaction_right) - total_load) < 0.01

    return BeamReactionResponse(
        reaction_left=reaction_left,
        reaction_right=reaction_right,
        total_load=total_load,
        is_balanced=is_balanced,
    )
