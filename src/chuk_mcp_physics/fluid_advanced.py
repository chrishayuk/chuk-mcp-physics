"""Advanced fluid dynamics calculations.

Implements lift force, Magnus effect, and Bernoulli's principle.
"""

import math
from typing import Optional

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class LiftForceRequest(BaseModel):
    """Request for lift force calculation (Bernoulli's principle)."""

    velocity: float = Field(..., description="Flow velocity in m/s", gt=0.0)
    wing_area: float = Field(..., description="Wing area in m²", gt=0.0)
    lift_coefficient: float = Field(..., description="Lift coefficient C_L (dimensionless)", ge=0.0)
    fluid_density: float = Field(
        default=1.225, description="Fluid density in kg/m³ (air=1.225)", gt=0.0
    )


class LiftForceResponse(BaseModel):
    """Response for lift force calculation."""

    lift_force: float = Field(..., description="Lift force in Newtons")
    dynamic_pressure: float = Field(..., description="Dynamic pressure (q) in Pascals")


class MagnusForceRequest(BaseModel):
    """Request for Magnus force calculation (spinning ball in fluid)."""

    velocity: list[float] = Field(..., description="Ball velocity [x, y, z] in m/s")
    angular_velocity: list[float] = Field(
        ..., description="Angular velocity [x, y, z] in rad/s (spin)"
    )
    radius: float = Field(..., description="Ball radius in meters", gt=0.0)
    fluid_density: float = Field(
        default=1.225, description="Fluid density in kg/m³ (air=1.225)", gt=0.0
    )


class MagnusForceResponse(BaseModel):
    """Response for Magnus force calculation."""

    magnus_force: list[float] = Field(..., description="Magnus force vector [x, y, z] in Newtons")
    magnus_force_magnitude: float = Field(..., description="Magnus force magnitude in Newtons")
    spin_rate: float = Field(..., description="Spin rate (angular velocity magnitude) in rad/s")


class BernoulliRequest(BaseModel):
    """Request for Bernoulli's equation calculation."""

    pressure1: float = Field(..., description="Pressure at point 1 in Pascals")
    velocity1: float = Field(..., description="Velocity at point 1 in m/s", ge=0.0)
    height1: float = Field(..., description="Height at point 1 in meters")
    velocity2: Optional[float] = Field(None, description="Velocity at point 2 in m/s (if known)")
    height2: Optional[float] = Field(None, description="Height at point 2 in meters (if known)")
    fluid_density: float = Field(
        default=1000.0, description="Fluid density in kg/m³ (water=1000)", gt=0.0
    )
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)


class BernoulliResponse(BaseModel):
    """Response for Bernoulli's equation."""

    pressure2: Optional[float] = Field(None, description="Pressure at point 2 in Pascals")
    total_pressure_1: float = Field(..., description="Total pressure at point 1 in Pascals")
    dynamic_pressure_1: float = Field(..., description="Dynamic pressure at point 1 in Pascals")
    static_pressure_1: float = Field(..., description="Static pressure at point 1 in Pascals")


class PressureAtDepthRequest(BaseModel):
    """Request for pressure at depth calculation."""

    depth: float = Field(..., description="Depth below surface in meters", ge=0.0)
    fluid_density: float = Field(
        default=1000.0, description="Fluid density in kg/m³ (water=1000)", gt=0.0
    )
    atmospheric_pressure: float = Field(
        default=101325.0, description="Atmospheric pressure in Pascals"
    )
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)


class PressureAtDepthResponse(BaseModel):
    """Response for pressure at depth."""

    total_pressure: float = Field(..., description="Total pressure at depth in Pascals")
    gauge_pressure: float = Field(..., description="Gauge pressure (above atmospheric) in Pascals")
    pressure_atmospheres: float = Field(..., description="Pressure in atmospheres (atm)")


class ReynoldsNumberRequest(BaseModel):
    """Request for Reynolds number calculation."""

    velocity: float = Field(..., description="Flow velocity in m/s", gt=0.0)
    characteristic_length: float = Field(
        ..., description="Characteristic length (diameter, chord) in meters", gt=0.0
    )
    fluid_density: float = Field(..., description="Fluid density in kg/m³", gt=0.0)
    dynamic_viscosity: float = Field(..., description="Dynamic viscosity in Pa⋅s", gt=0.0)


class ReynoldsNumberResponse(BaseModel):
    """Response for Reynolds number."""

    reynolds_number: float = Field(..., description="Reynolds number (dimensionless)")
    flow_regime: str = Field(..., description="Flow regime: laminar, transitional, or turbulent")


class VenturiEffectRequest(BaseModel):
    """Request for Venturi effect calculation (flow through constriction)."""

    inlet_diameter: float = Field(..., description="Inlet diameter in meters", gt=0.0)
    throat_diameter: float = Field(..., description="Throat diameter in meters", gt=0.0)
    inlet_velocity: float = Field(..., description="Inlet velocity in m/s", ge=0.0)
    fluid_density: float = Field(default=1000.0, description="Fluid density in kg/m³", gt=0.0)


class VenturiEffectResponse(BaseModel):
    """Response for Venturi effect."""

    throat_velocity: float = Field(..., description="Velocity at throat in m/s")
    pressure_drop: float = Field(..., description="Pressure drop from inlet to throat in Pascals")
    flow_rate: float = Field(..., description="Volumetric flow rate in m³/s")


# ============================================================================
# Helper Functions
# ============================================================================


def _vector_magnitude(v: list[float]) -> float:
    """Calculate magnitude of a vector."""
    return math.sqrt(sum(x * x for x in v))


def _cross_product(a: list[float], b: list[float]) -> list[float]:
    """Calculate cross product a × b."""
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


# ============================================================================
# Calculation Functions
# ============================================================================


def calculate_lift_force(request: LiftForceRequest) -> LiftForceResponse:
    """Calculate lift force using: L = (1/2) ρ v² C_L A.

    This is based on Bernoulli's principle and wing aerodynamics.

    Args:
        request: Lift force request

    Returns:
        Lift force and dynamic pressure
    """
    rho = request.fluid_density
    v = request.velocity
    C_L = request.lift_coefficient
    A = request.wing_area

    # Dynamic pressure
    q = 0.5 * rho * v * v

    # Lift force
    L = q * C_L * A

    return LiftForceResponse(
        lift_force=L,
        dynamic_pressure=q,
    )


def calculate_magnus_force(request: MagnusForceRequest) -> MagnusForceResponse:
    """Calculate Magnus force on a spinning ball.

    The Magnus force is perpendicular to both the velocity and spin axis.
    F_magnus ∝ ω × v

    Simplified model: F ≈ (1/2) ρ π r³ |ω × v|

    Args:
        request: Magnus force request

    Returns:
        Magnus force vector
    """
    rho = request.fluid_density
    r = request.radius
    v = request.velocity
    omega = request.angular_velocity

    # Magnus force is proportional to ω × v
    cross = _cross_product(omega, v)

    # Magnitude of spin
    spin_rate = _vector_magnitude(omega)

    # Simplified Magnus force coefficient
    # F ≈ (1/2) * ρ * π * r³ * |ω × v|
    coeff = 0.5 * rho * math.pi * (r**3)

    magnus = [coeff * c for c in cross]
    magnitude = _vector_magnitude(magnus)

    return MagnusForceResponse(
        magnus_force=magnus,
        magnus_force_magnitude=magnitude,
        spin_rate=spin_rate,
    )


def calculate_bernoulli(request: BernoulliRequest) -> BernoulliResponse:
    """Apply Bernoulli's equation: P + (1/2)ρv² + ρgh = constant.

    Args:
        request: Bernoulli request

    Returns:
        Pressure and energy calculations
    """
    rho = request.fluid_density
    g = request.gravity

    # Calculate total pressure at point 1
    dynamic_1 = 0.5 * rho * request.velocity1 * request.velocity1
    potential_1 = rho * g * request.height1
    static_1 = request.pressure1
    total_1 = static_1 + dynamic_1 + potential_1

    pressure2 = None
    if request.velocity2 is not None and request.height2 is not None:
        # Calculate pressure at point 2
        dynamic_2 = 0.5 * rho * request.velocity2 * request.velocity2
        potential_2 = rho * g * request.height2
        pressure2 = total_1 - dynamic_2 - potential_2

    return BernoulliResponse(
        pressure2=pressure2,
        total_pressure_1=total_1,
        dynamic_pressure_1=dynamic_1,
        static_pressure_1=static_1,
    )


def calculate_pressure_at_depth(request: PressureAtDepthRequest) -> PressureAtDepthResponse:
    """Calculate pressure at depth: P = P_atm + ρgh.

    Args:
        request: Pressure at depth request

    Returns:
        Total and gauge pressure
    """
    rho = request.fluid_density
    g = request.gravity
    h = request.depth
    P_atm = request.atmospheric_pressure

    # Gauge pressure (pressure due to fluid column)
    P_gauge = rho * g * h

    # Total pressure
    P_total = P_atm + P_gauge

    # Convert to atmospheres
    P_atmospheres = P_total / 101325.0

    return PressureAtDepthResponse(
        total_pressure=P_total,
        gauge_pressure=P_gauge,
        pressure_atmospheres=P_atmospheres,
    )


def calculate_reynolds_number(request: ReynoldsNumberRequest) -> ReynoldsNumberResponse:
    """Calculate Reynolds number: Re = ρvL/μ.

    The Reynolds number characterizes the flow regime:
    - Re < 2300: Laminar flow
    - 2300 < Re < 4000: Transitional flow
    - Re > 4000: Turbulent flow

    Args:
        request: Reynolds number request

    Returns:
        Reynolds number and flow regime
    """
    rho = request.fluid_density
    v = request.velocity
    L = request.characteristic_length
    mu = request.dynamic_viscosity

    Re = (rho * v * L) / mu

    # Determine flow regime
    if Re < 2300:
        regime = "laminar"
    elif Re < 4000:
        regime = "transitional"
    else:
        regime = "turbulent"

    return ReynoldsNumberResponse(
        reynolds_number=Re,
        flow_regime=regime,
    )


def calculate_venturi_effect(request: VenturiEffectRequest) -> VenturiEffectResponse:
    """Calculate Venturi effect (flow through constriction).

    Uses continuity equation and Bernoulli's principle:
    - A₁v₁ = A₂v₂ (continuity)
    - P₁ + (1/2)ρv₁² = P₂ + (1/2)ρv₂² (Bernoulli)

    Args:
        request: Venturi effect request

    Returns:
        Throat velocity and pressure drop
    """
    d1 = request.inlet_diameter
    d2 = request.throat_diameter
    v1 = request.inlet_velocity
    rho = request.fluid_density

    # Areas
    A1 = math.pi * (d1 / 2.0) ** 2
    A2 = math.pi * (d2 / 2.0) ** 2

    # Continuity: A₁v₁ = A₂v₂
    v2 = v1 * (A1 / A2)

    # Bernoulli: ΔP = (1/2)ρ(v₂² - v₁²)
    pressure_drop = 0.5 * rho * (v2 * v2 - v1 * v1)

    # Flow rate
    flow_rate = A1 * v1

    return VenturiEffectResponse(
        throat_velocity=v2,
        pressure_drop=pressure_drop,
        flow_rate=flow_rate,
    )
