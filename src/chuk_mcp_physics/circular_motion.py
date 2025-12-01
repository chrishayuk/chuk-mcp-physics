"""Circular motion and orbital mechanics calculations.

Implements centripetal force, orbital periods, banking angles, and escape velocity.
"""

import math

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class CentripetalForceRequest(BaseModel):
    """Request for centripetal force calculation."""

    mass: float = Field(..., description="Mass in kg", gt=0.0)
    velocity: float = Field(..., description="Velocity magnitude in m/s", ge=0.0)
    radius: float = Field(..., description="Radius of circular path in meters", gt=0.0)


class CentripetalForceResponse(BaseModel):
    """Response for centripetal force calculation."""

    centripetal_force: float = Field(..., description="Centripetal force in Newtons")
    centripetal_acceleration: float = Field(..., description="Centripetal acceleration in m/s²")


class OrbitalPeriodRequest(BaseModel):
    """Request for orbital period calculation."""

    orbital_radius: float = Field(..., description="Orbital radius in meters", gt=0.0)
    central_mass: float = Field(..., description="Mass of central body in kg", gt=0.0)
    gravitational_constant: float = Field(
        default=6.674e-11, description="Gravitational constant G in m³/(kg⋅s²)"
    )


class OrbitalPeriodResponse(BaseModel):
    """Response for orbital period calculation."""

    period: float = Field(..., description="Orbital period in seconds")
    orbital_velocity: float = Field(..., description="Orbital velocity in m/s")
    period_hours: float = Field(..., description="Orbital period in hours (for convenience)")
    period_days: float = Field(..., description="Orbital period in days (for convenience)")


class BankingAngleRequest(BaseModel):
    """Request for banking angle calculation."""

    velocity: float = Field(..., description="Velocity in m/s", gt=0.0)
    radius: float = Field(..., description="Radius of turn in meters", gt=0.0)
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)


class BankingAngleResponse(BaseModel):
    """Response for banking angle calculation."""

    angle_radians: float = Field(..., description="Banking angle in radians")
    angle_degrees: float = Field(..., description="Banking angle in degrees")


class EscapeVelocityRequest(BaseModel):
    """Request for escape velocity calculation."""

    mass: float = Field(..., description="Mass of celestial body in kg", gt=0.0)
    radius: float = Field(..., description="Radius of celestial body in meters", gt=0.0)
    gravitational_constant: float = Field(
        default=6.674e-11, description="Gravitational constant G in m³/(kg⋅s²)"
    )


class EscapeVelocityResponse(BaseModel):
    """Response for escape velocity calculation."""

    escape_velocity: float = Field(..., description="Escape velocity in m/s")
    escape_velocity_kmh: float = Field(..., description="Escape velocity in km/h (for convenience)")


class CircularOrbitRequest(BaseModel):
    """Request for circular orbit analysis."""

    altitude: float = Field(..., description="Altitude above surface in meters", ge=0.0)
    planet_mass: float = Field(..., description="Planet mass in kg", gt=0.0)
    planet_radius: float = Field(..., description="Planet radius in meters", gt=0.0)
    gravitational_constant: float = Field(
        default=6.674e-11, description="Gravitational constant G in m³/(kg⋅s²)"
    )


class CircularOrbitResponse(BaseModel):
    """Response for circular orbit analysis."""

    orbital_radius: float = Field(..., description="Orbital radius (from planet center) in meters")
    orbital_velocity: float = Field(..., description="Orbital velocity in m/s")
    period_seconds: float = Field(..., description="Orbital period in seconds")
    period_minutes: float = Field(..., description="Orbital period in minutes")
    centripetal_acceleration: float = Field(
        ..., description="Centripetal acceleration at this orbit in m/s²"
    )


# ============================================================================
# Calculation Functions
# ============================================================================


def calculate_centripetal_force(request: CentripetalForceRequest) -> CentripetalForceResponse:
    """Calculate centripetal force: F_c = m * v² / r.

    Args:
        request: Centripetal force request

    Returns:
        Centripetal force and acceleration
    """
    m = request.mass
    v = request.velocity
    r = request.radius

    a_c = (v * v) / r
    F_c = m * a_c

    return CentripetalForceResponse(centripetal_force=F_c, centripetal_acceleration=a_c)


def calculate_orbital_period(request: OrbitalPeriodRequest) -> OrbitalPeriodResponse:
    """Calculate orbital period using Kepler's Third Law: T = 2π√(r³/GM).

    Args:
        request: Orbital period request

    Returns:
        Orbital period and velocity
    """
    r = request.orbital_radius
    M = request.central_mass
    G = request.gravitational_constant

    # T = 2π√(r³/GM)
    T = 2.0 * math.pi * math.sqrt((r * r * r) / (G * M))

    # Orbital velocity: v = 2πr/T
    v = (2.0 * math.pi * r) / T

    T_hours = T / 3600.0
    T_days = T / 86400.0

    return OrbitalPeriodResponse(
        period=T, orbital_velocity=v, period_hours=T_hours, period_days=T_days
    )


def calculate_banking_angle(request: BankingAngleRequest) -> BankingAngleResponse:
    """Calculate ideal banking angle for a turn: θ = arctan(v² / (rg)).

    Args:
        request: Banking angle request

    Returns:
        Banking angle in radians and degrees
    """
    v = request.velocity
    r = request.radius
    g = request.gravity

    # θ = arctan(v² / (rg))
    theta_rad = math.atan((v * v) / (r * g))
    theta_deg = math.degrees(theta_rad)

    return BankingAngleResponse(angle_radians=theta_rad, angle_degrees=theta_deg)


def calculate_escape_velocity(request: EscapeVelocityRequest) -> EscapeVelocityResponse:
    """Calculate escape velocity: v_escape = √(2GM/r).

    Args:
        request: Escape velocity request

    Returns:
        Escape velocity in m/s and km/h
    """
    M = request.mass
    r = request.radius
    G = request.gravitational_constant

    # v_escape = √(2GM/r)
    v_escape = math.sqrt((2.0 * G * M) / r)
    v_escape_kmh = v_escape * 3.6  # Convert m/s to km/h

    return EscapeVelocityResponse(escape_velocity=v_escape, escape_velocity_kmh=v_escape_kmh)


def analyze_circular_orbit(request: CircularOrbitRequest) -> CircularOrbitResponse:
    """Analyze a circular orbit at given altitude.

    Combines multiple orbital calculations into a comprehensive analysis.

    Args:
        request: Circular orbit request

    Returns:
        Complete orbital analysis
    """
    h = request.altitude
    R = request.planet_radius
    M = request.planet_mass
    G = request.gravitational_constant

    # Orbital radius (from planet center)
    r = R + h

    # Orbital velocity: v = √(GM/r)
    v = math.sqrt((G * M) / r)

    # Period: T = 2π√(r³/GM)
    T = 2.0 * math.pi * math.sqrt((r * r * r) / (G * M))
    T_min = T / 60.0

    # Centripetal acceleration
    a_c = (v * v) / r

    return CircularOrbitResponse(
        orbital_radius=r,
        orbital_velocity=v,
        period_seconds=T,
        period_minutes=T_min,
        centripetal_acceleration=a_c,
    )
