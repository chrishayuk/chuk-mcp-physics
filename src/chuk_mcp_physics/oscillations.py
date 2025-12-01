"""Spring and oscillation calculations.

Implements Hooke's law, simple harmonic motion, damped oscillations, and pendulums.
"""

import math
from typing import Literal

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class HookesLawRequest(BaseModel):
    """Request for Hooke's Law calculation."""

    spring_constant: float = Field(..., description="Spring constant k in N/m", gt=0.0)
    displacement: float = Field(..., description="Displacement from equilibrium in meters")


class HookesLawResponse(BaseModel):
    """Response for Hooke's Law calculation."""

    force: float = Field(..., description="Restoring force magnitude in Newtons")
    potential_energy: float = Field(..., description="Elastic potential energy in Joules")


class SpringMassPeriodRequest(BaseModel):
    """Request for spring-mass period calculation."""

    mass: float = Field(..., description="Mass in kg", gt=0.0)
    spring_constant: float = Field(..., description="Spring constant k in N/m", gt=0.0)


class SpringMassPeriodResponse(BaseModel):
    """Response for spring-mass period calculation."""

    period: float = Field(..., description="Period T in seconds")
    frequency: float = Field(..., description="Frequency f in Hz")
    angular_frequency: float = Field(..., description="Angular frequency ω in rad/s")


class SimpleHarmonicMotionRequest(BaseModel):
    """Request for simple harmonic motion calculation."""

    amplitude: float = Field(..., description="Amplitude A in meters", gt=0.0)
    angular_frequency: float = Field(
        ..., description="Angular frequency ω in rad/s (ω = 2πf)", gt=0.0
    )
    phase: float = Field(default=0.0, description="Phase shift φ in radians")
    time: float = Field(..., description="Time t in seconds", ge=0.0)


class SimpleHarmonicMotionResponse(BaseModel):
    """Response for simple harmonic motion calculation."""

    position: float = Field(..., description="Position x(t) in meters")
    velocity: float = Field(..., description="Velocity v(t) in m/s")
    acceleration: float = Field(..., description="Acceleration a(t) in m/s²")


class DampedOscillationRequest(BaseModel):
    """Request for damped oscillation calculation."""

    mass: float = Field(..., description="Mass in kg", gt=0.0)
    spring_constant: float = Field(..., description="Spring constant k in N/m", gt=0.0)
    damping_coefficient: float = Field(..., description="Damping coefficient b in kg/s", ge=0.0)
    initial_position: float = Field(default=1.0, description="Initial position in meters")
    initial_velocity: float = Field(default=0.0, description="Initial velocity in m/s")
    time: float = Field(..., description="Time t in seconds", ge=0.0)


class DampedOscillationResponse(BaseModel):
    """Response for damped oscillation calculation."""

    position: float = Field(..., description="Position x(t) in meters")
    velocity: float = Field(..., description="Velocity v(t) in m/s")
    damping_ratio: float = Field(..., description="Damping ratio ζ (zeta)")
    regime: Literal["underdamped", "critically_damped", "overdamped"] = Field(
        ..., description="Damping regime"
    )


class PendulumPeriodRequest(BaseModel):
    """Request for pendulum period calculation."""

    length: float = Field(..., description="Pendulum length in meters", gt=0.0)
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)
    amplitude_degrees: float | None = Field(
        None, description="Amplitude in degrees (for large angle correction)", ge=0.0, le=180.0
    )


class PendulumPeriodResponse(BaseModel):
    """Response for pendulum period calculation."""

    period: float = Field(..., description="Period T in seconds")
    frequency: float = Field(..., description="Frequency f in Hz")
    angular_frequency: float = Field(..., description="Angular frequency ω in rad/s")
    small_angle_approximation: bool = Field(
        ..., description="Whether small angle approximation was used"
    )


# ============================================================================
# Calculation Functions
# ============================================================================


def calculate_hookes_law(request: HookesLawRequest) -> HookesLawResponse:
    """Calculate spring force using Hooke's Law: F = -kx.

    Args:
        request: Hooke's Law request

    Returns:
        Restoring force and potential energy
    """
    k = request.spring_constant
    x = request.displacement

    force = k * abs(x)  # Magnitude only
    potential_energy = 0.5 * k * x * x

    return HookesLawResponse(force=force, potential_energy=potential_energy)


def calculate_spring_mass_period(request: SpringMassPeriodRequest) -> SpringMassPeriodResponse:
    """Calculate period of spring-mass system: T = 2π√(m/k).

    Args:
        request: Spring-mass period request

    Returns:
        Period, frequency, and angular frequency
    """
    m = request.mass
    k = request.spring_constant

    omega = math.sqrt(k / m)
    T = 2.0 * math.pi / omega
    f = 1.0 / T

    return SpringMassPeriodResponse(period=T, frequency=f, angular_frequency=omega)


def calculate_simple_harmonic_motion(
    request: SimpleHarmonicMotionRequest,
) -> SimpleHarmonicMotionResponse:
    """Calculate simple harmonic motion: x(t) = A cos(ωt + φ).

    Args:
        request: SHM request

    Returns:
        Position, velocity, and acceleration at time t
    """
    A = request.amplitude
    omega = request.angular_frequency
    phi = request.phase
    t = request.time

    # x(t) = A cos(ωt + φ)
    x = A * math.cos(omega * t + phi)

    # v(t) = -Aω sin(ωt + φ)
    v = -A * omega * math.sin(omega * t + phi)

    # a(t) = -Aω² cos(ωt + φ) = -ω²x
    a = -omega * omega * x

    return SimpleHarmonicMotionResponse(position=x, velocity=v, acceleration=a)


def calculate_damped_oscillation(request: DampedOscillationRequest) -> DampedOscillationResponse:
    """Calculate damped oscillation with damping coefficient.

    Damping ratio: ζ = b / (2√(mk))
    - ζ < 1: Underdamped (oscillates)
    - ζ = 1: Critically damped (returns to equilibrium fastest)
    - ζ > 1: Overdamped (slow return, no oscillation)

    Args:
        request: Damped oscillation request

    Returns:
        Position, velocity, damping ratio, and regime
    """
    m = request.mass
    k = request.spring_constant
    b = request.damping_coefficient
    x0 = request.initial_position
    v0 = request.initial_velocity
    t = request.time

    omega0 = math.sqrt(k / m)  # Natural frequency
    zeta = b / (2.0 * math.sqrt(m * k))  # Damping ratio

    # Determine regime
    if zeta < 1.0:
        regime: Literal["underdamped", "critically_damped", "overdamped"] = "underdamped"
        omega_d = omega0 * math.sqrt(1.0 - zeta * zeta)  # Damped frequency
        exp_term = math.exp(-zeta * omega0 * t)
        x = exp_term * (
            x0 * math.cos(omega_d * t)
            + ((v0 + zeta * omega0 * x0) / omega_d) * math.sin(omega_d * t)
        )
        v = -exp_term * (
            (zeta * omega0 * x0 + omega_d * (v0 + zeta * omega0 * x0) / omega_d)
            * math.cos(omega_d * t)
            - omega_d * x0 * math.sin(omega_d * t)
        )
    elif abs(zeta - 1.0) < 1e-6:
        regime = "critically_damped"
        exp_term = math.exp(-omega0 * t)
        x = exp_term * (x0 + (v0 + omega0 * x0) * t)
        v = exp_term * (v0 - omega0 * (x0 + (v0 + omega0 * x0) * t) + (v0 + omega0 * x0))
    else:  # zeta > 1
        regime = "overdamped"
        r1 = -omega0 * (zeta + math.sqrt(zeta * zeta - 1.0))
        r2 = -omega0 * (zeta - math.sqrt(zeta * zeta - 1.0))
        c1 = (v0 - r2 * x0) / (r1 - r2)
        c2 = x0 - c1
        x = c1 * math.exp(r1 * t) + c2 * math.exp(r2 * t)
        v = c1 * r1 * math.exp(r1 * t) + c2 * r2 * math.exp(r2 * t)

    return DampedOscillationResponse(position=x, velocity=v, damping_ratio=zeta, regime=regime)


def calculate_pendulum_period(request: PendulumPeriodRequest) -> PendulumPeriodResponse:
    """Calculate pendulum period: T = 2π√(L/g).

    For large amplitudes, uses first-order correction:
    T ≈ 2π√(L/g) * (1 + θ₀²/16)

    Args:
        request: Pendulum period request

    Returns:
        Period, frequency, and angular frequency
    """
    L = request.length
    g = request.gravity

    omega = math.sqrt(g / L)
    T = 2.0 * math.pi / omega

    # Apply large angle correction if amplitude is provided
    small_angle = True
    if request.amplitude_degrees is not None:
        theta0_rad = math.radians(request.amplitude_degrees)
        # First-order correction for large amplitudes
        if abs(theta0_rad) > 0.1:  # > ~5.7 degrees
            small_angle = False
            T = T * (1.0 + (theta0_rad * theta0_rad) / 16.0)

    f = 1.0 / T
    omega_corrected = 2.0 * math.pi * f

    return PendulumPeriodResponse(
        period=T,
        frequency=f,
        angular_frequency=omega_corrected,
        small_angle_approximation=small_angle,
    )
