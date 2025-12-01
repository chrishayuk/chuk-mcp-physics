"""Springs and oscillations MCP tool endpoints."""

from typing import Optional

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def calculate_hookes_law(
    spring_constant: float,
    displacement: float,
) -> dict:
    """Calculate spring force using Hooke's Law: F = -kx.

    The restoring force is proportional to displacement from equilibrium.
    Fundamental for springs, elastic materials, and simple harmonic motion.

    Args:
        spring_constant: Spring constant k in N/m (stiffness)
        displacement: Displacement from equilibrium in meters

    Returns:
        Dict containing:
            - force: Restoring force magnitude in Newtons
            - potential_energy: Elastic potential energy in Joules

    Tips for LLMs:
        - Stiffer spring → larger k → more force for same displacement
        - Potential energy stored in spring: PE = (1/2)kx²
        - Negative sign in F = -kx means force opposes displacement

    Example - Compressing a car spring:
        result = await calculate_hookes_law(
            spring_constant=10000,  # N/m (stiff car spring)
            displacement=0.05  # 5cm compression
        )
        # Force = 500 N, PE = 12.5 J
    """
    from ..oscillations import HookesLawRequest, calculate_hookes_law as calc_hookes

    request = HookesLawRequest(
        spring_constant=spring_constant,
        displacement=displacement,
    )
    response = calc_hookes(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_spring_mass_period(
    mass: float,
    spring_constant: float,
) -> dict:
    """Calculate period of spring-mass system: T = 2π√(m/k).

    Natural oscillation frequency of a mass attached to a spring.
    Independent of amplitude (for ideal springs).

    Args:
        mass: Mass in kg
        spring_constant: Spring constant k in N/m

    Returns:
        Dict containing:
            - period: T in seconds
            - frequency: f in Hz
            - angular_frequency: ω in rad/s

    Tips for LLMs:
        - Heavier mass → longer period (slower oscillation)
        - Stiffer spring → shorter period (faster oscillation)
        - ω = 2πf = √(k/m)

    Example - Mass on spring:
        result = await calculate_spring_mass_period(
            mass=0.5,  # 500g mass
            spring_constant=20.0  # N/m
        )
        # T ≈ 0.99s, f ≈ 1.01 Hz
    """
    from ..oscillations import SpringMassPeriodRequest, calculate_spring_mass_period as calc_period

    request = SpringMassPeriodRequest(
        mass=mass,
        spring_constant=spring_constant,
    )
    response = calc_period(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_simple_harmonic_motion(
    amplitude: float,
    angular_frequency: float,
    time: float,
    phase: float = 0.0,
) -> dict:
    """Calculate simple harmonic motion: x(t) = A cos(ωt + φ).

    Position, velocity, and acceleration for sinusoidal oscillation.
    Models ideal springs, pendulums, and many other oscillating systems.

    Args:
        amplitude: Amplitude A in meters (maximum displacement)
        angular_frequency: ω in rad/s (ω = 2πf)
        time: Time t in seconds
        phase: Phase shift φ in radians (default 0)

    Returns:
        Dict containing:
            - position: x(t) in meters
            - velocity: v(t) = -Aω sin(ωt + φ) in m/s
            - acceleration: a(t) = -Aω² cos(ωt + φ) in m/s²

    Tips for LLMs:
        - Position and acceleration are 180° out of phase
        - Maximum velocity occurs at equilibrium (x = 0)
        - Maximum acceleration occurs at maximum displacement

    Example - Oscillating mass:
        result = await calculate_simple_harmonic_motion(
            amplitude=0.1,  # 10cm amplitude
            angular_frequency=5.0,  # rad/s
            time=1.0  # at t = 1s
        )
    """
    from ..oscillations import (
        SimpleHarmonicMotionRequest,
        calculate_simple_harmonic_motion as calc_shm,
    )

    request = SimpleHarmonicMotionRequest(
        amplitude=amplitude,
        angular_frequency=angular_frequency,
        phase=phase,
        time=time,
    )
    response = calc_shm(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_damped_oscillation(
    mass: float,
    spring_constant: float,
    damping_coefficient: float,
    time: float,
    initial_position: float = 1.0,
    initial_velocity: float = 0.0,
) -> dict:
    """Calculate damped oscillation with friction/resistance.

    Real oscillators lose energy over time due to damping (air resistance,
    friction). Three regimes: underdamped, critically damped, overdamped.

    Args:
        mass: Mass in kg
        spring_constant: k in N/m
        damping_coefficient: b in kg/s (damping strength)
        time: Time t in seconds
        initial_position: Initial position in meters (default 1.0)
        initial_velocity: Initial velocity in m/s (default 0.0)

    Returns:
        Dict containing:
            - position: x(t) in meters
            - velocity: v(t) in m/s
            - damping_ratio: ζ (zeta) = b/(2√(mk))
            - regime: "underdamped", "critically_damped", or "overdamped"

    Damping regimes:
        - ζ < 1: Underdamped (oscillates, gradually decays)
        - ζ = 1: Critically damped (returns fastest without oscillating)
        - ζ > 1: Overdamped (slow return, no oscillation)

    Example - Car suspension:
        result = await calculate_damped_oscillation(
            mass=300,  # kg (quarter car mass)
            spring_constant=20000,  # N/m
            damping_coefficient=2000,  # kg/s
            time=1.0
        )
        # Should be slightly underdamped for comfort
    """
    from ..oscillations import DampedOscillationRequest, calculate_damped_oscillation as calc_damped

    request = DampedOscillationRequest(
        mass=mass,
        spring_constant=spring_constant,
        damping_coefficient=damping_coefficient,
        initial_position=initial_position,
        initial_velocity=initial_velocity,
        time=time,
    )
    response = calc_damped(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_pendulum_period(
    length: float,
    gravity: float = 9.81,
    amplitude_degrees: Optional[float] = None,
) -> dict:
    """Calculate pendulum period: T = 2π√(L/g).

    Period of a simple pendulum depends only on length and gravity
    (for small amplitudes). Includes correction for large amplitudes.

    Args:
        length: Pendulum length in meters (pivot to center of mass)
        gravity: Gravitational acceleration in m/s² (default 9.81)
        amplitude_degrees: Amplitude in degrees (optional, for large angle correction)

    Returns:
        Dict containing:
            - period: T in seconds
            - frequency: f in Hz
            - angular_frequency: ω in rad/s
            - small_angle_approximation: Whether small angle formula was used

    Tips for LLMs:
        - Period independent of mass (Galileo's discovery)
        - Period independent of amplitude (for small angles < 15°)
        - Longer pendulum → longer period
        - Use for: clocks, playground swings, seismometers

    Example - Grandfather clock:
        result = await calculate_pendulum_period(
            length=0.994,  # meters (for 2-second period)
            gravity=9.81
        )
        # T = 2.0 seconds
    """
    from ..oscillations import PendulumPeriodRequest, calculate_pendulum_period as calc_pendulum

    request = PendulumPeriodRequest(
        length=length,
        gravity=gravity,
        amplitude_degrees=amplitude_degrees,
    )
    response = calc_pendulum(request)
    return response.model_dump()
