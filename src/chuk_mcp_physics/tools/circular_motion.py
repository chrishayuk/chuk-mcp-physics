"""Circular motion MCP tool endpoints."""

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def calculate_centripetal_force(
    mass: float,
    velocity: float,
    radius: float,
) -> dict:
    """Calculate centripetal force: F_c = m v² / r.

    Force required to keep an object moving in a circle.
    Always points toward the center of the circular path.

    Args:
        mass: Mass in kg
        velocity: Speed (velocity magnitude) in m/s
        radius: Radius of circular path in meters

    Returns:
        Dict containing:
            - centripetal_force: F_c in Newtons
            - centripetal_acceleration: a_c in m/s²

    Tips for LLMs:
        - Not a new force - it's the net inward force (tension, friction, gravity)
        - Faster speed → much more force needed (v² relationship)
        - Tighter turn → more force needed
        - Use for: car turns, satellite orbits, centrifuges

    Example - Car turning:
        result = await calculate_centripetal_force(
            mass=1500,  # kg
            velocity=20,  # m/s (72 km/h)
            radius=50  # meter turn radius
        )
        # F_c = 12000 N (provided by friction between tires and road)
    """
    from ..circular_motion import CentripetalForceRequest, calculate_centripetal_force as calc_fc

    request = CentripetalForceRequest(
        mass=mass,
        velocity=velocity,
        radius=radius,
    )
    response = calc_fc(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_orbital_period(
    orbital_radius: float,
    central_mass: float,
    gravitational_constant: float = 6.674e-11,
) -> dict:
    """Calculate orbital period: T = 2π√(r³/GM).

    Kepler's Third Law for circular orbits. Period depends on orbital
    radius and central body mass.

    Args:
        orbital_radius: Orbital radius in meters (from center of central body)
        central_mass: Mass of central body in kg
        gravitational_constant: G in m³/(kg⋅s²) (default 6.674e-11)

    Returns:
        Dict containing:
            - period: Orbital period in seconds
            - orbital_velocity: v in m/s
            - period_hours: Period in hours (for convenience)
            - period_days: Period in days (for convenience)

    Tips for LLMs:
        - Higher orbit → longer period
        - More massive central body → shorter period
        - Earth: M = 5.972e24 kg, R = 6.371e6 m
        - Moon orbit: r ≈ 384,400 km, T ≈ 27.3 days
        - ISS orbit: r ≈ 6,771 km (altitude 400 km), T ≈ 90 minutes

    Example - ISS orbit:
        result = await calculate_orbital_period(
            orbital_radius=6.771e6,  # meters
            central_mass=5.972e24  # Earth mass (kg)
        )
        # T ≈ 5,558 seconds ≈ 92.6 minutes
    """
    from ..circular_motion import OrbitalPeriodRequest, calculate_orbital_period as calc_orbit

    request = OrbitalPeriodRequest(
        orbital_radius=orbital_radius,
        central_mass=central_mass,
        gravitational_constant=gravitational_constant,
    )
    response = calc_orbit(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_banking_angle(
    velocity: float,
    radius: float,
    gravity: float = 9.81,
) -> dict:
    """Calculate ideal banking angle: θ = arctan(v² / (rg)).

    For a banked curve, the ideal angle where no friction is needed
    to maintain the turn at a given speed.

    Args:
        velocity: Speed in m/s
        radius: Turn radius in meters
        gravity: Gravitational acceleration in m/s² (default 9.81)

    Returns:
        Dict containing:
            - angle_radians: Banking angle in radians
            - angle_degrees: Banking angle in degrees

    Tips for LLMs:
        - Faster speed → steeper banking angle
        - Tighter turn → steeper banking angle
        - NASCAR tracks banked ~30° for high-speed turns
        - At ideal angle, normal force provides all centripetal force

    Example - Highway exit ramp:
        result = await calculate_banking_angle(
            velocity=25,  # m/s (90 km/h)
            radius=100  # meter radius turn
        )
        # θ ≈ 32.5°
    """
    from ..circular_motion import BankingAngleRequest, calculate_banking_angle as calc_bank

    request = BankingAngleRequest(
        velocity=velocity,
        radius=radius,
        gravity=gravity,
    )
    response = calc_bank(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_escape_velocity(
    mass: float,
    radius: float,
    gravitational_constant: float = 6.674e-11,
) -> dict:
    """Calculate escape velocity: v_escape = √(2GM/r).

    Minimum speed needed to escape a celestial body's gravitational pull.
    Independent of the escaping object's mass.

    Args:
        mass: Mass of celestial body in kg
        radius: Radius of celestial body in meters
        gravitational_constant: G in m³/(kg⋅s²) (default 6.674e-11)

    Returns:
        Dict containing:
            - escape_velocity: v_escape in m/s
            - escape_velocity_kmh: v_escape in km/h (for convenience)

    Tips for LLMs:
        - Earth: v_escape ≈ 11,200 m/s (40,320 km/h)
        - Moon: v_escape ≈ 2,380 m/s
        - Sun: v_escape ≈ 617,500 m/s
        - Independent of escape direction or mass of escaping object

    Example - Earth escape velocity:
        result = await calculate_escape_velocity(
            mass=5.972e24,  # Earth mass (kg)
            radius=6.371e6  # Earth radius (meters)
        )
        # v_escape ≈ 11,186 m/s
    """
    from ..circular_motion import EscapeVelocityRequest, calculate_escape_velocity as calc_escape

    request = EscapeVelocityRequest(
        mass=mass,
        radius=radius,
        gravitational_constant=gravitational_constant,
    )
    response = calc_escape(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def analyze_circular_orbit(
    altitude: float,
    planet_mass: float,
    planet_radius: float,
    gravitational_constant: float = 6.674e-11,
) -> dict:
    """Analyze circular orbit at given altitude above planet surface.

    Comprehensive orbital analysis combining period, velocity, and acceleration.

    Args:
        altitude: Altitude above surface in meters
        planet_mass: Planet mass in kg
        planet_radius: Planet radius in meters
        gravitational_constant: G in m³/(kg⋅s²) (default 6.674e-11)

    Returns:
        Dict containing:
            - orbital_radius: r from planet center in meters
            - orbital_velocity: v in m/s
            - period_seconds: Orbital period in seconds
            - period_minutes: Orbital period in minutes
            - centripetal_acceleration: a_c in m/s²

    Example - LEO satellite at 400km altitude:
        result = await analyze_circular_orbit(
            altitude=400000,  # 400 km
            planet_mass=5.972e24,  # Earth
            planet_radius=6.371e6  # Earth
        )
        # v ≈ 7,670 m/s, T ≈ 92.6 min
    """
    from ..circular_motion import CircularOrbitRequest, analyze_circular_orbit as analyze_orbit

    request = CircularOrbitRequest(
        altitude=altitude,
        planet_mass=planet_mass,
        planet_radius=planet_radius,
        gravitational_constant=gravitational_constant,
    )
    response = analyze_orbit(request)
    return response.model_dump()
