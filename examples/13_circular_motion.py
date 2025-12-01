"""
Example 13: Circular Motion & Orbits (Phase 2.3)

Demonstrates circular motion calculations including:
- Centripetal force
- Orbital mechanics
- Banking angles
- Escape velocity
- Complete orbital analysis

No external services required - uses analytic provider.
"""

import asyncio
from chuk_mcp_physics.tools.circular_motion import (
    calculate_centripetal_force,
    calculate_banking_angle,
    calculate_escape_velocity,
    analyze_circular_orbit,
)


async def main():
    print("\n" + "=" * 70)
    print("CIRCULAR MOTION & ORBITS EXAMPLES (Phase 2.3)")
    print("=" * 70)

    # Example 1: Centripetal Force - Car turning
    print("\n1. Centripetal Force - Car on Curved Road")
    print("-" * 70)

    result = await calculate_centripetal_force(
        mass=1500.0,  # kg
        velocity=20.0,  # m/s (~72 km/h)
        radius=50.0,  # m
    )

    print("Scenario: 1500 kg car turning at 20 m/s on 50m radius curve")
    print("\nRESULTS:")
    print(f"  Centripetal force: {result['centripetal_force']:.0f} N")
    print(f"  Centripetal acceleration: {result['centripetal_acceleration']:.1f} m/s²")
    print(f"  As fraction of g: {result['centripetal_acceleration'] / 9.81:.2f}g")
    print("  Tire friction must provide this force!")

    # Example 2: ISS Orbital Period
    print("\n\n2. International Space Station Orbit")
    print("-" * 70)

    result = await analyze_circular_orbit(
        altitude=408000.0,  # 408 km
        planet_mass=5.972e24,  # Earth mass
        planet_radius=6.371e6,  # Earth radius
    )

    print("ISS Altitude: 408 km above Earth")
    print("\nOrbital Properties:")
    print(f"  Orbital radius: {result['orbital_radius'] / 1000:.0f} km")
    print(
        f"  Orbital velocity: {result['orbital_velocity']:.0f} m/s ({result['orbital_velocity'] * 3.6:.0f} km/h)"
    )
    print(
        f"  Period: {result['period_seconds']:.0f} seconds ({result['period_minutes']:.1f} minutes)"
    )
    print(f"  Orbits per day: {86400 / result['period_seconds']:.1f}")

    # Example 3: Banking Angle for Highway Curve
    print("\n\n3. Optimal Banking Angle - Highway Design")
    print("-" * 70)

    speeds_kmh = [60, 80, 100, 120]
    radius = 200.0  # m

    print(f"Curve radius: {radius} m")
    print(f"\n{'Speed (km/h)':<15} {'Speed (m/s)':<15} {'Banking Angle':<20}")
    print("-" * 70)

    for speed_kmh in speeds_kmh:
        speed_ms = speed_kmh / 3.6
        result = await calculate_banking_angle(velocity=speed_ms, radius=radius)
        print(f"{speed_kmh:<15} {speed_ms:<15.1f} {result['angle_degrees']:<20.1f}°")

    # Example 4: Escape Velocities
    print("\n\n4. Escape Velocities - Various Celestial Bodies")
    print("-" * 70)

    bodies = [
        ("Moon", 7.342e22, 1.737e6),
        ("Earth", 5.972e24, 6.371e6),
        ("Jupiter", 1.898e27, 6.9911e7),
        ("Sun", 1.989e30, 6.96e8),
    ]

    print(f"\n{'Body':<15} {'Mass (kg)':<20} {'Radius (km)':<15} {'Escape Velocity':<20}")
    print("-" * 70)

    for name, mass, radius in bodies:
        result = await calculate_escape_velocity(mass=mass, radius=radius)
        print(
            f"{name:<15} {mass:<20.2e} {radius / 1000:<15.0f} {result['escape_velocity']:.0f} m/s ({result['escape_velocity'] * 3.6:.0f} km/h)"
        )

    # Example 5: Geostationary Orbit
    print("\n\n5. Geostationary Satellite Orbit")
    print("-" * 70)
    print("Finding altitude for 24-hour orbital period...")

    # Geostationary: T = 24 hours = 86400 seconds
    # From T² = (4π²/GM) * r³, solve for r
    G = 6.674e-11
    M_earth = 5.972e24
    T_geo = 86400.0

    r_orbit = ((G * M_earth * T_geo**2) / (4 * 3.14159**2)) ** (1 / 3)
    altitude = r_orbit - 6.371e6

    result = await analyze_circular_orbit(
        altitude=altitude, planet_mass=M_earth, planet_radius=6.371e6
    )

    print("\nGeostationary Orbit:")
    print(f"  Altitude: {altitude / 1000:.0f} km")
    print(f"  Orbital radius: {result['orbital_radius'] / 1000:.0f} km")
    print(f"  Orbital velocity: {result['orbital_velocity']:.0f} m/s")
    print(f"  Period: {result['period_seconds'] / 3600:.1f} hours")
    print("  ✓ Stays above same point on Earth's equator!")

    print("\n" + "=" * 70)
    print("Phase 2.3 Complete! ✓")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
