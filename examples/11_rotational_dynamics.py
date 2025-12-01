"""
Example 11: Rotational Dynamics (Phase 2.1)

Demonstrates rotational motion calculations including:
- Torque calculations
- Moment of inertia for various shapes
- Angular momentum
- Rotational kinetic energy
- Angular acceleration

No external services required - uses analytic provider.
"""

import asyncio
from chuk_mcp_physics.tools.rotational import (
    calculate_torque,
    calculate_moment_of_inertia,
    calculate_angular_momentum,
    calculate_rotational_kinetic_energy,
    calculate_angular_acceleration,
)


async def main():
    print("\n" + "=" * 70)
    print("ROTATIONAL DYNAMICS EXAMPLES (Phase 2.1)")
    print("=" * 70)

    # Example 1: Torque - Wrench applying force
    print("\n1. Torque Calculation - Wrench on Bolt")
    print("-" * 70)
    print("Scenario: 50 N force applied perpendicular to 80 cm wrench")

    result = await calculate_torque(
        force_x=50.0,
        force_y=0.0,
        force_z=0.0,
        position_x=0.0,
        position_y=0.0,
        position_z=0.8,  # 80 cm wrench
    )

    print("\nRESULTS:")
    print(f"  Torque vector: {result['torque']}")
    print(f"  Torque magnitude: {result['magnitude']:.1f} N⋅m")
    print(f"  Direction: {'Counter-clockwise' if result['torque'][1] > 0 else 'Clockwise'}")

    # Example 2: Moment of Inertia - Various shapes
    print("\n\n2. Moment of Inertia - Comparing Shapes")
    print("-" * 70)
    print("Comparing rotational inertia of different shapes with same mass")

    shapes = [
        ("disk", {"mass": 5.0, "radius": 0.3}),
        ("sphere", {"mass": 5.0, "radius": 0.3}),
        ("hollow_sphere", {"mass": 5.0, "radius": 0.3}),
        ("rod", {"mass": 5.0, "length": 1.0}),
    ]

    print(f"\n{'Shape':<20} {'Mass (kg)':<12} {'Radius/Length':<15} {'I (kg⋅m²)':<12}")
    print("-" * 70)

    for shape_name, params in shapes:
        result = await calculate_moment_of_inertia(shape=shape_name, **params)
        dimension = params.get("radius", params.get("length", 0))
        print(
            f"{shape_name:<20} {params['mass']:<12.1f} {dimension:<15.2f} {result['moment_of_inertia']:<12.4f}"
        )

    # Example 3: Angular Momentum - Spinning figure skater
    print("\n\n3. Angular Momentum - Figure Skater Spin")
    print("-" * 70)
    print("Scenario: Skater spins with arms extended, then pulls arms in")

    # Arms extended (approximated as disk)
    I_extended = (
        await calculate_moment_of_inertia(
            shape="disk",
            mass=60.0,
            radius=0.8,  # Arms extended
        )
    )["moment_of_inertia"]

    # Arms pulled in
    I_compact = (
        await calculate_moment_of_inertia(
            shape="disk",
            mass=60.0,
            radius=0.3,  # Arms pulled in
        )
    )["moment_of_inertia"]

    omega_initial = 2.0  # rad/s (slow spin)

    # Calculate initial angular momentum
    L_initial = await calculate_angular_momentum(
        moment_of_inertia=I_extended,
        angular_velocity_x=0.0,
        angular_velocity_y=omega_initial,
        angular_velocity_z=0.0,
    )

    # Conservation of angular momentum: L = I*ω = constant
    # So: I_extended * ω_initial = I_compact * ω_final
    omega_final = (I_extended * omega_initial) / I_compact

    L_final = await calculate_angular_momentum(
        moment_of_inertia=I_compact,
        angular_velocity_x=0.0,
        angular_velocity_y=omega_final,
        angular_velocity_z=0.0,
    )

    print("\nArms Extended:")
    print(f"  I = {I_extended:.2f} kg⋅m²")
    print(f"  ω = {omega_initial:.2f} rad/s")
    print(f"  L = {L_initial['magnitude']:.2f} kg⋅m²/s")

    print("\nArms Pulled In:")
    print(f"  I = {I_compact:.2f} kg⋅m²")
    print(f"  ω = {omega_final:.2f} rad/s ({omega_final / omega_initial:.1f}x faster!)")
    print(f"  L = {L_final['magnitude']:.2f} kg⋅m²/s")

    print(
        f"\nConservation check: ΔL = {abs(L_initial['magnitude'] - L_final['magnitude']):.6f} (should be ~0)"
    )

    # Example 4: Rotational Kinetic Energy - Flywheel
    print("\n\n4. Rotational Kinetic Energy - Flywheel Energy Storage")
    print("-" * 70)
    print("Scenario: Industrial flywheel storing energy")

    # Flywheel specs
    mass = 500.0  # kg
    radius = 1.0  # m
    rpm = 3000  # revolutions per minute
    omega = (rpm * 2 * 3.14159) / 60  # Convert to rad/s

    moment_of_inertia = (await calculate_moment_of_inertia(shape="disk", mass=mass, radius=radius))[
        "moment_of_inertia"
    ]

    result = await calculate_rotational_kinetic_energy(
        moment_of_inertia=moment_of_inertia, angular_velocity=omega
    )

    print("\nFlywheel Specifications:")
    print(f"  Mass: {mass:.0f} kg")
    print(f"  Radius: {radius:.1f} m")
    print(f"  Speed: {rpm:.0f} RPM ({omega:.1f} rad/s)")
    print(f"  Moment of inertia: {moment_of_inertia:.2f} kg⋅m²")

    print("\nEnergy Storage:")
    print(f"  Rotational KE: {result['rotational_ke']:.0f} J")
    print(f"  In kWh: {result['rotational_ke'] / 3.6e6:.2f} kWh")
    print(f"  Equivalent to lifting {result['rotational_ke'] / 9.81:.0f} kg by 1 meter")

    # Example 5: Angular Acceleration - Motor starting
    print("\n\n5. Angular Acceleration - Electric Motor Startup")
    print("-" * 70)
    print("Scenario: Motor applies torque to accelerate rotor")

    torque = 50.0  # N⋅m
    I_motor = (await calculate_moment_of_inertia(shape="disk", mass=10.0, radius=0.15))[
        "moment_of_inertia"
    ]

    result = await calculate_angular_acceleration(torque=torque, moment_of_inertia=I_motor)

    alpha = result["angular_acceleration"]
    time_to_3000rpm = (3000 * 2 * 3.14159 / 60) / alpha  # Time to reach 3000 RPM

    print("\nMotor Specifications:")
    print(f"  Applied torque: {torque:.1f} N⋅m")
    print(f"  Rotor I: {I_motor:.4f} kg⋅m²")

    print("\nAcceleration:")
    print(f"  Angular acceleration: {alpha:.1f} rad/s²")
    print(f"  Time to 3000 RPM: {time_to_3000rpm:.2f} seconds")
    print(f"  Linear analogy: Like accelerating at {alpha:.1f} m/s² at 1m radius")

    # Example 6: Real-world application - Bicycle wheel
    print("\n\n6. Real-World Application - Bicycle Wheel")
    print("-" * 70)
    print("Scenario: Analyzing bicycle wheel rotation")

    wheel_mass = 2.0  # kg
    wheel_radius = 0.35  # m (700c wheel)
    speed_kmh = 25.0  # km/h
    speed_ms = speed_kmh / 3.6

    # Angular velocity: v = ωr, so ω = v/r
    omega_wheel = speed_ms / wheel_radius

    I_wheel = (
        await calculate_moment_of_inertia(shape="cylinder", mass=wheel_mass, radius=wheel_radius)
    )["moment_of_inertia"]

    L_wheel = await calculate_angular_momentum(
        moment_of_inertia=I_wheel,
        angular_velocity_x=0.0,
        angular_velocity_y=omega_wheel,
        angular_velocity_z=0.0,
    )

    KE_wheel = await calculate_rotational_kinetic_energy(
        moment_of_inertia=I_wheel, angular_velocity=omega_wheel
    )

    print("\nWheel Specifications:")
    print(f"  Mass: {wheel_mass:.1f} kg")
    print(f"  Radius: {wheel_radius:.2f} m")
    print(f"  Bike speed: {speed_kmh:.1f} km/h ({speed_ms:.2f} m/s)")
    print(f"  Wheel RPM: {omega_wheel * 60 / (2 * 3.14159):.0f}")

    print("\nRotational Properties:")
    print(f"  Angular velocity: {omega_wheel:.1f} rad/s")
    print(f"  Moment of inertia: {I_wheel:.4f} kg⋅m²")
    print(f"  Angular momentum: {L_wheel['magnitude']:.2f} kg⋅m²/s")
    print(f"  Rotational KE: {KE_wheel['rotational_ke']:.1f} J")
    print("  (This gyroscopic effect helps bike stay upright!)")

    print("\n" + "=" * 70)
    print("Phase 2.1 Complete! ✓")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
