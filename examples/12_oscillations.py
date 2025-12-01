"""
Example 12: Oscillations & Harmonic Motion (Phase 2.2)

Demonstrates oscillation calculations including:
- Hooke's law (springs)
- Spring-mass systems
- Simple harmonic motion
- Damped oscillations
- Pendulum motion

No external services required - uses analytic provider.
"""

import asyncio
from chuk_mcp_physics.tools.oscillations import (
    calculate_hookes_law,
    calculate_spring_mass_period,
    calculate_simple_harmonic_motion,
    calculate_damped_oscillation,
    calculate_pendulum_period,
)


async def main():
    print("\n" + "=" * 70)
    print("OSCILLATIONS & HARMONIC MOTION EXAMPLES (Phase 2.2)")
    print("=" * 70)

    # Example 1: Hooke's Law - Spring compression
    print("\n1. Hooke's Law - Car Suspension Spring")
    print("-" * 70)
    print("Scenario: Car suspension spring compressed by 5 cm")

    result = await calculate_hookes_law(
        spring_constant=10000.0,  # N/m (stiff spring)
        displacement=0.05,  # 5 cm compression
    )

    print("\nSpring Specifications:")
    print("  Spring constant (k): 10,000 N/m")
    print("  Compression: 5 cm (0.05 m)")

    print("\nRESULTS:")
    print(f"  Force: {result['force']:.1f} N ({result['force'] / 9.81:.1f} kg equivalent)")
    print(f"  Potential energy stored: {result['potential_energy']:.1f} J")
    print(
        f"  Energy interpretation: Can lift {result['potential_energy'] / 9.81:.1f} kg by 1 meter"
    )

    # Example 2: Spring-Mass System Period
    print("\n\n2. Spring-Mass System - Mass Oscillating on Spring")
    print("-" * 70)
    print("Scenario: 500g mass attached to spring")

    result = await calculate_spring_mass_period(
        mass=0.5,  # 500g
        spring_constant=20.0,  # N/m
    )

    print("\nSystem Specifications:")
    print("  Mass: 500 g")
    print("  Spring constant: 20 N/m")

    print("\nOscillation Properties:")
    print(f"  Period (T): {result['period']:.3f} seconds")
    print(f"  Frequency (f): {result['frequency']:.2f} Hz")
    print(f"  Angular frequency (ω): {result['angular_frequency']:.2f} rad/s")
    print(f"  Meaning: Completes one oscillation every {result['period']:.3f} seconds")

    # Example 3: Simple Harmonic Motion - Time evolution
    print("\n\n3. Simple Harmonic Motion - Position vs. Time")
    print("-" * 70)
    print("Scenario: Mass oscillating with 10 cm amplitude")

    amplitude = 0.1  # 10 cm
    omega = result["angular_frequency"]  # From previous calculation

    print(
        f"\n{'Time (s)':<12} {'Position (cm)':<15} {'Velocity (cm/s)':<18} {'Acceleration (cm/s²)':<20}"
    )
    print("-" * 70)

    for t in [0, 0.25, 0.5, 0.75, 1.0]:
        result_shm = await calculate_simple_harmonic_motion(
            amplitude=amplitude, angular_frequency=omega, time=t, phase=0.0
        )
        print(
            f"{t:<12.2f} {result_shm['position'] * 100:<15.2f} {result_shm['velocity'] * 100:<18.2f} {result_shm['acceleration'] * 100:<20.2f}"
        )

    # Example 4: Damped Oscillation - Car shock absorber
    print("\n\n4. Damped Oscillation - Automotive Shock Absorber")
    print("-" * 70)
    print("Scenario: Analyzing different damping regimes")

    mass = 250.0  # kg (quarter car mass)
    spring_constant = 25000.0  # N/m

    damping_cases = [
        ("Underdamped (bouncy)", 1000.0),
        ("Critically damped (optimal)", 5000.0),
        ("Overdamped (sluggish)", 10000.0),
    ]

    print(
        f"\n{'Damping Type':<30} {'Coefficient (N⋅s/m)':<20} {'Damping Ratio':<15} {'Regime':<15}"
    )
    print("-" * 70)

    for name, damping_coef in damping_cases:
        result_damped = await calculate_damped_oscillation(
            mass=mass, spring_constant=spring_constant, damping_coefficient=damping_coef, time=0.0
        )
        print(
            f"{name:<30} {damping_coef:<20.0f} {result_damped['damping_ratio']:<15.3f} {result_damped['regime']:<15}"
        )

    # Analyze underdamped case over time
    print("\nUnderdamped Response Over Time (1000 N⋅s/m):")
    print(f"{'Time (s)':<12} {'Position (cm)':<15} {'Velocity (m/s)':<15} {'Amplitude Decay':<20}")
    print("-" * 70)

    initial_position = 0.05  # 5 cm initial displacement
    for t in [0, 0.2, 0.4, 0.6, 0.8]:
        result_damped = await calculate_damped_oscillation(
            mass=mass,
            spring_constant=spring_constant,
            damping_coefficient=1000.0,
            time=t,
            initial_position=initial_position,
        )
        print(
            f"{t:<12.1f} {result_damped['position'] * 100:<15.2f} {result_damped['velocity']:<15.2f} {(result_damped['position'] / initial_position) * 100:<20.1f}%"
        )

    # Example 5: Pendulum Period - Various lengths
    print("\n\n5. Pendulum Period - Effect of Length")
    print("-" * 70)
    print("Scenario: Comparing pendulums of different lengths")

    print(f"\n{'Length (m)':<15} {'Period (s)':<15} {'Frequency (Hz)':<15} {'Application':<30}")
    print("-" * 70)

    pendulum_lengths = [
        (0.248, "Metronome (120 BPM)"),
        (0.994, "Grandfather clock (1 Hz)"),
        (2.0, "Playground swing"),
        (10.0, "Foucault pendulum"),
    ]

    for length, application in pendulum_lengths:
        result_pendulum = await calculate_pendulum_period(length=length)
        print(
            f"{length:<15.3f} {result_pendulum['period']:<15.2f} {result_pendulum['frequency']:<15.3f} {application:<30}"
        )

    # Example 6: Real-world - Seismometer
    print("\n\n6. Real-World Application - Seismometer")
    print("-" * 70)
    print("Scenario: Detecting earthquake vibrations")

    # Seismometer: mass on spring sensitive to ground motion
    seismo_mass = 10.0  # kg
    seismo_k = 50.0  # N/m (soft spring for low frequencies)

    result_seismo = await calculate_spring_mass_period(mass=seismo_mass, spring_constant=seismo_k)

    print("\nSeismometer Design:")
    print(f"  Mass: {seismo_mass:.1f} kg")
    print(f"  Spring constant: {seismo_k:.1f} N/m")
    print(f"  Natural period: {result_seismo['period']:.2f} seconds")
    print(f"  Natural frequency: {result_seismo['frequency']:.3f} Hz")

    print("\nEarthquake Detection:")
    print(
        f"  Can detect frequencies from ~{result_seismo['frequency'] * 0.5:.3f} to ~{result_seismo['frequency'] * 2:.3f} Hz"
    )
    print("  Typical earthquake frequencies: 0.5 - 10 Hz")
    print("  ✓ This design is suitable for earthquake detection")

    # Example 7: Energy in oscillating system
    print("\n\n7. Energy Analysis - Oscillating Mass on Spring")
    print("-" * 70)
    print("Scenario: Energy transfer between kinetic and potential")

    mass_energy = 1.0  # kg
    k_energy = 100.0  # N/m
    amplitude_energy = 0.05  # 5 cm

    result_period = await calculate_spring_mass_period(mass=mass_energy, spring_constant=k_energy)
    omega_energy = result_period["angular_frequency"]

    # Total energy = ½kA² (constant)
    total_energy = 0.5 * k_energy * amplitude_energy**2

    print("\nSystem Properties:")
    print(f"  Total mechanical energy: {total_energy:.3f} J (constant)")
    print(f"\n{'Time':<12} {'Position':<12} {'KE (J)':<12} {'PE (J)':<12} {'Total (J)':<12}")
    print("-" * 70)

    for t in [0, 0.157, 0.314, 0.471, 0.628]:  # Quarter periods
        result_energy = await calculate_simple_harmonic_motion(
            amplitude=amplitude_energy, angular_frequency=omega_energy, time=t, phase=0.0
        )

        x = result_energy["position"]
        v = result_energy["velocity"]

        KE = 0.5 * mass_energy * v**2
        PE = 0.5 * k_energy * x**2
        total = KE + PE

        print(f"{t:<12.3f} {x * 100:<12.2f} {KE:<12.3f} {PE:<12.3f} {total:<12.3f}")

    print("\n  Energy oscillates between kinetic and potential")
    print("  At extremes: All PE, zero KE")
    print("  At equilibrium: All KE, zero PE")

    print("\n" + "=" * 70)
    print("Phase 2.2 Complete! ✓")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
