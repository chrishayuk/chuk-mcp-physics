#!/usr/bin/env python3
"""Example 3: Force, Energy, and Momentum Calculations

This example shows fundamental physics calculations useful for
engineering analysis, crash testing, and educational purposes.
"""

import asyncio
import math
from chuk_mcp_physics.providers.analytic import AnalyticProvider
from chuk_mcp_physics.models import (
    ForceCalculationRequest,
    KineticEnergyRequest,
    MomentumRequest,
)


async def main():
    """Demonstrate force, energy, and momentum calculations."""

    provider = AnalyticProvider()

    print("=" * 60)
    print("FORCE, ENERGY, AND MOMENTUM CALCULATIONS")
    print("=" * 60)
    print()

    # Example 1: Force calculations (F = ma)
    print("1. Force Calculations (F = ma)")
    print("-" * 60)

    # Car acceleration
    request = ForceCalculationRequest(
        mass=1500.0,  # 1500 kg car
        acceleration=[3.0, 0.0, 0.0],  # 3 m/s² forward acceleration
    )
    result = await provider.calculate_force(request)

    print("Scenario: Car accelerating")
    print(f"  Mass: {request.mass} kg")
    print(f"  Acceleration: {request.acceleration[0]} m/s²")
    print(f"  Force required: {result.force[0]:.0f} N ({result.magnitude:.0f} N magnitude)")
    print()

    # Rocket launch
    request = ForceCalculationRequest(
        mass=50000.0,  # 50,000 kg rocket
        acceleration=[0.0, 20.0, 0.0],  # 20 m/s² upward (2g)
    )
    result = await provider.calculate_force(request)

    print("Scenario: Rocket launch")
    print(f"  Mass: {request.mass:,} kg")
    print(f"  Acceleration: {request.acceleration[1]} m/s² (upward)")
    print(f"  Thrust required: {result.magnitude:,.0f} N")
    print(f"  That's {result.magnitude / 9.81 / 1000:.1f} metric tons of force!")
    print()

    # Braking force
    request = ForceCalculationRequest(
        mass=1500.0,
        acceleration=[-8.0, 0.0, 0.0],  # Hard braking (deceleration)
    )
    result = await provider.calculate_force(request)

    print("Scenario: Emergency braking")
    print(f"  Mass: {request.mass} kg")
    print(f"  Deceleration: {abs(request.acceleration[0])} m/s²")
    print(f"  Braking force: {abs(result.force[0]):.0f} N")
    print()

    # Example 2: Kinetic Energy (KE = 0.5 * m * v²)
    print("2. Kinetic Energy Calculations (KE = ½mv²)")
    print("-" * 60)

    # Car at highway speed
    request = KineticEnergyRequest(
        mass=1500.0,
        velocity=[30.0, 0.0, 0.0],  # 30 m/s (~67 mph)
    )
    result = await provider.calculate_kinetic_energy(request)

    print("Scenario: Car at highway speed")
    print(f"  Mass: {request.mass} kg")
    print(f"  Speed: {result.speed} m/s ({result.speed * 2.237:.1f} mph)")
    print(f"  Kinetic energy: {result.kinetic_energy:,.0f} J")
    print(f"  Equivalent to lifting {result.kinetic_energy / 9.81:.0f} kg by 1 meter!")
    print()

    # Baseball pitch
    request = KineticEnergyRequest(
        mass=0.145,  # 145g baseball
        velocity=[40.0, 0.0, 0.0],  # 40 m/s (~90 mph)
    )
    result = await provider.calculate_kinetic_energy(request)

    print("Scenario: Baseball pitch")
    print(f"  Mass: {request.mass} kg")
    print(f"  Speed: {result.speed} m/s ({result.speed * 2.237:.1f} mph)")
    print(f"  Kinetic energy: {result.kinetic_energy:.1f} J")
    print()

    # Energy comparison at different speeds
    print("Energy comparison (1500 kg car at different speeds):")
    print(f"{'Speed (mph)':<15} {'Speed (m/s)':<15} {'Energy (J)':<15} {'Relative':<15}")
    print("-" * 60)

    speeds_mph = [10, 30, 50, 70, 90]
    energies = []

    for speed_mph in speeds_mph:
        speed_ms = speed_mph / 2.237
        request = KineticEnergyRequest(mass=1500.0, velocity=[speed_ms, 0.0, 0.0])
        result = await provider.calculate_kinetic_energy(request)
        energies.append(result.kinetic_energy)

        relative = result.kinetic_energy / energies[0] if energies[0] > 0 else 1.0
        print(
            f"{speed_mph:<15} {speed_ms:<15.1f} {result.kinetic_energy:<15,.0f} {relative:<15.1f}x"
        )

    print("\nNote: Energy increases with the SQUARE of speed!")
    print("Doubling speed = 4x energy, tripling speed = 9x energy")
    print()

    # Example 3: Momentum (p = mv)
    print("3. Momentum Calculations (p = mv)")
    print("-" * 60)

    # Truck vs car
    scenarios = [
        ("Truck", 10000.0, [15.0, 0.0, 0.0]),
        ("Car", 1500.0, [30.0, 0.0, 0.0]),
        ("Motorcycle", 200.0, [40.0, 0.0, 0.0]),
    ]

    print("Comparing momentum of different vehicles:")
    print(f"{'Vehicle':<15} {'Mass (kg)':<15} {'Speed (m/s)':<15} {'Momentum (kg·m/s)':<20}")
    print("-" * 60)

    for name, mass, velocity in scenarios:
        request = MomentumRequest(mass=mass, velocity=velocity)
        result = await provider.calculate_momentum(request)

        print(f"{name:<15} {mass:<15,.0f} {velocity[0]:<15.1f} {result.magnitude:<20,.0f}")

    print()
    print("Note: Truck has same momentum as car despite lower speed!")
    print()

    # Collision momentum conservation
    print("4. Collision Analysis (Momentum Conservation)")
    print("-" * 60)

    # Two cars colliding
    car1_mass = 1500.0
    car1_velocity = [20.0, 0.0, 0.0]
    car2_mass = 1200.0
    car2_velocity = [-15.0, 0.0, 0.0]

    # Calculate individual momenta
    request1 = MomentumRequest(mass=car1_mass, velocity=car1_velocity)
    result1 = await provider.calculate_momentum(request1)

    request2 = MomentumRequest(mass=car2_mass, velocity=car2_velocity)
    result2 = await provider.calculate_momentum(request2)

    # Calculate total momentum (vector sum)
    total_momentum = [
        result1.momentum[0] + result2.momentum[0],
        result1.momentum[1] + result2.momentum[1],
        result1.momentum[2] + result2.momentum[2],
    ]
    total_mass = car1_mass + car2_mass

    # Final velocity (assuming they stick together)
    final_velocity = [p / total_mass for p in total_momentum]

    print("Before collision:")
    print(
        f"  Car 1 ({car1_mass}kg at {car1_velocity[0]}m/s): momentum = {result1.magnitude:,.0f} kg·m/s"
    )
    print(
        f"  Car 2 ({car2_mass}kg at {car2_velocity[0]}m/s): momentum = {result2.magnitude:,.0f} kg·m/s"
    )
    print()
    print("After collision (if stuck together):")
    print(f"  Total mass: {total_mass} kg")
    print(f"  Total momentum: {math.sqrt(sum(p**2 for p in total_momentum)):,.0f} kg·m/s")
    print(f"  Final velocity: {final_velocity[0]:.2f} m/s")
    print()

    # Calculate energy loss
    ke1 = KineticEnergyRequest(mass=car1_mass, velocity=car1_velocity)
    ke1_result = await provider.calculate_kinetic_energy(ke1)

    ke2 = KineticEnergyRequest(mass=car2_mass, velocity=car2_velocity)
    ke2_result = await provider.calculate_kinetic_energy(ke2)

    initial_ke = ke1_result.kinetic_energy + ke2_result.kinetic_energy

    final_ke_request = KineticEnergyRequest(mass=total_mass, velocity=final_velocity)
    final_ke_result = await provider.calculate_kinetic_energy(final_ke_request)

    energy_loss = initial_ke - final_ke_result.kinetic_energy

    print("Energy analysis:")
    print(f"  Initial kinetic energy: {initial_ke:,.0f} J")
    print(f"  Final kinetic energy: {final_ke_result.kinetic_energy:,.0f} J")
    print(
        f"  Energy lost to deformation/heat: {energy_loss:,.0f} J ({energy_loss / initial_ke * 100:.1f}%)"
    )
    print()
    print("Momentum is conserved, but energy is not (inelastic collision)!")
    print()

    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
