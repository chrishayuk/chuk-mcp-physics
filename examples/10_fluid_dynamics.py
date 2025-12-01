#!/usr/bin/env python3
"""Fluid Dynamics Demo: Drag, Buoyancy, and Underwater Motion.

This demonstrates the new fluid dynamics capabilities:
- Drag force calculations
- Buoyancy (Archimedes' principle)
- Terminal velocity
- Underwater projectile motion simulation
"""

import asyncio
import math

from chuk_mcp_physics.tools.fluid import (
    calculate_drag_force,
    calculate_buoyancy,
    calculate_terminal_velocity,
    simulate_underwater_motion,
)


async def demo_drag_force():
    """Demo: Calculate drag force on a falling ball."""
    print("\n" + "=" * 70)
    print("DEMO 1: Drag Force on Falling Ball")
    print("=" * 70)

    # 10cm diameter ball falling at 5 m/s through water
    print("\nScenario: 10cm diameter ball falling through water at 5 m/s")
    print("  - Ball diameter: 0.1 m")
    print("  - Velocity: 5 m/s downward")
    print("  - Fluid: Water (density = 1000 kg/m³)")
    print("  - Shape: Sphere (Cd = 0.47)")

    radius = 0.05  # 5cm radius
    cross_sectional_area = math.pi * radius * radius  # π*r²

    result = await calculate_drag_force(
        velocity=[0.0, -5.0, 0.0],  # Falling downward
        cross_sectional_area=cross_sectional_area,
        fluid_density=1000.0,  # water
        drag_coefficient=0.47,  # sphere
    )

    print(
        f"\n  Drag force: [{result['drag_force'][0]:.2f}, {result['drag_force'][1]:.2f}, {result['drag_force'][2]:.2f}] N"
    )
    print(f"  Magnitude: {result['magnitude']:.2f} N (upward, opposing motion)")
    print(f"  Reynolds number: {result['reynolds_number']:.0f} (turbulent flow)")


async def demo_buoyancy():
    """Demo: Check if objects float or sink."""
    print("\n" + "=" * 70)
    print("DEMO 2: Buoyancy - Will it Float?")
    print("=" * 70)

    # Test 1: Steel ball
    print("\nTest 1: Steel ball (1 kg, 10cm diameter)")
    radius = 0.05
    volume = (4.0 / 3.0) * math.pi * radius**3  # V = (4/3)πr³
    mass = 1.0  # kg

    result = await calculate_buoyancy(
        volume=volume,
        fluid_density=1000.0,  # water
    )

    weight = mass * 9.81  # Weight in Newtons
    print(f"  Volume: {volume:.6f} m³")
    print(f"  Weight: {weight:.2f} N")
    print(f"  Buoyant force: {result['buoyant_force']:.2f} N")
    print(f"  Displaced water: {result['displaced_mass']:.3f} kg")

    if weight > result["buoyant_force"]:
        print("  Result: ⬇️  SINKS (weight > buoyancy)")
    else:
        print("  Result: ⬆️  FLOATS (buoyancy > weight)")

    # Test 2: Ping pong ball (hollow, light)
    print("\nTest 2: Ping pong ball (2.7g, 4cm diameter)")
    radius = 0.02
    volume = (4.0 / 3.0) * math.pi * radius**3
    mass = 0.0027  # 2.7 grams

    result = await calculate_buoyancy(
        volume=volume,
        fluid_density=1000.0,
    )

    weight = mass * 9.81
    print(f"  Volume: {volume:.6f} m³")
    print(f"  Weight: {weight:.4f} N")
    print(f"  Buoyant force: {result['buoyant_force']:.2f} N")

    if weight < result["buoyant_force"]:
        print("  Result: ⬆️  FLOATS (buoyancy > weight)")
    else:
        print("  Result: ⬇️  SINKS (weight > buoyancy)")


async def demo_terminal_velocity():
    """Demo: Calculate terminal velocity for different scenarios."""
    print("\n" + "=" * 70)
    print("DEMO 3: Terminal Velocity")
    print("=" * 70)

    # Scenario 1: Skydiver
    print("\nScenario 1: Skydiver (belly-down position)")
    print("  - Mass: 70 kg")
    print("  - Cross-sectional area: 0.7 m²")
    print("  - Drag coefficient: 1.0")
    print("  - Fluid: Air (1.225 kg/m³)")

    result = await calculate_terminal_velocity(
        mass=70.0,
        cross_sectional_area=0.7,
        fluid_density=1.225,  # air
        drag_coefficient=1.0,
    )

    print(
        f"\n  Terminal velocity: {result['terminal_velocity']:.1f} m/s ({result['terminal_velocity'] * 2.237:.0f} mph)"
    )
    print(f"  Time to 95%: {result['time_to_95_percent']:.1f} seconds")
    print(f"  Drag force at terminal: {result['drag_force_at_terminal']:.0f} N (equals weight)")

    # Scenario 2: Raindrop
    print("\nScenario 2: Raindrop (2mm diameter)")
    radius = 0.001  # 1mm
    mass = 0.0000042  # 4.2 mg
    area = math.pi * radius * radius

    result = await calculate_terminal_velocity(
        mass=mass,
        cross_sectional_area=area,
        fluid_density=1.225,
        drag_coefficient=0.47,  # sphere
    )

    print(f"  Terminal velocity: {result['terminal_velocity']:.1f} m/s")
    print(f"  Time to 95%: {result['time_to_95_percent']:.3f} seconds")


async def demo_underwater_motion():
    """Demo: Simulate underwater projectile with drag and buoyancy."""
    print("\n" + "=" * 70)
    print("DEMO 4: Underwater Torpedo Launch")
    print("=" * 70)

    # Torpedo: streamlined, heavy, fast
    print("\nScenario: Streamlined torpedo launched underwater")
    print("  - Initial velocity: 20 m/s forward")
    print("  - Mass: 100 kg")
    print("  - Volume: 0.05 m³")
    print("  - Cross-sectional area: 0.03 m²")
    print("  - Drag coefficient: 0.04 (streamlined)")
    print("  - Fluid: Water (1000 kg/m³)")

    result = await simulate_underwater_motion(
        initial_velocity=[20.0, 0.0, 0.0],  # 20 m/s forward
        mass=100.0,
        volume=0.05,
        cross_sectional_area=0.03,
        fluid_density=1000.0,
        fluid_viscosity=1.002e-3,  # water
        drag_coefficient=0.04,  # streamlined
        duration=30.0,
        dt=0.01,
    )

    print("\n  Simulation duration: 30 seconds")
    print(f"  Trajectory points: {len(result['trajectory'])}")
    print(
        f"  Final position: [{result['final_position'][0]:.1f}, {result['final_position'][1]:.1f}, {result['final_position'][2]:.1f}] m"
    )
    print(
        f"  Final velocity: [{result['final_velocity'][0]:.1f}, {result['final_velocity'][1]:.1f}, {result['final_velocity'][2]:.1f}] m/s"
    )
    print(f"  Total distance traveled: {result['total_distance']:.1f} m")
    print(f"  Max depth: {abs(result['max_depth']):.1f} m")
    print(f"  Settled: {'Yes' if result['settled'] else 'No'}")

    # Analyze deceleration
    initial_speed = 20.0
    final_speed = math.sqrt(
        result["final_velocity"][0] ** 2
        + result["final_velocity"][1] ** 2
        + result["final_velocity"][2] ** 2
    )
    speed_loss_percent = ((initial_speed - final_speed) / initial_speed) * 100

    print(f"\n  Speed loss due to drag: {speed_loss_percent:.1f}%")
    print(f"  Initial: {initial_speed:.1f} m/s → Final: {final_speed:.1f} m/s")


async def main():
    """Run all fluid dynamics demos."""
    print("=" * 70)
    print("FLUID DYNAMICS DEMONSTRATIONS")
    print("=" * 70)
    print("\nShowcasing new fluid dynamics capabilities:")
    print("  - Drag force calculations (quadratic drag)")
    print("  - Buoyancy (Archimedes' principle)")
    print("  - Terminal velocity (force balance)")
    print("  - Underwater motion simulation")

    await demo_drag_force()
    await demo_buoyancy()
    await demo_terminal_velocity()
    await demo_underwater_motion()

    print("\n" + "=" * 70)
    print("✨ FLUID DYNAMICS COMPLETE!")
    print("=" * 70)
    print("\nNew tools available:")
    print("  - calculate_drag_force()")
    print("  - calculate_buoyancy()")
    print("  - calculate_terminal_velocity()")
    print("  - simulate_underwater_motion()")
    print("\nThese analytical calculations complement the Rapier rigid-body")
    print("simulations for comprehensive physics modeling.")


if __name__ == "__main__":
    asyncio.run(main())
