#!/usr/bin/env python3
"""Quick Start Example - Physics MCP Server

This is the simplest possible example to get started with the physics MCP server.
It demonstrates all 5 analytic calculation tools.
"""

import asyncio
from chuk_mcp_physics.providers.analytic import AnalyticProvider
from chuk_mcp_physics.models import (
    ProjectileMotionRequest,
    CollisionCheckRequest,
    ForceCalculationRequest,
    KineticEnergyRequest,
    MomentumRequest,
)


async def main():
    """Quick demonstration of all analytic tools."""

    # Create the provider (no setup required!)
    provider = AnalyticProvider()

    print("🚀 Physics MCP Server - Quick Start")
    print("=" * 60)
    print()

    # 1. Projectile Motion
    print("1️⃣  PROJECTILE MOTION")
    print("Question: How far does a ball go if thrown at 20 m/s at 45°?")

    request = ProjectileMotionRequest(initial_velocity=20.0, angle_degrees=45.0)
    result = await provider.calculate_projectile_motion(request)

    print(f"Answer: {result.range:.1f} meters")
    print(f"  Max height: {result.max_height:.1f}m, Flight time: {result.time_of_flight:.1f}s")
    print()

    # 2. Collision Check
    print("2️⃣  COLLISION DETECTION")
    print("Question: Will two cars collide?")
    print("  Car A: position (0,0,0), velocity (20,0,0) m/s")
    print("  Car B: position (50,0,0), velocity (-15,0,0) m/s")

    request = CollisionCheckRequest(
        body1_position=[0, 0, 0],
        body1_velocity=[20, 0, 0],
        body1_radius=2.0,
        body2_position=[50, 0, 0],
        body2_velocity=[-15, 0, 0],
        body2_radius=2.0,
    )
    result = await provider.check_collision(request)

    if result.will_collide:
        print(f"Answer: YES! Collision in {result.collision_time:.1f} seconds")
        print(f"  Impact speed: {result.impact_speed:.1f} m/s")
    else:
        print("Answer: No collision")
    print()

    # 3. Force Calculation
    print("3️⃣  FORCE CALCULATION (F = ma)")
    print("Question: What force accelerates a 1500kg car at 3 m/s²?")

    request = ForceCalculationRequest(mass=1500.0, acceleration=[3.0, 0.0, 0.0])
    result = await provider.calculate_force(request)

    print(f"Answer: {result.magnitude:.0f} Newtons")
    print()

    # 4. Kinetic Energy
    print("4️⃣  KINETIC ENERGY (KE = ½mv²)")
    print("Question: How much energy does a 1500kg car have at 30 m/s?")

    request = KineticEnergyRequest(mass=1500.0, velocity=[30.0, 0.0, 0.0])
    result = await provider.calculate_kinetic_energy(request)

    print(f"Answer: {result.kinetic_energy:,.0f} Joules")
    print(f"  Speed: {result.speed} m/s ({result.speed * 2.237:.0f} mph)")
    print()

    # 5. Momentum
    print("5️⃣  MOMENTUM (p = mv)")
    print("Question: What's the momentum of a 70kg person running at 5 m/s?")

    request = MomentumRequest(mass=70.0, velocity=[5.0, 0.0, 0.0])
    result = await provider.calculate_momentum(request)

    print(f"Answer: {result.magnitude:.0f} kg·m/s")
    print()

    print("=" * 60)
    print("✅ All 5 analytic calculations complete!")
    print()
    print("Next steps:")
    print("  • Run examples/01_simple_projectile.py for detailed trajectories")
    print("  • Run examples/02_collision_detection.py for crash scenarios")
    print("  • Run examples/03_force_energy_momentum.py for physics analysis")
    print("  • Run examples/04_r3f_visualization.py for 3D animation data")
    print()
    print("For simulations (Rapier):")
    print("  • See RAPIER_SERVICE.md for Rust service setup")
    print("  • Use create_simulation, add_body, step_simulation tools")
    print()


if __name__ == "__main__":
    asyncio.run(main())
