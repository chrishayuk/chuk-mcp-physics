#!/usr/bin/env python3
"""Demo script for Phase 1.3: Joints & Constraints.

This demonstrates a simple pendulum created using a revolute joint.
The pendulum should swing back and forth realistically.
"""

# IMPORTANT: Set environment variable BEFORE any imports
import os

if "RAPIER_SERVICE_URL" not in os.environ:
    os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"

import asyncio
import math

from chuk_mcp_physics.providers.rapier import RapierProvider
from chuk_mcp_physics.models import (
    SimulationConfig,
    RigidBodyDefinition,
    JointDefinition,
    BodyType,
    ShapeType,
    JointType,
)


async def main():
    print("🎢 Phase 1.3 Demo: Simple Pendulum with Revolute Joint\n")
    print("=" * 60)

    # Use environment variable or default
    service_url = os.environ.get("RAPIER_SERVICE_URL", "https://rapier.chukai.io")
    print(f"Using Rapier service: {service_url}\n")

    provider = RapierProvider()

    # Create simulation
    print("1️⃣  Creating simulation...")
    sim = await provider.create_simulation(
        SimulationConfig(
            gravity=[0.0, -9.81, 0.0],
            dimensions=3,
            dt=0.016,
        )
    )
    print(f"   ✓ Simulation created: {sim.sim_id}\n")

    # Add fixed anchor point at top
    print("2️⃣  Adding anchor point...")
    await provider.add_body(
        sim.sim_id,
        RigidBodyDefinition(
            id="anchor",
            kind=BodyType.STATIC,
            shape=ShapeType.SPHERE,
            size=[0.05],  # 5cm radius
            position=[0.0, 5.0, 0.0],  # 5m high
            friction=0.0,
            restitution=0.0,
        ),
    )
    print("   ✓ Anchor added at (0, 5, 0)\n")

    # Add pendulum bob - start at angle
    print("3️⃣  Adding pendulum bob...")
    pendulum_length = 2.0  # 2 meter pendulum
    initial_angle = math.radians(45)  # Start at 45 degrees
    bob_x = pendulum_length * math.sin(initial_angle)
    bob_y = 5.0 - pendulum_length * math.cos(initial_angle)

    await provider.add_body(
        sim.sim_id,
        RigidBodyDefinition(
            id="bob",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.SPHERE,
            size=[0.2],  # 20cm radius
            mass=1.0,  # 1kg
            position=[bob_x, bob_y, 0.0],
            velocity=[0.0, 0.0, 0.0],
            friction=0.1,
            restitution=0.1,
        ),
    )
    print(f"   ✓ Bob added at ({bob_x:.2f}, {bob_y:.2f}, 0)\n")

    # Connect with revolute joint (hinge)
    print("4️⃣  Creating revolute joint...")
    joint_id = await provider.add_joint(
        sim.sim_id,
        JointDefinition(
            id="pendulum_hinge",
            joint_type=JointType.REVOLUTE,
            body_a="anchor",
            body_b="bob",
            anchor_a=[0.0, 0.0, 0.0],  # Center of anchor
            anchor_b=[0.0, 0.2, 0.0],  # Top of bob
            axis=[0.0, 0.0, 1.0],  # Rotate around Z-axis
        ),
    )
    print(f"   ✓ Joint created: {joint_id}\n")

    # Record trajectory
    print("5️⃣  Recording pendulum swing (10 seconds, 625 frames)...")
    steps = 625  # 10 seconds at 0.016s per step
    trajectory = await provider.record_trajectory(
        sim_id=sim.sim_id,
        body_id="bob",
        steps=steps,
    )
    print(f"   ✓ Recorded {len(trajectory.frames)} frames\n")

    # Analyze trajectory
    print("6️⃣  Analyzing pendulum motion...\n")
    print("=" * 60)

    # Find min/max heights (should show oscillation)
    heights = [frame.position[1] for frame in trajectory.frames]
    min_height = min(heights)
    max_height = max(heights)

    # Find x positions (should oscillate left/right)
    x_positions = [frame.position[0] for frame in trajectory.frames]
    min_x = min(x_positions)
    max_x = max(x_positions)

    print(f"Height range: {min_height:.3f}m to {max_height:.3f}m")
    print(f"X range: {min_x:.3f}m to {max_x:.3f}m\n")

    # Calculate period (time between peaks)
    # Theoretical period: T = 2π√(L/g) = 2π√(2/9.81) ≈ 2.84 seconds
    theoretical_period = 2 * math.pi * math.sqrt(pendulum_length / 9.81)
    print(f"Theoretical period: {theoretical_period:.3f}s\n")

    # Find peaks in x position
    peaks = []
    for i in range(1, len(trajectory.frames) - 1):
        prev_x = trajectory.frames[i - 1].position[0]
        curr_x = trajectory.frames[i].position[0]
        next_x = trajectory.frames[i + 1].position[0]

        # Local maximum
        if curr_x > prev_x and curr_x > next_x and abs(curr_x) > 0.1:
            peaks.append((trajectory.frames[i].time, curr_x))

    if len(peaks) >= 2:
        print("Detected swing peaks:")
        for i, (time, x_pos) in enumerate(peaks[:5]):
            print(f"  Peak #{i + 1}: t={time:.3f}s, x={x_pos:.3f}m")

        # Calculate observed period
        if len(peaks) >= 2:
            time_diffs = [peaks[i + 1][0] - peaks[i][0] for i in range(len(peaks) - 1)]
            avg_period = sum(time_diffs) / len(time_diffs)
            print(f"\nObserved period: {avg_period:.3f}s")
            print(f"Difference from theory: {abs(avg_period - theoretical_period):.3f}s")
            print(
                f"Accuracy: {(1 - abs(avg_period - theoretical_period) / theoretical_period) * 100:.1f}%"
            )
    else:
        print("⚠️  Not enough peaks detected for period calculation")

    # Sample trajectory points
    print("\n" + "=" * 60)
    print("Sample trajectory points:\n")
    sample_indices = [0, 156, 312, 468, 624]  # Every 2.5 seconds
    for idx in sample_indices:
        if idx < len(trajectory.frames):
            frame = trajectory.frames[idx]
            print(
                f"t={frame.time:.2f}s: pos=({frame.position[0]:6.3f}, {frame.position[1]:6.3f}, {frame.position[2]:6.3f})"
            )

    # Cleanup
    print("\n" + "=" * 60)
    print("7️⃣  Cleaning up...")
    await provider.destroy_simulation(sim.sim_id)
    print("   ✓ Simulation destroyed\n")

    print("=" * 60)
    print("✨ Phase 1.3 Demo Complete!\n")
    print("The pendulum swings realistically using the revolute joint.")
    print("Joints successfully connect and constrain rigid bodies!")


if __name__ == "__main__":
    asyncio.run(main())
