#!/usr/bin/env python3
"""Test bouncing ball with plane ground shape."""

import asyncio
import os
import logging

# Enable info logging only
logging.basicConfig(level=logging.WARNING)

# IMPORTANT: Set environment variables BEFORE importing physics modules
os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"
os.environ["PHYSICS_PROVIDER"] = "rapier"  # Use Rapier provider for simulations

from chuk_mcp_physics.server import (  # noqa: E402
    create_simulation,
    add_rigid_body,
    record_trajectory,
    destroy_simulation,
)


async def test_plane_ground():
    """Test bouncing ball with plane ground."""
    print("Creating simulation...")
    sim = await create_simulation(gravity_y=-9.81, dt=0.008)
    print(f"✓ Simulation created: {sim.sim_id}")

    try:
        # Add ground PLANE (testing new feature!)
        print("\nAdding ground plane...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ground",
            body_type="static",
            shape="plane",
            normal=[0.0, 1.0, 0.0],  # Upward facing
            offset=0.0,  # At y=0
            restitution=0.3,
            friction=0.8,
        )
        print("✓ Ground plane added!")

        # Add ball
        print("\nAdding ball...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ball",
            body_type="dynamic",
            shape="sphere",
            size=[0.05],  # 5cm radius
            mass=1.0,
            position=[0.0, 10.0, 0.0],
            restitution=0.3,
            friction=0.8,
        )
        print("✓ Ball added")

        # Record short trajectory
        print("\nRecording trajectory for 5 seconds...")
        steps = int(5.0 / 0.008)
        trajectory = await record_trajectory(sim_id=sim.sim_id, body_id="ball", steps=steps)
        print(f"✓ Recorded {trajectory.meta.num_frames} frames")

        # Find bounces
        bounce_count = 0
        for i in range(1, len(trajectory.frames) - 1):
            height = trajectory.frames[i].position[1]
            prev_height = trajectory.frames[i - 1].position[1]
            next_height = trajectory.frames[i + 1].position[1]

            if prev_height > height and next_height > height and height < 0.1:
                bounce_count += 1
                if bounce_count <= 3:
                    print(f"  Bounce #{bounce_count} at t={trajectory.frames[i].time:.2f}s")

        print("\n✅ SUCCESS! Plane ground shape works!")
        print(f"   Detected {bounce_count} bounces using plane shape")

    finally:
        await destroy_simulation(sim.sim_id)
        print("\n✓ Simulation destroyed")


if __name__ == "__main__":
    asyncio.run(test_plane_ground())
