#!/usr/bin/env python3
"""Test bouncing ball simulation to answer: How many bounces before stopping at 5mm?

Given:
- Ball: 10cm diameter (0.1m), 1kg mass
- Drop height: 10 meters
- Stop threshold: 5mm (0.005m)
- Coefficient of restitution: assume 0.6 (typical for a medicine ball)
"""

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


async def test_bouncing_ball():
    """Simulate a bouncing ball and count bounces."""
    print("Creating simulation...")
    # Use smaller dt for better accuracy and prevent tunneling
    sim = await create_simulation(gravity_y=-9.81, dt=0.008)  # 125 FPS for accuracy
    print(f"✓ Simulation created: {sim.sim_id}")

    try:
        # Add ground (using a thicker box to prevent tunneling)
        print("\nAdding ground (large box)...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ground",
            body_type="static",
            shape="box",
            size=[100.0, 1.0, 100.0],  # Very large, thick box
            position=[0.0, -0.5, 0.0],  # Positioned with top at y=0
            restitution=0.3,  # Lower restitution for more energy loss
            friction=0.8,  # Higher friction
        )
        print("✓ Ground added")

        # Add ball (10cm diameter = 0.05m radius, 1kg, dropped from 10m)
        print("\nAdding ball (0.1m diameter, 1kg, dropped from 10m)...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ball",
            body_type="dynamic",
            shape="sphere",
            size=[0.05],  # Rapier expects size even for spheres: [radius]
            mass=1.0,
            position=[0.0, 10.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            restitution=0.3,  # Lower restitution = less bouncy, more realistic for medicine ball
            friction=0.8,  # Higher friction for more energy dissipation
        )
        print("✓ Ball added")

        # Record trajectory for 60 seconds (longer to see bounces decay)
        print("\nRecording trajectory for 60 seconds...")
        steps = int(60.0 / 0.008)  # 60 seconds at 125 FPS
        trajectory = await record_trajectory(sim_id=sim.sim_id, body_id="ball", steps=steps)
        print(f"✓ Recorded {trajectory.meta.num_frames} frames")

        # Check final state
        final_height = trajectory.frames[-1].position[1]
        print(f"Final ball height after 60s: {final_height * 1000:.1f}mm")

        # Analyze bounces
        print("\nAnalyzing bounces...")
        bounce_count = 0
        max_heights = []
        last_bounce_frame = -100  # Track last bounce to avoid double-counting
        last_bounce_above_threshold = 0  # Track last bounce >= 5mm

        # Detect bounces by finding local minima (valleys) in height
        for i in range(1, len(trajectory.frames) - 1):
            height = trajectory.frames[i].position[1]
            prev_height = trajectory.frames[i - 1].position[1]
            next_height = trajectory.frames[i + 1].position[1]

            # Detect bounce: local minimum in height (valley)
            # Ball was falling, hit ground (local minimum), then rising
            # Must be at least 10 frames since last bounce to avoid duplicates
            if (
                prev_height > height
                and next_height > height
                and height < 0.1
                and i - last_bounce_frame > 10
            ):
                # Find peak after this bounce
                peak_height = height
                for j in range(i, min(i + 300, len(trajectory.frames))):
                    if trajectory.frames[j].position[1] > peak_height:
                        peak_height = trajectory.frames[j].position[1]
                    elif trajectory.frames[j].position[1] < peak_height - 0.005:
                        # Started falling again, we found the peak
                        break

                # Count all bounces above 0.5mm (to capture very tiny bounces)
                if peak_height > 0.0005:
                    bounce_count += 1
                    max_heights.append(peak_height)
                    last_bounce_frame = i

                    # Check if this is the last bounce above 5mm
                    if peak_height >= 0.005:
                        print(
                            f"  Bounce #{bounce_count}: peak height = {peak_height:.4f}m ({peak_height * 1000:.1f}mm)"
                        )
                        last_bounce_above_threshold = bounce_count
                    else:
                        print(
                            f"  Bounce #{bounce_count}: peak height = {peak_height:.4f}m ({peak_height * 1000:.1f}mm) - BELOW 5mm THRESHOLD"
                        )
                        # Found first bounce below 5mm, we're done
                        print(
                            f"\n✓ Last bounce above 5mm was bounce #{last_bounce_above_threshold}"
                        )
                        break

        print(f"\n{'=' * 60}")
        print("RESULTS:")
        print(f"{'=' * 60}")
        print("Initial drop height: 10.0 m")
        print("Ball properties: 10cm diameter, 1kg, restitution=0.3 (medicine ball)")
        print("Stop threshold: 5mm (0.005m)")
        print(f"Total bounces detected: {bounce_count}")
        print(f"\n🎯 ANSWER: The ball makes {last_bounce_above_threshold} bounces before")
        print("   it stops bouncing above 5mm (0.005m)")
        print("\nAll bounce heights:")
        for i, h in enumerate(max_heights, 1):
            marker = " ✓" if h >= 0.005 else " ✗ (below 5mm)"
            print(f"  Bounce {i}: {h * 1000:.1f}mm{marker}")

    finally:
        # Cleanup
        print(f"\nCleaning up simulation {sim.sim_id}...")
        await destroy_simulation(sim.sim_id)
        print("✓ Simulation destroyed")


if __name__ == "__main__":
    asyncio.run(test_bouncing_ball())
