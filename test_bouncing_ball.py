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
from chuk_mcp_physics.server import (
    create_simulation,
    add_rigid_body,
    record_trajectory,
    destroy_simulation,
)

# Use public Rapier service
os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"


async def test_bouncing_ball():
    """Simulate a bouncing ball and count bounces."""
    print("Creating simulation...")
    sim = await create_simulation(gravity_y=-9.81, dt=0.016)
    print(f"✓ Simulation created: {sim.sim_id}")

    try:
        # Add ground plane
        print("\nAdding ground plane...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ground",
            body_type="static",
            shape="plane",
            normal=[0.0, 1.0, 0.0],
            offset=0.0,
        )
        print("✓ Ground plane added")

        # Add ball (10cm diameter = 0.05m radius, 1kg, dropped from 10m)
        print("\nAdding ball (0.1m diameter, 1kg, dropped from 10m)...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ball",
            body_type="dynamic",
            shape="sphere",
            radius=0.05,  # 5cm radius = 10cm diameter
            mass=1.0,
            position=[0.0, 10.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            restitution=0.6,  # Medicine ball-like bounciness
            friction=0.5,
        )
        print("✓ Ball added")

        # Record trajectory for 15 seconds (should be plenty for all bounces)
        print("\nRecording trajectory for 15 seconds...")
        steps = int(15.0 / 0.016)  # 15 seconds at 60 FPS
        trajectory = await record_trajectory(
            sim_id=sim.sim_id, body_id="ball", steps=steps
        )
        print(f"✓ Recorded {trajectory.meta.num_frames} frames")

        # Analyze bounces
        print("\nAnalyzing bounces...")
        bounce_count = 0
        max_heights = []
        last_direction = 0  # -1 = down, 1 = up
        last_height = trajectory.frames[0].position[1]

        for i, frame in enumerate(trajectory.frames):
            height = frame.position[1]
            velocity_y = frame.velocity[1] if frame.velocity else 0.0

            # Detect direction change (bounce)
            current_direction = 1 if velocity_y > 0 else -1

            # Bounce detected: was going down, now going up, and close to ground
            if last_direction < 0 and current_direction > 0 and height < 0.1:
                bounce_count += 1
                # Find peak after this bounce
                peak_height = height
                for j in range(i, min(i + 100, len(trajectory.frames))):
                    if trajectory.frames[j].position[1] > peak_height:
                        peak_height = trajectory.frames[j].position[1]
                max_heights.append(peak_height)

                print(f"  Bounce #{bounce_count}: peak height = {peak_height:.4f}m")

                # Stop if peak height is below 5mm
                if peak_height < 0.005:
                    print(
                        f"\n✓ Ball stopped bouncing (peak < 5mm) after {bounce_count} bounces"
                    )
                    break

            last_direction = current_direction
            last_height = height

        print(f"\n{'='*60}")
        print(f"RESULTS:")
        print(f"{'='*60}")
        print(f"Initial drop height: 10.0 m")
        print(f"Ball properties: 10cm diameter, 1kg, restitution=0.6")
        print(f"Stop threshold: 5mm (0.005m)")
        print(f"Total bounces before stopping: {bounce_count}")
        print(f"\nBounce heights:")
        for i, h in enumerate(max_heights[:10], 1):  # Show first 10
            print(f"  Bounce {i}: {h*1000:.1f}mm")
        if len(max_heights) > 10:
            print(f"  ... ({len(max_heights) - 10} more)")

    finally:
        # Cleanup
        print(f"\nCleaning up simulation {sim.sim_id}...")
        await destroy_simulation(sim.sim_id)
        print("✓ Simulation destroyed")


if __name__ == "__main__":
    asyncio.run(test_bouncing_ball())
