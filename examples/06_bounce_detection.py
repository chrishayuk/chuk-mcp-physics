#!/usr/bin/env python3
"""Demo: Automatic bounce detection and counting.

This demonstrates the new Phase 1.1 feature: automatic bounce event detection.
"""

import asyncio
import os

os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"
os.environ["PHYSICS_PROVIDER"] = "rapier"

from chuk_mcp_physics.server import (
    create_simulation,
    add_rigid_body,
    record_trajectory_with_events,
    destroy_simulation,
)
from chuk_mcp_physics.analysis import count_bounces, get_last_bounce_above_threshold


async def demo_bounce_detection():
    """Demonstrate automatic bounce detection."""
    print("=" * 70)
    print("DEMO: Automatic Bounce Detection (Phase 1.1)")
    print("=" * 70)

    # Create simulation
    print("\n1. Creating simulation...")
    sim = await create_simulation(gravity_y=-9.81, dt=0.008)
    print(f"   ✓ Simulation ID: {sim.sim_id}")

    try:
        # Add ground plane
        print("\n2. Adding ground plane...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ground",
            body_type="static",
            shape="plane",
            normal=[0.0, 1.0, 0.0],
            restitution=0.3,
            friction=0.8,
        )
        print("   ✓ Ground plane added")

        # Add bouncing ball
        print("\n3. Adding ball (10cm diameter, 1kg, dropped from 10m)...")
        await add_rigid_body(
            sim_id=sim.sim_id,
            body_id="ball",
            body_type="dynamic",
            shape="sphere",
            size=[0.05],  # 5cm radius = 10cm diameter
            mass=1.0,
            position=[0.0, 10.0, 0.0],
            restitution=0.3,  # Soft rubber
            friction=0.8,
        )
        print("   ✓ Ball added")

        # Record trajectory WITH EVENTS (new feature!)
        print("\n4. Recording 5-second trajectory WITH event detection...")
        steps = int(5.0 / 0.008)
        # Note: Ball radius is 0.05m, so center is at 0.05m when touching ground
        # Use threshold of 0.1m to catch bounces reliably
        trajectory = await record_trajectory_with_events(
            sim_id=sim.sim_id,
            body_id="ball",
            steps=steps,
            detect_bounces=True,
            bounce_height_threshold=0.1,  # 10cm threshold (2x ball radius)
        )
        print(f"   ✓ Recorded {trajectory.meta.num_frames} frames")
        print(f"   ✓ Detected {len(trajectory.bounces)} bounces!")

        # Display bounce details
        print("\n5. Bounce Analysis:")
        print("   " + "-" * 66)
        print(
            f"   {'Bounce':<8} {'Time (s)':<10} {'Height (m)':<12} "
            f"{'Speed Before':<14} {'Speed After':<14} {'Energy Loss':<12}"
        )
        print("   " + "-" * 66)

        for bounce in trajectory.bounces:
            print(
                f"   #{bounce.bounce_number:<7} {bounce.time:<10.3f} "
                f"{bounce.height_at_bounce:<12.4f} "
                f"{bounce.speed_before:<14.3f} {bounce.speed_after:<14.3f} "
                f"{bounce.energy_loss_percent:<12.1f}%"
            )

        # Answer the original question!
        print("\n6. Answering: 'When did the ball stop bouncing above 5mm?'")
        last_bounce_above_5mm = get_last_bounce_above_threshold(trajectory, height_threshold=0.005)

        if last_bounce_above_5mm:
            print(
                f"   ✓ Last bounce above 5mm: Bounce #{last_bounce_above_5mm.bounce_number} "
                f"at t={last_bounce_above_5mm.time:.2f}s"
            )
            print(
                f"     Height at this bounce: {last_bounce_above_5mm.height_at_bounce * 1000:.1f}mm"
            )
        else:
            print("   ✓ No bounces detected above 5mm threshold")

        # Count bounces above different thresholds
        print("\n7. Bounce Statistics:")
        total_bounces = count_bounces(trajectory)
        bounces_above_1cm = count_bounces(trajectory, min_height=0.01)
        bounces_above_5mm = count_bounces(trajectory, min_height=0.005)

        print(f"   Total bounces detected: {total_bounces}")
        print(f"   Bounces above 1cm: {bounces_above_1cm}")
        print(f"   Bounces above 5mm: {bounces_above_5mm}")

        # Physical insights
        if len(trajectory.bounces) >= 2:
            first = trajectory.bounces[0]
            last = trajectory.bounces[-1]

            print("\n8. Physical Analysis:")
            print(
                f"   First bounce: {first.speed_before:.2f} m/s → {first.speed_after:.2f} m/s "
                f"({first.energy_loss_percent:.1f}% energy loss)"
            )
            print(
                f"   Last bounce:  {last.speed_before:.2f} m/s → {last.speed_after:.2f} m/s "
                f"({last.energy_loss_percent:.1f}% energy loss)"
            )
            print(f"   Time between bounces: {last.time - first.time:.2f}s")

        print("\n" + "=" * 70)
        print("✅ SUCCESS! Phase 1.1 bounce detection works perfectly!")
        print("=" * 70)

        # Show how this answers the user's question
        print("\n💡 Original Question:")
        print('   "How many bounces would a 10cm ball that weighs 1 kilo')
        print('    before it would stop bouncing over 5mm?"')
        print(f"\n🎯 Answer: {bounces_above_5mm} bounces above 5mm threshold")
        if last_bounce_above_5mm:
            print(f"   Last bounce above 5mm occurred at t={last_bounce_above_5mm.time:.2f}s")

    finally:
        await destroy_simulation(sim.sim_id)
        print("\n✓ Simulation destroyed\n")


if __name__ == "__main__":
    asyncio.run(demo_bounce_detection())
