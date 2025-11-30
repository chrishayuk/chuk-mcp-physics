#!/usr/bin/env python3
"""Demo script for Phase 1.2: Contact Events from Rapier.

This demonstrates that contact events from the Rapier physics engine
are being correctly tracked and reported alongside bounce detection.
"""

# IMPORTANT: Set environment variable BEFORE any imports
import os

if "RAPIER_SERVICE_URL" not in os.environ:
    os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"

import asyncio

from chuk_mcp_physics.providers.rapier import RapierProvider
from chuk_mcp_physics.models import (
    SimulationConfig,
    RigidBodyDefinition,
    BodyType,
    ShapeType,
)


async def main():
    print("🦀 Phase 1.2 Demo: Contact Events from Rapier\n")
    print("=" * 60)

    # Get the configured service URL
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

    # Add ground plane
    print("2️⃣  Adding ground plane...")
    await provider.add_body(
        sim.sim_id,
        RigidBodyDefinition(
            id="ground",
            kind=BodyType.STATIC,
            shape=ShapeType.PLANE,
            normal=[0.0, 1.0, 0.0],
            offset=0.0,
            friction=0.5,
            restitution=0.3,  # 30% energy retention per bounce
        ),
    )
    print("   ✓ Ground plane added\n")

    # Add bouncing ball
    print("3️⃣  Adding bouncing ball...")
    ball_radius = 0.05  # 5cm radius (10cm diameter)
    await provider.add_body(
        sim.sim_id,
        RigidBodyDefinition(
            id="ball",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.SPHERE,
            size=[ball_radius],
            mass=1.0,  # 1kg
            position=[0.0, 10.0, 0.0],  # Drop from 10m
            velocity=[0.0, 0.0, 0.0],
            friction=0.5,
            restitution=0.3,  # Match ground restitution
        ),
    )
    print("   ✓ Ball added (1kg, 10cm diameter)\n")

    # Record trajectory with contact events
    print("4️⃣  Recording trajectory (10 seconds, 625 frames)...")
    steps = 625  # 10 seconds at 0.016s per step
    trajectory = await provider.record_trajectory(
        sim_id=sim.sim_id,
        body_id="ball",
        steps=steps,
    )
    print(f"   ✓ Recorded {len(trajectory.frames)} frames\n")

    # Analyze contact events
    print("5️⃣  Analyzing contact events from Rapier...\n")
    print("=" * 60)

    if not trajectory.contact_events:
        print("⚠️  No contact events detected!")
        print("   This might mean:")
        print("   - Rapier service doesn't have Phase 1.2 code")
        print("   - Contact detection isn't working")
        print("   - Ball hasn't hit ground yet")
    else:
        print(f"✅ Detected {len(trajectory.contact_events)} contact events!\n")

        # Group events by type
        started_events = [e for e in trajectory.contact_events if e.event_type == "started"]
        ended_events = [e for e in trajectory.contact_events if e.event_type == "ended"]
        ongoing_events = [e for e in trajectory.contact_events if e.event_type == "ongoing"]

        print(f"   Contact starts: {len(started_events)}")
        print(f"   Contact ends:   {len(ended_events)}")
        print(f"   Ongoing:        {len(ongoing_events)}\n")

        # Show first 10 contact events
        print("First contact events:")
        print("-" * 60)
        for i, event in enumerate(trajectory.contact_events[:10]):
            print(f"{i + 1}. t={event.time:.3f}s - {event.event_type.upper()}")
            print(f"   Bodies: {event.body_a} ↔ {event.body_b}")
            print(
                f"   Contact point: [{event.contact_point[0]:.3f}, {event.contact_point[1]:.3f}, {event.contact_point[2]:.3f}]"
            )
            print(
                f"   Normal: [{event.normal[0]:.3f}, {event.normal[1]:.3f}, {event.normal[2]:.3f}]"
            )
            print(f"   Impulse magnitude: {event.impulse_magnitude:.3f}")
            print(
                f"   Relative velocity: [{event.relative_velocity[0]:.3f}, {event.relative_velocity[1]:.3f}, {event.relative_velocity[2]:.3f}]"
            )
            print()

        if len(trajectory.contact_events) > 10:
            print(f"... and {len(trajectory.contact_events) - 10} more events\n")

        # Analyze bounces from contact starts
        print("=" * 60)
        print("6️⃣  Bounce analysis from contact events:\n")

        bounce_count = 0
        for event in started_events:
            if abs(event.impulse_magnitude) > 0.1:  # Filter noise
                bounce_count += 1
                height = event.contact_point[1]
                print(f"Bounce #{bounce_count} at t={event.time:.3f}s")
                print(f"   Height: {height:.4f}m")
                print(f"   Impulse: {event.impulse_magnitude:.3f}")
                print(f"   Rel. velocity: {event.relative_velocity[1]:.3f} m/s")
                print()

    # Cleanup
    print("=" * 60)
    print("7️⃣  Cleaning up...")
    await provider.destroy_simulation(sim.sim_id)
    print("   ✓ Simulation destroyed\n")

    print("=" * 60)
    print("✨ Phase 1.2 Demo Complete!\n")
    print("Contact events are being tracked from Rapier physics engine.")
    print("This provides ground-truth collision data for validation.")


if __name__ == "__main__":
    asyncio.run(main())
