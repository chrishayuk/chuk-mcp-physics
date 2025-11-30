"""Example 5: Rapier Physics Simulation (Requires Rapier Service)

This example demonstrates the Rapier simulation tools for rigid-body physics.

REQUIREMENTS:
    This example uses the public Rapier service at https://rapier.chukai.io

    To use a local service instead:
        export RAPIER_SERVICE_URL=http://localhost:9000

    The analytic examples (00-04) work without any external services.

WHAT THIS DEMONSTRATES:
    - Creating a physics simulation world
    - Adding rigid bodies (boxes, spheres, capsules)
    - Stepping the simulation forward in time
    - Recording trajectories of objects
    - Collision detection through simulation
    - Gravity and realistic physics
"""

# IMPORTANT: Set environment variable BEFORE any imports
import os

if "RAPIER_SERVICE_URL" not in os.environ:
    os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"

import asyncio
from chuk_mcp_physics.providers.factory import get_provider
from chuk_mcp_physics.models import (
    SimulationConfig,
    RigidBodyDefinition,
    BodyType,
    ShapeType,
)


async def example_bouncing_ball():
    """Simulate a ball bouncing on the ground."""
    print("\n" + "=" * 70)
    print("EXAMPLE 1: Bouncing Ball")
    print("=" * 70)

    # Get Rapier provider (will try to connect to service)
    provider = get_provider("rapier")

    try:
        # 1. Create simulation world with gravity
        print("\n1️⃣  Creating simulation world...")
        config = SimulationConfig(
            gravity=[0, -9.81, 0],  # Earth gravity pointing down
            dimensions=3,
            dt=0.016,  # 60 FPS timestep
        )
        sim = await provider.create_simulation(config)
        print(f"   ✓ Created simulation: {sim.sim_id}")

        # 2. Add ground plane (static body)
        print("\n2️⃣  Adding ground plane...")
        ground = RigidBodyDefinition(
            id="ground",
            kind=BodyType.STATIC,
            shape=ShapeType.BOX,
            size=[100.0, 0.1, 100.0],  # Wide, thin platform
            position=[0.0, 0.0, 0.0],
        )
        await provider.add_body(sim.sim_id, ground)
        print("   ✓ Ground added at y=0")

        # 3. Add bouncing ball above ground
        print("\n3️⃣  Adding ball 10m above ground...")
        ball = RigidBodyDefinition(
            id="ball",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.SPHERE,
            size=[0.5],  # 0.5m radius
            mass=1.0,
            position=[0.0, 10.0, 0.0],
            restitution=0.8,  # 80% bounce
        )
        await provider.add_body(sim.sim_id, ball)
        print("   ✓ Ball added (mass=1kg, radius=0.5m, bounce=0.8)")

        # 4. Record trajectory for 3 seconds (180 frames at 60fps)
        print("\n4️⃣  Simulating 3 seconds of bouncing...")
        trajectory = await provider.record_trajectory(
            sim.sim_id,
            "ball",
            steps=180,
        )
        print(
            f"   ✓ Recorded {trajectory.meta.num_frames} frames over {trajectory.meta.total_time:.2f}s"
        )

        # 5. Analyze bounces
        print("\n5️⃣  Bounce Analysis:")
        bounces = []
        for i in range(1, len(trajectory.frames)):
            prev_frame = trajectory.frames[i - 1]
            curr_frame = trajectory.frames[i]

            # Detect bounce: velocity changes from down to up
            if prev_frame.velocity[1] < 0 and curr_frame.velocity[1] > 0:
                bounces.append(
                    {
                        "time": curr_frame.time,
                        "height": curr_frame.position[1],
                        "velocity": curr_frame.velocity[1],
                    }
                )

        print(f"   Detected {len(bounces)} bounces:")
        for i, bounce in enumerate(bounces[:5], 1):  # Show first 5
            print(
                f"   Bounce {i}: t={bounce['time']:.2f}s, h={bounce['height']:.2f}m, v={bounce['velocity']:.1f}m/s"
            )

        # 6. Cleanup
        await provider.destroy_simulation(sim.sim_id)
        print("\n   ✓ Simulation cleaned up")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        service_url = os.environ.get("RAPIER_SERVICE_URL", "https://rapier.chukai.io")
        print(f"\nCannot connect to Rapier service at {service_url}")
        print("Using public service: https://rapier.chukai.io (default)")
        print("For local service: export RAPIER_SERVICE_URL=http://localhost:9000")


async def example_collision_simulation():
    """Simulate two objects colliding."""
    print("\n" + "=" * 70)
    print("EXAMPLE 2: Head-On Collision")
    print("=" * 70)

    provider = get_provider("rapier")

    try:
        # Create simulation
        print("\n1️⃣  Creating simulation world...")
        config = SimulationConfig(
            gravity=[0, 0, 0],  # No gravity (space collision)
            dimensions=3,
            dt=0.01,  # 100 FPS for precise collision detection
        )
        sim = await provider.create_simulation(config)
        print(f"   ✓ Created simulation: {sim.sim_id}")

        # Add two boxes moving toward each other
        print("\n2️⃣  Adding two boxes on collision course...")

        box1 = RigidBodyDefinition(
            id="box1",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.BOX,
            size=[1.0, 1.0, 1.0],
            mass=5.0,
            position=[-10.0, 0.0, 0.0],
            velocity=[5.0, 0.0, 0.0],  # Moving right at 5 m/s
            restitution=0.5,  # 50% elastic collision
        )
        await provider.add_body(sim.sim_id, box1)
        print("   ✓ Box 1: mass=5kg, position=(-10, 0, 0), velocity=(5, 0, 0)")

        box2 = RigidBodyDefinition(
            id="box2",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.BOX,
            size=[1.0, 1.0, 1.0],
            mass=3.0,
            position=[10.0, 0.0, 0.0],
            velocity=[-5.0, 0.0, 0.0],  # Moving left at 5 m/s
            restitution=0.5,
        )
        await provider.add_body(sim.sim_id, box2)
        print("   ✓ Box 2: mass=3kg, position=(10, 0, 0), velocity=(-5, 0, 0)")

        # Simulate collision
        print("\n3️⃣  Simulating 4 seconds...")
        print("\n   Time  | Box1 Position | Box1 Velocity | Box2 Position | Box2 Velocity")
        print("   " + "-" * 75)

        for step in range(0, 400, 40):  # Show every 0.4 seconds
            state = await provider.step_simulation(sim.sim_id, steps=40)

            box1_state = next(b for b in state.bodies if b.id == "box1")
            box2_state = next(b for b in state.bodies if b.id == "box2")

            print(
                f"   {state.time:4.1f}s | "
                f"{box1_state.position[0]:6.1f}        | "
                f"{box1_state.velocity[0]:6.1f}        | "
                f"{box2_state.position[0]:6.1f}        | "
                f"{box2_state.velocity[0]:6.1f}"
            )

        await provider.destroy_simulation(sim.sim_id)
        print("\n   ✓ Simulation cleaned up")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        service_url = os.environ.get("RAPIER_SERVICE_URL", "https://rapier.chukai.io")
        print(f"\nCannot connect to Rapier service at {service_url}")
        print("Using public service: https://rapier.chukai.io (default)")
        print("For local service: export RAPIER_SERVICE_URL=http://localhost:9000")


async def example_stacking_boxes():
    """Simulate a tower of boxes under gravity."""
    print("\n" + "=" * 70)
    print("EXAMPLE 3: Stacking Boxes (Tower Stability)")
    print("=" * 70)

    provider = get_provider("rapier")

    try:
        print("\n1️⃣  Creating simulation world...")
        config = SimulationConfig(
            gravity=[0, -9.81, 0],
            dimensions=3,
            dt=0.016,
        )
        sim = await provider.create_simulation(config)
        print(f"   ✓ Created simulation: {sim.sim_id}")

        # Add ground
        print("\n2️⃣  Adding ground...")
        ground = RigidBodyDefinition(
            id="ground",
            kind=BodyType.STATIC,
            shape=ShapeType.BOX,
            size=[20.0, 0.1, 20.0],
            position=[0.0, 0.0, 0.0],
        )
        await provider.add_body(sim.sim_id, ground)
        print("   ✓ Ground added")

        # Stack 5 boxes
        print("\n3️⃣  Stacking 5 boxes vertically...")
        box_height = 1.0
        for i in range(5):
            box = RigidBodyDefinition(
                id=f"box_{i}",
                kind=BodyType.DYNAMIC,
                shape=ShapeType.BOX,
                size=[1.0, box_height, 1.0],
                mass=2.0,
                position=[0.0, 0.5 + i * box_height, 0.0],
                friction=0.5,
                restitution=0.1,  # Low bounce
            )
            await provider.add_body(sim.sim_id, box)
        print("   ✓ 5 boxes stacked (2kg each)")

        # Let physics settle
        print("\n4️⃣  Letting tower settle for 5 seconds...")
        for i in range(5):
            await provider.step_simulation(sim.sim_id, steps=60)  # 1 second
            state = await provider.get_simulation_state(sim.sim_id)

            # Check top box height
            top_box = next(b for b in state.bodies if b.id == "box_4")
            print(f"   t={state.time:.1f}s: Top box height = {top_box.position[1]:.3f}m")

        await provider.destroy_simulation(sim.sim_id)
        print("\n   ✓ Simulation cleaned up")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        service_url = os.environ.get("RAPIER_SERVICE_URL", "https://rapier.chukai.io")
        print(f"\nCannot connect to Rapier service at {service_url}")
        print("Using public service: https://rapier.chukai.io (default)")
        print("For local service: export RAPIER_SERVICE_URL=http://localhost:9000")


async def main():
    """Run all Rapier simulation examples."""
    print("=" * 70)
    print("RAPIER PHYSICS SIMULATION EXAMPLES")
    print("=" * 70)
    service_url = os.environ.get("RAPIER_SERVICE_URL", "https://rapier.chukai.io")
    print(f"\nThese examples use the Rapier service at: {service_url}")
    print("(Public service available, or set RAPIER_SERVICE_URL for local testing)")
    print("\nIf you want to run examples without external services,")
    print("use examples 00-04 which use the built-in analytic provider.")

    try:
        # Test connection first
        provider = get_provider("rapier")
        # This will fail if service isn't running
        test_config = SimulationConfig(gravity=[0, -9.81, 0], dimensions=3)
        await provider.create_simulation(test_config)

        print("\n✓ Rapier service is running! Proceeding with examples...\n")

    except Exception as e:
        print(f"\n❌ Cannot connect to Rapier service: {e}")
        print("\nPossible solutions:")
        print("  1. Check your internet connection (public service at https://rapier.chukai.io)")
        print("  2. Run analytic examples instead (00-04)")
        print("  3. Run local Rapier service: export RAPIER_SERVICE_URL=http://localhost:9000")
        return

    # Run examples
    await example_bouncing_ball()
    await example_collision_simulation()
    await example_stacking_boxes()

    print("\n" + "=" * 70)
    print("All examples completed successfully!")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
