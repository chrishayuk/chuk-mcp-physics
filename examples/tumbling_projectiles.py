"""Tumbling Projectile Examples - Orientation-Dependent Drag.

Demonstrates how object orientation affects aerodynamic drag using Rapier simulations.
This shows realistic physics for objects like footballs, frisbees, and other projectiles
that have dramatically different drag depending on their orientation.

Requires: PHYSICS_PROVIDER=rapier and Rapier service running
"""

import asyncio
import math
import os

os.environ["RAPIER_SERVICE_URL"] = "https://rapier.chukai.io"
os.environ["PHYSICS_PROVIDER"] = "rapier"

from chuk_mcp_physics.server import (
    create_simulation,
    add_rigid_body,
    step_simulation,
    destroy_simulation,
)


async def football_spiral_vs_tumble():
    """Compare football with perfect spiral (low drag) vs tumbling end-over-end (high drag).

    A football in a perfect spiral has minimal drag because it's streamlined along
    the direction of motion. When tumbling, it presents a much larger cross-section
    and experiences significantly more drag, reducing range dramatically.

    Football specs:
        - Mass: 0.42 kg (NFL regulation)
        - Length: 0.28 m
        - Diameter: 0.17 m
        - Cd (streamlined): ~0.05-0.15
        - Cd (broadside): ~0.5-0.8
    """
    print("\n" + "=" * 80)
    print("FOOTBALL: Perfect Spiral vs End-Over-End Tumble")
    print("=" * 80)

    # Initial velocity for 50 yard throw (~45 m)
    v0 = 25.0  # m/s (~56 mph)
    angle = 30.0  # degrees
    vx = v0 * math.cos(math.radians(angle))
    vy = v0 * math.sin(math.radians(angle))

    # Football dimensions
    mass = 0.42  # kg
    length = 0.28  # m
    diameter = 0.17  # m

    # Cross-sectional areas
    area_streamlined = math.pi * (diameter / 2) ** 2  # ~0.023 m²
    area_broadside = length * diameter  # ~0.048 m²

    # Drag coefficients
    cd_streamlined = 0.1  # Low drag when pointed into wind
    cd_broadside = 0.6  # High drag when sideways

    # ========================================================================
    # Scenario 1: Perfect Spiral (stable orientation, streamlined)
    # ========================================================================
    print("\n--- Scenario 1: PERFECT SPIRAL ---")
    print("Football maintains streamlined orientation (like a well-thrown pass)")

    sim_spiral = await create_simulation(gravity_y=-9.81)
    sim_id_spiral = sim_spiral.sim_id

    # For a spiral, the football is streamlined along its velocity direction
    # We use drag_axis_ratios to make drag much lower along the long axis (Y)
    await add_rigid_body(
        sim_id=sim_id_spiral,
        body_id="football_spiral",
        body_type="dynamic",
        shape="sphere",  # Using sphere for simplicity (capsule would be more accurate)
        size=[diameter / 2],
        mass=mass,
        position=[0.0, 2.0, 0.0],  # 2m high (QB release)
        velocity=[vx, vy, 0.0],
        angular_velocity=[0.0, 126.0, 0.0],  # 20 rev/s spiral
        linear_damping=0.0,  # No artificial damping (physics handles drag)
        angular_damping=0.1,  # Slight spin decay
        # Orientation-dependent drag parameters
        drag_coefficient=cd_streamlined,  # Base Cd
        drag_area=area_streamlined,  # Reference area
        drag_axis_ratios=[1.0, 0.2, 1.0],  # Much lower drag along Y (flight direction)
        fluid_density=1.225,  # Sea level air
    )

    # Run simulation
    trajectory_spiral = []
    for _ in range(200):  # ~2 seconds at 0.01s steps
        step = await step_simulation(sim_id=sim_id_spiral, dt=0.01)
        body_state = next(b for b in step.bodies if b.id == "football_spiral")
        trajectory_spiral.append((body_state.position[0], body_state.position[1]))

        # Stop when hits ground
        if body_state.position[1] < 0:
            break

    # Use max x (not final x) to avoid negative ranges from solver jitter at impact
    xs_spiral = [x for x, y in trajectory_spiral]
    ys_spiral = [y for x, y in trajectory_spiral]
    range_spiral = max(xs_spiral)
    max_height_spiral = max(ys_spiral)

    print(f"  Range: {range_spiral:.1f} m ({range_spiral * 1.09361:.1f} yards)")
    print(f"  Max height: {max_height_spiral:.1f} m")
    print(f"  Trajectory points: {len(trajectory_spiral)}")

    await destroy_simulation(sim_id=sim_id_spiral)

    # ========================================================================
    # Scenario 2: End-Over-End Tumble (rotating, high drag)
    # ========================================================================
    print("\n--- Scenario 2: END-OVER-END TUMBLE ---")
    print("Football tumbles (like a bad pass or punt)")

    sim_tumble = await create_simulation(gravity_y=-9.81)
    sim_id_tumble = sim_tumble.sim_id

    # For tumbling, the football constantly changes orientation
    # Use higher base Cd and less variation (averages out)
    await add_rigid_body(
        sim_id=sim_id_tumble,
        body_id="football_tumble",
        body_type="dynamic",
        shape="sphere",
        size=[diameter / 2],
        mass=mass,
        position=[0.0, 2.0, 0.0],
        velocity=[vx, vy, 0.0],
        angular_velocity=[31.4, 0.0, 0.0],  # Tumbling rotation (5 rev/s around X)
        linear_damping=0.0,
        angular_damping=0.05,
        # Orientation-dependent drag - averages to higher drag
        drag_coefficient=cd_broadside,  # Higher base Cd
        drag_area=area_broadside,  # Larger reference area
        drag_axis_ratios=[0.8, 1.0, 0.8],  # Less streamlining effect
        fluid_density=1.225,
    )

    # Run simulation
    trajectory_tumble = []
    for _ in range(200):
        step = await step_simulation(sim_id=sim_id_tumble, dt=0.01)
        body_state = next(b for b in step.bodies if b.id == "football_tumble")
        trajectory_tumble.append((body_state.position[0], body_state.position[1]))

        if body_state.position[1] < 0:
            break

    xs_tumble = [x for x, y in trajectory_tumble]
    ys_tumble = [y for x, y in trajectory_tumble]
    range_tumble = max(xs_tumble)
    max_height_tumble = max(ys_tumble)

    print(f"  Range: {range_tumble:.1f} m ({range_tumble * 1.09361:.1f} yards)")
    print(f"  Max height: {max_height_tumble:.1f} m")
    print(f"  Trajectory points: {len(trajectory_tumble)}")

    await destroy_simulation(sim_id=sim_id_tumble)

    # ========================================================================
    # Comparison
    # ========================================================================
    print("\n" + "-" * 80)
    print("COMPARISON:")
    print(f"  Spiral range:       {range_spiral:.1f} m ({range_spiral * 1.09361:.1f} yd)")
    print(f"  Tumble range:       {range_tumble:.1f} m ({range_tumble * 1.09361:.1f} yd)")
    print(f"  Range reduction:    {(1 - range_tumble / range_spiral) * 100:.1f}%")
    print(
        f"  Extra distance:     {range_spiral - range_tumble:.1f} m ({(range_spiral - range_tumble) * 1.09361:.1f} yd)"
    )
    print("\nConclusion: A perfect spiral can gain 20-40% more distance!")


async def frisbee_stable_vs_wobble():
    """Compare frisbee with stable flight vs wobbling flight.

    A frisbee relies on gyroscopic stability from spin. When spinning fast and stable,
    it maintains a flat orientation with low drag. When wobbling or flying at an angle,
    drag increases significantly.

    Frisbee specs:
        - Mass: 0.175 kg
        - Diameter: 0.27 m
        - Thickness: 0.025 m
        - Cd (flat): ~0.08
        - Cd (edge): ~1.2
    """
    print("\n" + "=" * 80)
    print("FRISBEE: Stable Flight vs Wobbling")
    print("=" * 80)

    # Initial velocity for long throw
    v0 = 20.0  # m/s
    angle = 10.0  # degrees (frisbees are thrown fairly flat)
    vx = v0 * math.cos(math.radians(angle))
    vy = v0 * math.sin(math.radians(angle))

    # Frisbee dimensions
    mass = 0.175  # kg
    diameter = 0.27  # m
    thickness = 0.025  # m

    area_flat = math.pi * (diameter / 2) ** 2  # ~0.057 m²
    area_edge = diameter * thickness  # ~0.007 m²

    cd_flat = 0.08
    cd_edge = 1.2

    # ========================================================================
    # Scenario 1: Stable Flight (fast spin, flat orientation)
    # ========================================================================
    print("\n--- Scenario 1: STABLE FLIGHT ---")
    print("Frisbee spins fast and maintains flat orientation")

    sim_stable = await create_simulation(gravity_y=-9.81)
    sim_id_stable = sim_stable.sim_id

    await add_rigid_body(
        sim_id=sim_id_stable,
        body_id="frisbee_stable",
        body_type="dynamic",
        shape="sphere",
        size=[diameter / 2],
        mass=mass,
        position=[0.0, 1.5, 0.0],
        velocity=[vx, vy, 0.0],
        angular_velocity=[0.0, 62.8, 0.0],  # Fast spin (600 rpm)
        linear_damping=0.0,
        angular_damping=0.05,
        # Orientation-dependent drag - very low when flat (Y axis up)
        drag_coefficient=cd_flat,
        drag_area=area_flat,
        drag_axis_ratios=[1.0, 0.1, 1.0],  # Very low drag along Y (vertical)
        fluid_density=1.225,
    )

    trajectory_stable = []
    for _ in range(500):  # Longer flight
        step = await step_simulation(sim_id=sim_id_stable, dt=0.01)
        body_state = next(b for b in step.bodies if b.id == "frisbee_stable")
        trajectory_stable.append((body_state.position[0], body_state.position[1]))

        if body_state.position[1] < 0:
            break

    xs_stable = [x for x, y in trajectory_stable]
    ys_stable = [y for x, y in trajectory_stable]
    range_stable = max(xs_stable)
    max_height_stable = max(ys_stable)
    flight_time_stable = len(trajectory_stable) * 0.01

    print(f"  Range: {range_stable:.1f} m")
    print(f"  Max height: {max_height_stable:.1f} m")
    print(f"  Flight time: {flight_time_stable:.1f} s")

    await destroy_simulation(sim_id=sim_id_stable)

    # ========================================================================
    # Scenario 2: Wobbling Flight (slow spin, unstable)
    # ========================================================================
    print("\n--- Scenario 2: WOBBLING FLIGHT ---")
    print("Frisbee wobbles with slow spin")

    sim_wobble = await create_simulation(gravity_y=-9.81)
    sim_id_wobble = sim_wobble.sim_id

    await add_rigid_body(
        sim_id=sim_id_wobble,
        body_id="frisbee_wobble",
        body_type="dynamic",
        shape="sphere",
        size=[diameter / 2],
        mass=mass,
        position=[0.0, 1.5, 0.0],
        velocity=[vx, vy, 0.0],
        angular_velocity=[5.0, 20.9, 5.0],  # Slow spin + wobble
        linear_damping=0.0,
        angular_damping=0.1,
        # Higher drag due to tilted orientation
        drag_coefficient=(cd_flat + cd_edge) / 2,  # Average
        drag_area=(area_flat + area_edge) / 2,
        drag_axis_ratios=[0.6, 0.8, 0.6],  # More uniform (less streamlining)
        fluid_density=1.225,
    )

    trajectory_wobble = []
    for _ in range(500):
        step = await step_simulation(sim_id=sim_id_wobble, dt=0.01)
        body_state = next(b for b in step.bodies if b.id == "frisbee_wobble")
        trajectory_wobble.append((body_state.position[0], body_state.position[1]))

        if body_state.position[1] < 0:
            break

    xs_wobble = [x for x, y in trajectory_wobble]
    ys_wobble = [y for x, y in trajectory_wobble]
    range_wobble = max(xs_wobble)
    max_height_wobble = max(ys_wobble)
    flight_time_wobble = len(trajectory_wobble) * 0.01

    print(f"  Range: {range_wobble:.1f} m")
    print(f"  Max height: {max_height_wobble:.1f} m")
    print(f"  Flight time: {flight_time_wobble:.1f} s")

    await destroy_simulation(sim_id=sim_id_wobble)

    # ========================================================================
    # Comparison
    # ========================================================================
    print("\n" + "-" * 80)
    print("COMPARISON:")
    print(f"  Stable range:       {range_stable:.1f} m")
    print(f"  Wobble range:       {range_wobble:.1f} m")
    print(f"  Range reduction:    {(1 - range_wobble / range_stable) * 100:.1f}%")
    print(f"  Extra distance:     {range_stable - range_wobble:.1f} m")
    print("\nConclusion: Stable spin maintains flat orientation for much better range!")


async def javelin_optimal_angle():
    """Demonstrate javelin throw with orientation-dependent drag.

    A javelin must maintain a specific angle of attack for optimal flight.
    Too flat or too steep and drag increases. This example shows how
    orientation affects the throw distance.

    Javelin specs:
        - Mass: 0.8 kg (men's)
        - Length: 2.6 m
        - Diameter: 0.025 m
        - Cd (optimal angle): ~0.35
        - Cd (broadside): ~1.2
    """
    print("\n" + "=" * 80)
    print("JAVELIN: Optimal Angle vs Poor Angle")
    print("=" * 80)

    # Release velocity for good throw
    v0 = 30.0  # m/s (~67 mph)
    release_angle = 35.0  # degrees
    vx = v0 * math.cos(math.radians(release_angle))
    vy = v0 * math.sin(math.radians(release_angle))

    # Javelin dimensions
    mass = 0.8  # kg
    length = 2.6  # m
    diameter = 0.025  # m

    area_end = math.pi * (diameter / 2) ** 2  # ~0.0005 m²
    area_side = length * diameter  # ~0.065 m²

    cd_optimal = 0.35
    cd_broadside = 1.2

    # ========================================================================
    # Scenario 1: Optimal Flight (javelin pointed into wind)
    # ========================================================================
    print("\n--- Scenario 1: OPTIMAL FLIGHT ---")
    print("Javelin maintains optimal angle of attack (~35°)")

    sim_optimal = await create_simulation(gravity_y=-9.81)
    sim_id_optimal = sim_optimal.sim_id

    await add_rigid_body(
        sim_id=sim_id_optimal,
        body_id="javelin_optimal",
        body_type="dynamic",
        shape="sphere",
        size=[diameter],
        mass=mass,
        position=[0.0, 2.0, 0.0],  # Release height
        velocity=[vx, vy, 0.0],
        angular_velocity=[0.0, 5.0, 0.0],  # Slight rotation for stability
        linear_damping=0.0,
        angular_damping=0.1,
        # Streamlined along long axis (Y)
        drag_coefficient=cd_optimal,
        drag_area=area_end,
        drag_axis_ratios=[1.5, 0.2, 1.5],  # Very streamlined along Y
        fluid_density=1.225,
    )

    trajectory_optimal = []
    for _ in range(400):
        step = await step_simulation(sim_id=sim_id_optimal, dt=0.01)
        body_state = next(b for b in step.bodies if b.id == "javelin_optimal")
        trajectory_optimal.append((body_state.position[0], body_state.position[1]))

        if body_state.position[1] < 0:
            break

    xs_opt = [x for x, y in trajectory_optimal]
    ys_opt = [y for x, y in trajectory_optimal]
    range_optimal = max(xs_opt)
    max_height_optimal = max(ys_opt)

    print(f"  Range: {range_optimal:.1f} m")
    print(f"  Max height: {max_height_optimal:.1f} m")

    await destroy_simulation(sim_id=sim_id_optimal)

    # ========================================================================
    # Scenario 2: Poor Release (javelin at bad angle, tumbling)
    # ========================================================================
    print("\n--- Scenario 2: POOR RELEASE ---")
    print("Javelin tumbles and presents broadside to wind")

    sim_poor = await create_simulation(gravity_y=-9.81)
    sim_id_poor = sim_poor.sim_id

    await add_rigid_body(
        sim_id=sim_id_poor,
        body_id="javelin_poor",
        body_type="dynamic",
        shape="sphere",
        size=[diameter],
        mass=mass,
        position=[0.0, 2.0, 0.0],
        velocity=[vx, vy, 0.0],
        angular_velocity=[10.0, 2.0, 5.0],  # Tumbling
        linear_damping=0.0,
        angular_damping=0.05,
        # Much higher drag due to tumbling
        drag_coefficient=cd_broadside,
        drag_area=area_side,
        drag_axis_ratios=[0.8, 1.0, 0.8],  # Less streamlining
        fluid_density=1.225,
    )

    trajectory_poor = []
    for _ in range(400):
        step = await step_simulation(sim_id=sim_id_poor, dt=0.01)
        body_state = next(b for b in step.bodies if b.id == "javelin_poor")
        trajectory_poor.append((body_state.position[0], body_state.position[1]))

        if body_state.position[1] < 0:
            break

    xs_poor = [x for x, y in trajectory_poor]
    ys_poor = [y for x, y in trajectory_poor]
    range_poor = max(xs_poor)
    max_height_poor = max(ys_poor)

    print(f"  Range: {range_poor:.1f} m")
    print(f"  Max height: {max_height_poor:.1f} m")

    await destroy_simulation(sim_id=sim_id_poor)

    # ========================================================================
    # Comparison
    # ========================================================================
    print("\n" + "-" * 80)
    print("COMPARISON:")
    print(f"  Optimal range:      {range_optimal:.1f} m")
    print(f"  Poor range:         {range_poor:.1f} m")
    print(f"  Range reduction:    {(1 - range_poor / range_optimal) * 100:.1f}%")
    print(f"  Distance lost:      {range_optimal - range_poor:.1f} m")
    print("\nConclusion: Proper technique and orientation is crucial for distance!")


async def main():
    """Run all tumbling projectile examples."""
    print("\n" + "=" * 80)
    print("TUMBLING PROJECTILE EXAMPLES")
    print("Demonstrating Orientation-Dependent Drag with Rapier Physics")
    print("=" * 80)
    print("\nThese examples show how object orientation dramatically affects")
    print("aerodynamic drag and range. Real-world sports rely on this physics!")

    try:
        await football_spiral_vs_tumble()
        await frisbee_stable_vs_wobble()
        await javelin_optimal_angle()

        print("\n" + "=" * 80)
        print("All examples completed successfully!")
        print("=" * 80)

    except Exception as e:
        print(f"\n❌ Error running examples: {e}")
        print("\nMake sure:")
        print("  1. PHYSICS_PROVIDER=rapier environment variable is set")
        print("  2. Rapier service is running and accessible")
        print("  3. Rapier service implements orientation-dependent drag")


if __name__ == "__main__":
    asyncio.run(main())
