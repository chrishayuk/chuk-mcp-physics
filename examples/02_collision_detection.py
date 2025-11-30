#!/usr/bin/env python3
"""Example 2: Collision Detection

This example demonstrates collision prediction between moving objects,
useful for crash avoidance, game physics, or accident analysis.
"""

import asyncio
from chuk_mcp_physics.providers.analytic import AnalyticProvider
from chuk_mcp_physics.models import CollisionCheckRequest


async def main():
    """Demonstrate collision detection scenarios."""

    provider = AnalyticProvider()

    print("=" * 60)
    print("COLLISION DETECTION EXAMPLES")
    print("=" * 60)
    print()

    # Example 1: Head-on car collision
    print("1. Head-on car collision")
    print("-" * 60)

    request = CollisionCheckRequest(
        body1_position=[0.0, 0.0, 0.0],  # Car A at origin
        body1_velocity=[20.0, 0.0, 0.0],  # Moving east at 20 m/s (~45 mph)
        body1_radius=2.5,  # ~5m car length
        body2_position=[100.0, 0.0, 0.0],  # Car B 100m away
        body2_velocity=[-25.0, 0.0, 0.0],  # Moving west at 25 m/s (~56 mph)
        body2_radius=2.5,
        max_time=10.0,
    )

    result = await provider.check_collision(request)

    print("Car A: Position (0, 0, 0), Velocity (20, 0, 0) m/s")
    print("Car B: Position (100, 0, 0), Velocity (-25, 0, 0) m/s")
    print()
    print("RESULTS:")
    print(f"  Will collide: {result.will_collide}")
    if result.will_collide:
        print("  ⚠️  COLLISION ALERT!")
        print(f"  Time to impact: {result.collision_time:.2f} seconds")
        print(f"  Collision point: {result.collision_point}")
        print(
            f"  Impact speed: {result.impact_speed:.1f} m/s ({result.impact_speed * 2.237:.1f} mph)"
        )
        print(f"  Closest approach: {result.closest_approach_distance:.2f} m")
    print()

    # Example 2: Near miss
    print("2. Near miss scenario (parallel lanes)")
    print("-" * 60)

    request = CollisionCheckRequest(
        body1_position=[0.0, 0.0, 0.0],  # Car A in lane 1
        body1_velocity=[30.0, 0.0, 0.0],  # Both going east
        body1_radius=2.5,
        body2_position=[0.0, 4.0, 0.0],  # Car B in lane 2 (4m apart)
        body2_velocity=[28.0, 0.0, 0.0],  # Slightly slower
        body2_radius=2.5,
        max_time=10.0,
    )

    result = await provider.check_collision(request)

    print("Car A: Lane 1, 30 m/s")
    print("Car B: Lane 2 (4m apart), 28 m/s")
    print()
    print("RESULTS:")
    print(f"  Will collide: {result.will_collide}")
    print(f"  Closest approach: {result.closest_approach_distance:.2f} m")
    print(f"  Time of closest approach: {result.closest_approach_time:.2f} s")
    if not result.will_collide:
        print(f"  ✓ Safe - vehicles maintain {result.closest_approach_distance:.2f}m separation")
    print()

    # Example 3: Asteroid tracking
    print("3. Asteroid collision prediction")
    print("-" * 60)

    # Scale up to km/s and km for space scenario
    request = CollisionCheckRequest(
        body1_position=[0.0, 0.0, 0.0],  # Asteroid A
        body1_velocity=[15000.0, 0.0, 0.0],  # 15 km/s
        body1_radius=500.0,  # 500m radius
        body2_position=[100000.0, 2000.0, 0.0],  # Asteroid B (100km away, 2km offset)
        body2_velocity=[-12000.0, 0.0, 0.0],  # -12 km/s
        body2_radius=300.0,  # 300m radius
        max_time=10.0,
    )

    result = await provider.check_collision(request)

    print("Asteroid A: 500m radius, velocity 15 km/s")
    print("Asteroid B: 300m radius, velocity -12 km/s")
    print("Initial separation: 100 km")
    print()
    print("RESULTS:")
    print(f"  Will collide: {result.will_collide}")
    if result.will_collide:
        print("  ⚠️  COLLISION PREDICTED!")
        print(f"  Time to impact: {result.collision_time:.3f} seconds")
        print(
            f"  Impact speed: {result.impact_speed:.0f} m/s ({result.impact_speed / 1000:.1f} km/s)"
        )
    else:
        print("  ✓ No collision")
        print(f"  Closest approach: {result.closest_approach_distance / 1000:.2f} km")
        print(f"  Time of closest approach: {result.closest_approach_time:.3f} s")
    print()

    # Example 4: Multiple collision scenarios
    print("4. Testing multiple scenarios")
    print("-" * 60)

    scenarios = [
        ("Head-on", [0, 0, 0], [10, 0, 0], [50, 0, 0], [-10, 0, 0], 1.0, 1.0),
        ("Rear-end", [0, 0, 0], [20, 0, 0], [30, 0, 0], [15, 0, 0], 2.5, 2.5),
        ("Crossing paths", [0, 0, 0], [10, 0, 0], [0, 20, 0], [0, -10, 0], 1.0, 1.0),
        ("Near miss", [0, 0, 0], [10, 0, 0], [0, 5, 0], [10, 0, 0], 1.0, 1.0),
    ]

    print(f"{'Scenario':<15} {'Collision?':<12} {'Time (s)':<12} {'Impact Speed (m/s)':<20}")
    print("-" * 60)

    for name, p1, v1, p2, v2, r1, r2 in scenarios:
        request = CollisionCheckRequest(
            body1_position=p1,
            body1_velocity=v1,
            body1_radius=r1,
            body2_position=p2,
            body2_velocity=v2,
            body2_radius=r2,
            max_time=10.0,
        )
        result = await provider.check_collision(request)

        collision = "YES ⚠️" if result.will_collide else "NO ✓"
        time_str = f"{result.collision_time:.2f}" if result.collision_time else "N/A"
        speed_str = f"{result.impact_speed:.1f}" if result.impact_speed else "N/A"

        print(f"{name:<15} {collision:<12} {time_str:<12} {speed_str:<20}")

    print()
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
