#!/usr/bin/env python3
"""Example 1: Simple Projectile Motion Calculation

This example shows how to use the physics MCP server to calculate
the trajectory of a projectile (like a cannonball or basketball).
"""

import asyncio
from chuk_mcp_physics.providers.analytic import AnalyticProvider
from chuk_mcp_physics.models import ProjectileMotionRequest


async def main():
    """Calculate projectile motion for a cannonball."""

    # Create the analytic provider
    provider = AnalyticProvider()

    print("=" * 60)
    print("CANNONBALL TRAJECTORY CALCULATOR")
    print("=" * 60)
    print()

    # Example 1: 45-degree shot (maximum range)
    print("1. Cannonball fired at 45° (maximum range angle)")
    print("-" * 60)

    request = ProjectileMotionRequest(
        initial_velocity=50.0,  # 50 m/s
        angle_degrees=45.0,  # 45 degrees
        initial_height=0.0,  # Ground level
        gravity=9.81,  # Earth gravity
    )

    result = await provider.calculate_projectile_motion(request)

    print(f"Initial velocity: {request.initial_velocity} m/s")
    print(f"Launch angle: {request.angle_degrees}°")
    print(f"Initial height: {request.initial_height} m")
    print()
    print("RESULTS:")
    print(f"  Maximum height: {result.max_height:.2f} m")
    print(f"  Range: {result.range:.2f} m")
    print(f"  Time of flight: {result.time_of_flight:.2f} s")
    print(f"  Trajectory points: {len(result.trajectory_points)} samples")
    print()

    # Show some trajectory points
    print("Sample trajectory points (x, y):")
    for i in range(0, len(result.trajectory_points), 10):
        x, y = result.trajectory_points[i]
        print(
            f"  t={i * result.time_of_flight / len(result.trajectory_points):.2f}s: "
            f"({x:.1f}m, {y:.1f}m)"
        )
    print()

    # Example 2: Basketball free throw
    print("2. Basketball free throw simulation")
    print("-" * 60)

    # Free throw is 4.6m from basket, basket is 3.05m high, player releases at 2.0m
    # We need to find the right angle for a given velocity

    request = ProjectileMotionRequest(
        initial_velocity=7.0,  # 7 m/s (typical free throw speed)
        angle_degrees=52.0,  # Optimal angle for free throw
        initial_height=2.0,  # Player's release height
        gravity=9.81,
    )

    result = await provider.calculate_projectile_motion(request)

    print(f"Initial velocity: {request.initial_velocity} m/s")
    print(f"Launch angle: {request.angle_degrees}°")
    print(f"Release height: {request.initial_height} m")
    print()
    print("RESULTS:")
    print(f"  Maximum height: {result.max_height:.2f} m")
    print(f"  Range: {result.range:.2f} m")
    print(f"  Time of flight: {result.time_of_flight:.2f} s")
    print()

    # Check if it goes through the basket (4.6m away, 3.05m high)
    basket_distance = 4.6
    basket_height = 3.05

    # Find the height at basket distance
    for i, (x, y) in enumerate(result.trajectory_points):
        if x >= basket_distance:
            print(f"At basket position ({basket_distance}m):")
            print(f"  Ball height: {y:.2f} m")
            print(f"  Basket height: {basket_height:.2f} m")
            if abs(y - basket_height) < 0.5:
                print("  ✓ SWISH! Ball passes through basket!")
            elif y > basket_height:
                print(f"  ↑ Ball is {y - basket_height:.2f}m above basket")
            else:
                print(f"  ↓ Ball is {basket_height - y:.2f}m below basket")
            break
    print()

    # Example 3: Compare different angles
    print("3. Comparing different launch angles (same velocity)")
    print("-" * 60)
    print(f"{'Angle':<8} {'Max Height':<12} {'Range':<12} {'Flight Time':<12}")
    print("-" * 60)

    for angle in [15, 30, 45, 60, 75]:
        request = ProjectileMotionRequest(
            initial_velocity=30.0, angle_degrees=float(angle), initial_height=0.0, gravity=9.81
        )
        result = await provider.calculate_projectile_motion(request)

        print(
            f"{angle}°{'':<6} {result.max_height:<12.2f} {result.range:<12.2f} {result.time_of_flight:<12.2f}"
        )

    print()
    print("=" * 60)
    print("Note: 45° gives maximum range for projectiles!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
