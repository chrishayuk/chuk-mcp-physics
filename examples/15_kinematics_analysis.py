"""
Example 15: Kinematics Analysis (Phase 2.7)

Demonstrates motion analysis calculations including:
- Acceleration from position data
- Jerk calculations
- Trajectory fitting
- Motion graphs
- Average speed
- Instantaneous velocity

No external services required - uses analytic provider.
"""

import asyncio
from chuk_mcp_physics.tools.kinematics_tools import (
    calculate_acceleration_from_position,
    calculate_jerk,
    fit_trajectory,
    generate_motion_graph,
    calculate_average_speed,
    calculate_instantaneous_velocity,
)


async def main():
    print("\n" + "=" * 70)
    print("KINEMATICS ANALYSIS EXAMPLES (Phase 2.7)")
    print("=" * 70)

    # Example 1: Acceleration from Position - Motion capture data
    print("\n1. Calculate Acceleration from Position Data")
    print("-" * 70)
    print("Scenario: Analyzing motion capture data from accelerating object")

    times = [0, 1, 2, 3, 4]
    positions = [[0, 0, 0], [5, 0, 0], [20, 0, 0], [45, 0, 0], [80, 0, 0]]

    result = await calculate_acceleration_from_position(times=times, positions=positions)

    print("\nPosition data (x-axis motion):")
    for i, (t, p) in enumerate(zip(times, positions)):
        print(f"  t = {t}s: x = {p[0]} m")

    print("\nCalculated kinematics:")
    print(f"  Average velocity: {result['average_velocity']} m/s")
    print(f"  Average acceleration: {result['average_acceleration']} m/s²")
    print("\n  The object is accelerating! (v increases with time)")

    # Example 2: Jerk - Smooth vs. jerky motion
    print("\n\n2. Jerk Analysis - Motion Smoothness")
    print("-" * 70)
    print("Scenario: Comparing smooth acceleration vs. jerky motion")

    # Constant acceleration (smooth)
    times_smooth = [0, 1, 2, 3, 4]
    accels_smooth = [[2, 0, 0], [2, 0, 0], [2, 0, 0], [2, 0, 0], [2, 0, 0]]

    result_smooth = await calculate_jerk(times=times_smooth, accelerations=accels_smooth)

    # Variable acceleration (jerky)
    times_jerky = [0, 1, 2, 3, 4]
    accels_jerky = [[0, 0, 0], [5, 0, 0], [2, 0, 0], [8, 0, 0], [1, 0, 0]]

    result_jerky = await calculate_jerk(times=times_jerky, accelerations=accels_jerky)

    print("\nSmooth motion (constant acceleration):")
    print(f"  Average jerk: {result_smooth['average_jerk']} m/s³")
    print(f"  Max jerk magnitude: {result_smooth['max_jerk_magnitude']:.3f} m/s³")

    print("\nJerky motion (varying acceleration):")
    print(f"  Average jerk: {result_jerky['average_jerk']} m/s³")
    print(f"  Max jerk magnitude: {result_jerky['max_jerk_magnitude']:.2f} m/s³")

    print(
        f"\n  Jerky motion has {result_jerky['max_jerk_magnitude'] / max(result_smooth['max_jerk_magnitude'], 0.001):.0f}x more jerk!"
    )
    print("  High jerk = uncomfortable for passengers!")

    # Example 3: Trajectory Fitting - Projectile motion
    print("\n\n3. Trajectory Fitting - Finding the Equation")
    print("-" * 70)
    print("Scenario: Fitting parabola to projectile motion data")

    times_fit = [0, 1, 2, 3, 4]
    positions_fit = [[0, 0, 0], [10, 15, 0], [20, 20, 0], [30, 15, 0], [40, 0, 0]]

    result_fit = await fit_trajectory(
        times=times_fit, positions=positions_fit, fit_type="quadratic"
    )

    print("\nProjectile data points:")
    for t, p in zip(times_fit, positions_fit):
        print(f"  t = {t}s: ({p[0]}, {p[1]}) m")

    print("\nFitted equations (y = c₀ + c₁t + c₂t²):")
    print(
        f"  x(t) = {result_fit['coefficients_x'][0]:.2f} + {result_fit['coefficients_x'][1]:.2f}t + {result_fit['coefficients_x'][2]:.2f}t²"
    )
    print(
        f"  y(t) = {result_fit['coefficients_y'][0]:.2f} + {result_fit['coefficients_y'][1]:.2f}t + {result_fit['coefficients_y'][2]:.2f}t²"
    )
    print(f"  R² (goodness of fit): {result_fit['r_squared']:.4f}")

    print("\n  Y equation shows parabolic trajectory!")
    print("  Coefficient of t² ≈ -½g (gravitational acceleration)")

    # Example 4: Motion Graphs - Visualizing kinematics
    print("\n\n4. Motion Graph Generation")
    print("-" * 70)
    print("Scenario: Creating position/velocity/acceleration graphs")

    times_graph = [0, 1, 2, 3]
    positions_graph = [[0, 0, 0], [5, 0, 0], [20, 0, 0], [45, 0, 0]]

    result_graph = await generate_motion_graph(
        times=times_graph, positions=positions_graph, component="x"
    )

    print("\nGraph data for x-component:")
    print(f"{'Time (s)':<12} {'Position (m)':<15} {'Velocity (m/s)':<18} {'Accel (m/s²)':<15}")
    print("-" * 70)

    for i, t in enumerate(result_graph["times"]):
        p = result_graph["positions"][i]
        v = result_graph["velocities"][i]
        a = result_graph["accelerations"][i]
        print(f"{t:<12.1f} {p:<15.1f} {v:<18.1f} {a:<15.1f}")

    # Example 5: Average Speed - Path length vs. displacement
    print("\n\n5. Average Speed - Winding Path")
    print("-" * 70)
    print("Scenario: Car on winding road")

    positions_path = [[0, 0, 0], [10, 5, 0], [20, 10, 0], [15, 20, 0]]
    times_path = [0, 10, 20, 30]

    result_speed = await calculate_average_speed(positions=positions_path, times=times_path)

    print("\nPath taken:")
    for i, (t, p) in enumerate(zip(times_path, positions_path)):
        print(f"  Point {i + 1} at t={t}s: ({p[0]}, {p[1]}) m")

    print("\nRESULTS:")
    print(f"  Total distance traveled: {result_speed['total_distance']:.2f} m (path length)")
    print(
        f"  Displacement magnitude: {result_speed['displacement_magnitude']:.2f} m (straight line)"
    )
    print(f"  Total time: {result_speed['total_time']:.1f} s")
    print(f"  Average speed: {result_speed['average_speed']:.2f} m/s")

    print("\n  Distance > Displacement (winding path)")
    print(
        f"  Efficiency: {result_speed['displacement_magnitude'] / result_speed['total_distance'] * 100:.1f}%"
    )

    # Example 6: Instantaneous Velocity
    print("\n\n6. Instantaneous Velocity - Specific Time")
    print("-" * 70)
    print("Scenario: Finding velocity at exact moment in time")

    positions_inst = [[0, 0, 0], [3, 4, 0], [6, 8, 0]]
    times_inst = [0, 1, 2]

    # Calculate at t = 1.0s (on a data point)
    result_inst = await calculate_instantaneous_velocity(
        positions=positions_inst, times=times_inst, target_time=1.0
    )

    print("\nAt t = 1.0s (data point):")
    print(f"  Velocity: {result_inst['velocity']} m/s")
    print(f"  Speed: {result_inst['speed']:.2f} m/s")
    print(f"  Interpolated: {result_inst['interpolated']}")

    # Calculate at t = 0.5s (between points, requires interpolation)
    result_interp = await calculate_instantaneous_velocity(
        positions=positions_inst, times=times_inst, target_time=0.5
    )

    print("\nAt t = 0.5s (between points):")
    print(f"  Velocity: {result_interp['velocity']} m/s")
    print(f"  Speed: {result_interp['speed']:.2f} m/s")
    print(f"  Interpolated: {result_interp['interpolated']} ✓")

    print("\n" + "=" * 70)
    print("Phase 2.7 Complete! ✓")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
