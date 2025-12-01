"""Sports Projectile Motion Examples: Comparing Drag vs. No Drag.

This example demonstrates the dramatic impact of air resistance on sports
balls. Compare realistic trajectories (with drag) to ideal/vacuum conditions.

Common drag coefficients (Cd):
    - Baseball: 0.4
    - Golf ball (dimpled): 0.25
    - Basketball: 0.55
    - Soccer ball: 0.25
    - Tennis ball: 0.55
    - Football (American): 0.05-0.15
"""

from chuk_mcp_physics.models import ProjectileWithDragRequest, ProjectileMotionRequest
from chuk_mcp_physics.kinematics import calculate_projectile_with_drag
from chuk_mcp_physics.providers.analytic import AnalyticProvider


def baseball_90mph_fastball():
    """Compare baseball pitch with and without drag."""
    print("\n" + "=" * 70)
    print("⚾ BASEBALL: 90 mph Fastball")
    print("=" * 70)

    # 90 mph = 40.23 m/s
    velocity = 40.23
    angle = 10.0  # Low angle pitch
    height = 1.8  # Pitcher's release height

    # WITH DRAG (realistic)
    with_drag = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
        mass=0.145,  # kg (baseball)
        drag_coefficient=0.4,  # baseball Cd
        cross_sectional_area=0.0043,  # π * (0.037m)² for baseball
        fluid_density=1.225,  # air density
    )

    result_drag = calculate_projectile_with_drag(with_drag)

    # WITHOUT DRAG (vacuum/ideal)
    provider = AnalyticProvider()
    no_drag = ProjectileMotionRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
    )

    import asyncio

    result_nodrag = asyncio.run(provider.calculate_projectile_motion(no_drag))

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s (90 mph)")
    print(f"   Angle: {angle}°")
    print(f"   Height: {height} m")
    print("   Mass: 0.145 kg")
    print("   Drag coefficient: 0.4")

    print("\n🌪️  WITH AIR RESISTANCE (Realistic):")
    print(f"   Range: {result_drag.range:.1f} m ({result_drag.range * 3.28:.1f} ft)")
    print(f"   Max height: {result_drag.max_height:.1f} m")
    print(f"   Flight time: {result_drag.time_of_flight:.2f} s")
    print(f"   Impact velocity: {result_drag.impact_velocity:.1f} m/s")
    print(f"   Energy lost to drag: {result_drag.energy_lost_to_drag:.1f} J")

    print("\n🌌 WITHOUT AIR RESISTANCE (Vacuum):")
    print(f"   Range: {result_nodrag.range:.1f} m ({result_nodrag.range * 3.28:.1f} ft)")
    print(f"   Max height: {result_nodrag.max_height:.1f} m")
    print(f"   Flight time: {result_nodrag.time_of_flight:.2f} s")

    print("\n📊 DRAG IMPACT:")
    print(f"   Range reduction: {(1 - result_drag.range / result_nodrag.range) * 100:.1f}%")
    print(f"   Energy dissipated: {result_drag.energy_lost_to_drag:.1f} J")
    print(
        f"   ({result_drag.energy_lost_to_drag / result_drag.initial_kinetic_energy * 100:.1f}% of initial energy)"
    )


def golf_drive():
    """Compare golf drive with and without drag."""
    print("\n" + "=" * 70)
    print("⛳ GOLF: Pro Drive (155 mph clubhead speed)")
    print("=" * 70)

    velocity = 70.0  # m/s (~155 mph)
    angle = 12.0  # Optimal launch angle for distance
    height = 0.0

    # WITH DRAG (realistic)
    with_drag = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
        mass=0.0459,  # kg (golf ball)
        drag_coefficient=0.25,  # dimpled golf ball (reduced Cd)
        cross_sectional_area=0.00143,  # π * (0.0213m)² for golf ball
        fluid_density=1.225,
    )

    result_drag = calculate_projectile_with_drag(with_drag)

    # WITHOUT DRAG
    provider = AnalyticProvider()
    no_drag = ProjectileMotionRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
    )

    import asyncio

    result_nodrag = asyncio.run(provider.calculate_projectile_motion(no_drag))

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s ({velocity * 2.237:.0f} mph)")
    print(f"   Angle: {angle}°")
    print("   Mass: 0.0459 kg")
    print("   Drag coefficient: 0.25 (dimpled)")

    print("\n🌪️  WITH AIR RESISTANCE (Realistic):")
    print(f"   Range: {result_drag.range:.1f} m ({result_drag.range * 1.094:.1f} yards)")
    print(f"   Max height: {result_drag.max_height:.1f} m")
    print(f"   Flight time: {result_drag.time_of_flight:.2f} s")
    print(f"   Impact velocity: {result_drag.impact_velocity:.1f} m/s")

    print("\n🌌 WITHOUT AIR RESISTANCE (Vacuum):")
    print(f"   Range: {result_nodrag.range:.1f} m ({result_nodrag.range * 1.094:.1f} yards)")
    print(f"   Max height: {result_nodrag.max_height:.1f} m")

    print("\n📊 DRAG IMPACT:")
    print(f"   Range reduction: {(1 - result_drag.range / result_nodrag.range) * 100:.1f}%")
    print("   ⚠️  Dimples reduce drag by ~50% (Cd 0.47 → 0.25)!")
    print(f"   Without dimples, range would be ~{result_drag.range * 0.7:.1f} m")


def basketball_three_pointer():
    """Compare basketball 3-point shot with and without drag."""
    print("\n" + "=" * 70)
    print("🏀 BASKETBALL: 3-Point Shot")
    print("=" * 70)

    velocity = 7.5  # m/s
    angle = 48.0  # High arc shot
    height = 2.0  # Release height

    # WITH DRAG (realistic)
    with_drag = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
        mass=0.624,  # kg (basketball)
        drag_coefficient=0.55,  # basketball Cd
        cross_sectional_area=0.0456,  # π * (0.12m)² for basketball
        fluid_density=1.225,
    )

    result_drag = calculate_projectile_with_drag(with_drag)

    # WITHOUT DRAG
    provider = AnalyticProvider()
    no_drag = ProjectileMotionRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
    )

    import asyncio

    result_nodrag = asyncio.run(provider.calculate_projectile_motion(no_drag))

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s")
    print(f"   Angle: {angle}° (high arc)")
    print(f"   Release height: {height} m")
    print("   Mass: 0.624 kg")
    print("   Target: 6.75 m (3-point line)")

    print("\n🌪️  WITH AIR RESISTANCE (Realistic):")
    print(f"   Range: {result_drag.range:.2f} m")
    print(f"   Max height: {result_drag.max_height:.2f} m")
    print(f"   Flight time: {result_drag.time_of_flight:.2f} s")
    print(f"   Impact velocity: {result_drag.impact_velocity:.1f} m/s")

    print("\n🌌 WITHOUT AIR RESISTANCE (Vacuum):")
    print(f"   Range: {result_nodrag.range:.2f} m")
    print(f"   Max height: {result_nodrag.max_height:.2f} m")

    print("\n📊 DRAG IMPACT:")
    print(f"   Range reduction: {(1 - result_drag.range / result_nodrag.range) * 100:.1f}%")
    print(
        f"   For 3-point shots, drag reduces range by ~{result_nodrag.range - result_drag.range:.2f} m"
    )


def soccer_penalty_kick():
    """Compare soccer penalty kick with and without drag."""
    print("\n" + "=" * 70)
    print("⚽ SOCCER: Penalty Kick")
    print("=" * 70)

    velocity = 25.0  # m/s (~56 mph)
    angle = 10.0  # Low angle
    height = 0.3  # Ball on ground

    # WITH DRAG
    with_drag = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
        mass=0.43,  # kg (soccer ball)
        drag_coefficient=0.25,  # modern soccer ball
        cross_sectional_area=0.0388,  # π * (0.111m)² for soccer ball
        fluid_density=1.225,
    )

    result_drag = calculate_projectile_with_drag(with_drag)

    # WITHOUT DRAG
    provider = AnalyticProvider()
    no_drag = ProjectileMotionRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=height,
    )

    import asyncio

    result_nodrag = asyncio.run(provider.calculate_projectile_motion(no_drag))

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s ({velocity * 2.237:.0f} mph)")
    print(f"   Angle: {angle}°")
    print("   Mass: 0.43 kg")
    print("   Goal distance: 11 m (penalty spot)")

    print("\n🌪️  WITH AIR RESISTANCE (Realistic):")
    print(f"   Range: {result_drag.range:.1f} m")
    print(f"   Max height: {result_drag.max_height:.2f} m")
    print(f"   Flight time: {result_drag.time_of_flight:.3f} s")
    print(f"   Time to goal (11m): ~{11 / velocity:.3f} s")

    print("\n🌌 WITHOUT AIR RESISTANCE (Vacuum):")
    print(f"   Range: {result_nodrag.range:.1f} m")
    print(f"   Max height: {result_nodrag.max_height:.2f} m")

    print("\n📊 DRAG IMPACT:")
    print(f"   Range reduction: {(1 - result_drag.range / result_nodrag.range) * 100:.1f}%")
    print("   Less impact at short distances!")


if __name__ == "__main__":
    print("\n" + "🎾 " * 20)
    print("SPORTS PROJECTILES: THE IMPACT OF AIR RESISTANCE")
    print("🎾 " * 20)
    print("\nThis example demonstrates how drag dramatically affects sports balls.")
    print("Real-world trajectories (WITH drag) vs ideal/vacuum conditions (NO drag).")

    baseball_90mph_fastball()
    golf_drive()
    basketball_three_pointer()
    soccer_penalty_kick()

    print("\n" + "=" * 70)
    print("💡 KEY TAKEAWAYS")
    print("=" * 70)
    print("1. Drag reduces projectile range by 20-70% depending on velocity and shape")
    print("2. Higher velocity = more drag force (quadratic relationship)")
    print("3. Golf ball dimples reduce drag coefficient by ~50%!")
    print("4. Baseball range reduced by ~40% due to air resistance")
    print("5. Drag effect increases with projectile speed (v²)")
    print("\n✅ Always include drag for realistic sports simulations!")
    print()
