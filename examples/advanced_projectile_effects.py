"""Advanced Projectile Motion Examples: Magnus Force, Wind, and Altitude.

This example demonstrates the enhanced features of calculate_projectile_with_drag:
- Spin effects (Magnus force) for curveballs, slices, hooks
- Wind effects (tailwind, headwind, crosswind)
- Altitude and temperature effects on air density

All calculations include realistic air resistance and show dramatic differences
compared to ideal vacuum conditions.
"""

from chuk_mcp_physics.models import ProjectileWithDragRequest
from chuk_mcp_physics.kinematics import calculate_projectile_with_drag


def baseball_curveball_vs_fastball():
    """Compare baseball curveball (topspin) vs fastball (backspin)."""
    print("\n" + "=" * 70)
    print("⚾ BASEBALL: Curveball vs Fastball (Spin Effects)")
    print("=" * 70)

    velocity = 35.0  # m/s (~78 mph curveball)

    # Fastball with backspin (Magnus force lifts ball)
    fastball = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=0,  # Level pitch
        mass=0.145,
        drag_coefficient=0.4,
        cross_sectional_area=0.0043,
        spin_rate=200.0,  # rad/s (~1900 rpm backspin)
        spin_axis=[0, 0, 1],  # Backspin (lifts)
    )

    # Curveball with topspin (Magnus force drops ball)
    curveball = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=0,  # Level pitch
        mass=0.145,
        drag_coefficient=0.4,
        cross_sectional_area=0.0043,
        spin_rate=200.0,  # rad/s (~1900 rpm topspin)
        spin_axis=[0, 0, -1],  # Topspin (drops)
    )

    result_fastball = calculate_projectile_with_drag(fastball)
    result_curveball = calculate_projectile_with_drag(curveball)

    print("\n📊 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s (78 mph)")
    print("   Angle: 0° (level pitch)")
    print("   Spin: 200 rad/s (~1900 rpm)")

    print("\n🔝 FASTBALL (Backspin - Magnus lifts):")
    print(f"   Max height: {result_fastball.max_height:.2f} m")
    print(f"   Range: {result_fastball.range:.1f} m")
    print(f"   Magnus force: {result_fastball.magnus_force_max:.2f} N (upward)")
    print(f"   Flight time: {result_fastball.time_of_flight:.3f} s")

    print("\n🔽 CURVEBALL (Topspin - Magnus drops):")
    print(f"   Max height: {result_curveball.max_height:.2f} m")
    print(f"   Range: {result_curveball.range:.1f} m")
    print(f"   Magnus force: {result_curveball.magnus_force_max:.2f} N (downward)")
    print(f"   Flight time: {result_curveball.time_of_flight:.3f} s")

    print("\n💡 SPIN EFFECT:")
    height_diff = result_fastball.max_height - result_curveball.max_height
    print(f"   Height difference: {height_diff:.2f} m ({height_diff * 39.37:.1f} inches)")
    print("   Backspin makes fastball 'rise' relative to curveball")


def golf_denver_vs_sea_level():
    """Compare golf drive at Denver (altitude) vs sea level."""
    print("\n" + "=" * 70)
    print("⛳ GOLF: Denver (1600m altitude) vs Sea Level")
    print("=" * 70)

    velocity = 70.0  # m/s (~155 mph)
    angle = 12.0

    # Sea level (standard conditions)
    sea_level = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        mass=0.0459,
        drag_coefficient=0.25,
        cross_sectional_area=0.00143,
        altitude=0.0,
        temperature=15.0,  # Standard temperature
        spin_rate=200.0,  # Backspin
        spin_axis=[0, 0, 1],
    )

    # Denver (high altitude = less air density)
    denver = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        mass=0.0459,
        drag_coefficient=0.25,
        cross_sectional_area=0.00143,
        altitude=1600.0,  # Denver elevation
        temperature=20.0,  # Warm summer day
        spin_rate=200.0,  # Same backspin
        spin_axis=[0, 0, 1],
    )

    result_sea = calculate_projectile_with_drag(sea_level)
    result_denver = calculate_projectile_with_drag(denver)

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s ({velocity * 2.237:.0f} mph)")
    print(f"   Angle: {angle}°")
    print("   Backspin: 200 rad/s")

    print("\n🌊 SEA LEVEL (0m altitude, 15°C):")
    print(f"   Air density: {result_sea.effective_air_density:.3f} kg/m³")
    print(f"   Range: {result_sea.range:.1f} m ({result_sea.range * 1.094:.1f} yards)")
    print(f"   Max height: {result_sea.max_height:.1f} m")
    print(f"   Energy lost to drag: {result_sea.energy_lost_to_drag:.1f} J")

    print("\n🏔️  DENVER (1600m altitude, 20°C):")
    print(f"   Air density: {result_denver.effective_air_density:.3f} kg/m³")
    print(f"   Range: {result_denver.range:.1f} m ({result_denver.range * 1.094:.1f} yards)")
    print(f"   Max height: {result_denver.max_height:.1f} m")
    print(f"   Energy lost to drag: {result_denver.energy_lost_to_drag:.1f} J")

    print("\n📊 ALTITUDE ADVANTAGE:")
    range_gain = result_denver.range - result_sea.range
    print(f"   Extra distance: {range_gain:.1f} m ({range_gain * 1.094:.1f} yards)")
    print(f"   Percentage gain: {(range_gain / result_sea.range) * 100:.1f}%")
    density_ratio = result_denver.effective_air_density / result_sea.effective_air_density
    print(f"   Air density ratio: {density_ratio:.2%}")
    print(f"   ⚠️  Balls fly ~{(1 - density_ratio) * 100:.1f}% farther in Denver!")


def soccer_free_kick_with_wind():
    """Soccer free kick with crosswind and sidespin."""
    print("\n" + "=" * 70)
    print("⚽ SOCCER: Free Kick with Crosswind + Sidespin")
    print("=" * 70)

    velocity = 28.0  # m/s (~63 mph)
    angle = 12.0

    # No wind, no spin
    basic = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        mass=0.43,
        drag_coefficient=0.25,
        cross_sectional_area=0.0388,
        wind_velocity=[0.0, 0.0],
        spin_rate=0.0,
    )

    # With crosswind
    with_wind = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        mass=0.43,
        drag_coefficient=0.25,
        cross_sectional_area=0.0388,
        wind_velocity=[5.0, 0.0],  # 5 m/s crosswind
        spin_rate=0.0,
    )

    # With sidespin (bending shot)
    with_spin = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        mass=0.43,
        drag_coefficient=0.25,
        cross_sectional_area=0.0388,
        wind_velocity=[0.0, 0.0],
        spin_rate=80.0,  # rad/s sidespin
        spin_axis=[0, 1, 0],  # Horizontal spin axis
    )

    # With BOTH wind and spin
    combined = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        mass=0.43,
        drag_coefficient=0.25,
        cross_sectional_area=0.0388,
        wind_velocity=[5.0, 0.0],  # Crosswind
        spin_rate=80.0,  # Sidespin
        spin_axis=[0, 1, 0],
    )

    result_basic = calculate_projectile_with_drag(basic)
    result_wind = calculate_projectile_with_drag(with_wind)
    result_spin = calculate_projectile_with_drag(with_spin)
    result_combined = calculate_projectile_with_drag(combined)

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s ({velocity * 2.237:.0f} mph)")
    print(f"   Angle: {angle}°")
    print("   Goal distance: ~25m (typical free kick)")

    print("\n⚪ BASIC (no wind, no spin):")
    print(f"   Range: {result_basic.range:.1f} m")
    print(f"   Lateral deflection: {result_basic.lateral_deflection:.2f} m")

    print("\n💨 WITH CROSSWIND (5 m/s, no spin):")
    print(f"   Range: {result_wind.range:.1f} m")
    print(f"   Wind drift: {result_wind.wind_drift:.2f} m")

    print("\n🌀 WITH SIDESPIN (80 rad/s, no wind):")
    print(f"   Range: {result_spin.range:.1f} m")
    print(f"   Lateral deflection: {result_spin.lateral_deflection:.2f} m (Magnus curve)")
    print(f"   Magnus force: {result_spin.magnus_force_max:.2f} N")

    print("\n🌪️  WITH BOTH (wind + spin):")
    print(f"   Range: {result_combined.range:.1f} m")
    print(f"   Total lateral movement: {result_combined.lateral_deflection:.2f} m")
    print(f"   Wind drift: {result_combined.wind_drift:.2f} m")
    print(f"   Magnus force: {result_combined.magnus_force_max:.2f} N")
    print("   💡 Combined effects can bend ball around wall!")


def tennis_serve_hot_vs_cold():
    """Tennis serve on hot day vs cold day (temperature affects air density)."""
    print("\n" + "=" * 70)
    print("🎾 TENNIS: Serve on Hot Day vs Cold Day")
    print("=" * 70)

    velocity = 55.0  # m/s (~123 mph pro serve)
    angle = 0.0  # Flat serve

    # Cold day
    cold_day = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=2.5,  # High contact point
        mass=0.058,  # Tennis ball
        drag_coefficient=0.55,
        cross_sectional_area=0.00338,  # π * (0.0328m)²
        temperature=5.0,  # Cold day (41°F)
        spin_rate=100.0,  # Topspin
        spin_axis=[0, 0, -1],
    )

    # Hot day
    hot_day = ProjectileWithDragRequest(
        initial_velocity=velocity,
        angle_degrees=angle,
        initial_height=2.5,
        mass=0.058,
        drag_coefficient=0.55,
        cross_sectional_area=0.00338,
        temperature=35.0,  # Hot day (95°F)
        spin_rate=100.0,  # Same topspin
        spin_axis=[0, 0, -1],
    )

    result_cold = calculate_projectile_with_drag(cold_day)
    result_hot = calculate_projectile_with_drag(hot_day)

    print("\n📍 Initial Conditions:")
    print(f"   Velocity: {velocity} m/s ({velocity * 2.237:.0f} mph)")
    print("   Height: 2.5 m (contact point)")
    print("   Topspin: 100 rad/s")

    print("\n❄️  COLD DAY (5°C / 41°F):")
    print(f"   Air density: {result_cold.effective_air_density:.3f} kg/m³")
    print(f"   Range: {result_cold.range:.1f} m")
    print(f"   Max height: {result_cold.max_height:.2f} m")
    print(f"   Energy lost: {result_cold.energy_lost_to_drag:.1f} J")

    print("\n🔥 HOT DAY (35°C / 95°F):")
    print(f"   Air density: {result_hot.effective_air_density:.3f} kg/m³")
    print(f"   Range: {result_hot.range:.1f} m")
    print(f"   Max height: {result_hot.max_height:.2f} m")
    print(f"   Energy lost: {result_hot.energy_lost_to_drag:.1f} J")

    print("\n📊 TEMPERATURE EFFECT:")
    range_diff = result_hot.range - result_cold.range
    print(f"   Extra distance (hot): {range_diff:.2f} m")
    print(f"   Percentage difference: {(range_diff / result_cold.range) * 100:.1f}%")
    print("   💡 Hot air = thinner air = less drag = faster serves!")


if __name__ == "__main__":
    print("\n" + "🌟 " * 20)
    print("ADVANCED PROJECTILE PHYSICS: Magnus Force, Wind & Altitude")
    print("🌟 " * 20)
    print("\nDemonstrating enhanced features:")
    print("  • Magnus force (spin effects on trajectory)")
    print("  • Wind effects (tailwind, headwind, crosswind)")
    print("  • Altitude effects (air density changes with elevation)")
    print("  • Temperature effects (air density changes with temperature)")

    baseball_curveball_vs_fastball()
    golf_denver_vs_sea_level()
    soccer_free_kick_with_wind()
    tennis_serve_hot_vs_cold()

    print("\n" + "=" * 70)
    print("💡 KEY TAKEAWAYS")
    print("=" * 70)
    print("1. Backspin creates lift (Magnus force), topspin creates drop")
    print("2. Golf balls fly ~10% farther in Denver due to thin air")
    print("3. Crosswind + sidespin combine for dramatic trajectory bends")
    print("4. Hot weather reduces air density = less drag = longer shots")
    print("5. These effects are crucial for realistic sports simulations!")
    print("\n✅ All effects computed with full 3D physics simulation (RK4)")
    print()
