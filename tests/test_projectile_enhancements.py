"""Tests for enhanced projectile motion features (Magnus force, wind, altitude)."""

from chuk_mcp_physics.models import ProjectileWithDragRequest
from chuk_mcp_physics.kinematics import calculate_projectile_with_drag


class TestMagnusForce:
    """Test Magnus force (spin effects) on projectiles."""

    def test_backspin_increases_range(self):
        """Test that backspin increases range (like baseball/golf)."""
        # Baseball with no spin
        no_spin = ProjectileWithDragRequest(
            initial_velocity=40.23,
            angle_degrees=15,
            mass=0.145,
            drag_coefficient=0.4,
            cross_sectional_area=0.0043,
            spin_rate=0.0,
        )

        # Baseball with backspin (2500 rpm = 261.8 rad/s)
        with_backspin = ProjectileWithDragRequest(
            initial_velocity=40.23,
            angle_degrees=15,
            mass=0.145,
            drag_coefficient=0.4,
            cross_sectional_area=0.0043,
            spin_rate=261.8,  # 2500 rpm
            spin_axis=[0, 0, 1],  # Backspin (z-axis perpendicular to motion)
        )

        result_no_spin = calculate_projectile_with_drag(no_spin)
        result_backspin = calculate_projectile_with_drag(with_backspin)

        # Backspin should increase range and max height
        assert result_backspin.range > result_no_spin.range
        assert result_backspin.max_height > result_no_spin.max_height
        assert result_backspin.magnus_force_max > 0

    def test_sidespin_lateral_deflection(self):
        """Test that sidespin causes lateral deflection."""
        # Soccer ball with sidespin
        with_sidespin = ProjectileWithDragRequest(
            initial_velocity=25.0,
            angle_degrees=15,
            mass=0.43,
            drag_coefficient=0.25,
            cross_sectional_area=0.0388,
            spin_rate=100.0,  # rad/s
            spin_axis=[0, 1, 0],  # Horizontal spin axis
        )

        result = calculate_projectile_with_drag(with_sidespin)

        # Should have lateral deflection
        assert result.lateral_deflection > 0
        assert result.magnus_force_max > 0

    def test_no_spin_no_magnus(self):
        """Test that zero spin produces no Magnus force."""
        no_spin = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            spin_rate=0.0,
        )

        result = calculate_projectile_with_drag(no_spin)

        # No spin = no Magnus force = no lateral deflection
        assert result.magnus_force_max == 0.0
        assert result.lateral_deflection == 0.0


class TestWindEffects:
    """Test wind effects on projectile motion."""

    def test_tailwind_increases_range(self):
        """Test that tailwind increases range."""
        # No wind
        no_wind = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            wind_velocity=[0.0, 0.0],
        )

        # 10 m/s tailwind
        tailwind = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            wind_velocity=[10.0, 0.0],
        )

        result_no_wind = calculate_projectile_with_drag(no_wind)
        result_tailwind = calculate_projectile_with_drag(tailwind)

        # Tailwind should increase range
        assert result_tailwind.range > result_no_wind.range
        assert result_tailwind.wind_drift > 0

    def test_headwind_decreases_range(self):
        """Test that headwind decreases range."""
        # No wind
        no_wind = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            wind_velocity=[0.0, 0.0],
        )

        # 10 m/s headwind
        headwind = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            wind_velocity=[-10.0, 0.0],
        )

        result_no_wind = calculate_projectile_with_drag(no_wind)
        result_headwind = calculate_projectile_with_drag(headwind)

        # Headwind should decrease range
        assert result_headwind.range < result_no_wind.range
        assert result_headwind.wind_drift < 0

    def test_updraft_increases_height(self):
        """Test that updraft increases max height."""
        # No wind
        no_wind = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            wind_velocity=[0.0, 0.0],
        )

        # 5 m/s updraft
        updraft = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            wind_velocity=[0.0, 5.0],
        )

        result_no_wind = calculate_projectile_with_drag(no_wind)
        result_updraft = calculate_projectile_with_drag(updraft)

        # Updraft should increase max height and time of flight
        assert result_updraft.max_height > result_no_wind.max_height
        assert result_updraft.time_of_flight > result_no_wind.time_of_flight


class TestAltitudeEffects:
    """Test altitude and temperature effects on air density."""

    def test_high_altitude_less_drag(self):
        """Test that high altitude reduces air density and drag."""
        # Sea level
        sea_level = ProjectileWithDragRequest(
            initial_velocity=70.0,
            angle_degrees=12,
            mass=0.0459,  # Golf ball
            drag_coefficient=0.25,
            cross_sectional_area=0.00143,
            altitude=0.0,
            temperature=15.0,
        )

        # Denver altitude (1600m)
        high_altitude = ProjectileWithDragRequest(
            initial_velocity=70.0,
            angle_degrees=12,
            mass=0.0459,
            drag_coefficient=0.25,
            cross_sectional_area=0.00143,
            altitude=1600.0,
            temperature=15.0,
        )

        result_sea_level = calculate_projectile_with_drag(sea_level)
        result_high_altitude = calculate_projectile_with_drag(high_altitude)

        # High altitude = less air density = less drag = longer range
        assert result_high_altitude.effective_air_density < result_sea_level.effective_air_density
        assert result_high_altitude.range > result_sea_level.range
        assert result_high_altitude.energy_lost_to_drag < result_sea_level.energy_lost_to_drag

    def test_hot_temperature_less_drag(self):
        """Test that hot temperature reduces air density."""
        # Cold day
        cold = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            altitude=0.0,
            temperature=0.0,  # 0°C
        )

        # Hot day
        hot = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            altitude=0.0,
            temperature=30.0,  # 30°C
        )

        result_cold = calculate_projectile_with_drag(cold)
        result_hot = calculate_projectile_with_drag(hot)

        # Hot air = less dense = less drag = longer range
        assert result_hot.effective_air_density < result_cold.effective_air_density
        assert result_hot.range > result_cold.range

    def test_effective_density_calculation(self):
        """Test that effective air density is calculated correctly."""
        # Sea level, 15°C should be close to standard 1.225 kg/m³
        standard = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            altitude=0.0,
            temperature=15.0,
        )

        result = calculate_projectile_with_drag(standard)

        # Should be very close to standard air density
        assert abs(result.effective_air_density - 1.225) < 0.1


class TestCombinedEffects:
    """Test combinations of enhancements."""

    def test_golf_ball_with_all_effects(self):
        """Test golf ball with backspin, wind, and altitude."""
        # Golf drive with all effects
        golf = ProjectileWithDragRequest(
            initial_velocity=70.0,
            angle_degrees=12,
            mass=0.0459,
            drag_coefficient=0.25,
            cross_sectional_area=0.00143,
            spin_rate=200.0,  # Backspin
            spin_axis=[0, 0, 1],
            wind_velocity=[3.0, 0.0],  # Tailwind
            altitude=1000.0,  # Moderate altitude
            temperature=25.0,  # Warm day
        )

        result = calculate_projectile_with_drag(golf)

        # All effects should contribute to longer range
        assert result.range > 140  # meters
        assert result.magnus_force_max > 0
        assert result.wind_drift > 0
        assert result.effective_air_density < 1.225

    def test_baseball_curveball(self):
        """Test baseball curveball with spin and realistic conditions."""
        # Curveball with topspin (drops faster)
        curveball = ProjectileWithDragRequest(
            initial_velocity=35.0,  # 78 mph
            angle_degrees=5,
            mass=0.145,
            drag_coefficient=0.4,
            cross_sectional_area=0.0043,
            spin_rate=200.0,  # Topspin
            spin_axis=[0, 0, -1],  # Negative z = topspin
            temperature=20.0,
        )

        result = calculate_projectile_with_drag(curveball)

        # Should have significant Magnus force
        assert result.magnus_force_max > 0
        # Range should be reasonable for curveball
        assert 10 < result.range < 20
