"""Tests for projectile motion with drag calculations."""

import math
from chuk_mcp_physics.models import ProjectileWithDragRequest
from chuk_mcp_physics.kinematics import calculate_projectile_with_drag


class TestProjectileWithDragBasic:
    """Test basic projectile motion with drag."""

    def test_baseball_pitch(self):
        """Test baseball pitch (90 mph fastball)."""
        # 90 mph = 40.23 m/s
        request = ProjectileWithDragRequest(
            initial_velocity=40.23,
            angle_degrees=10.0,
            initial_height=1.8,
            mass=0.145,  # kg
            drag_coefficient=0.4,  # baseball
            cross_sectional_area=0.0043,  # π * (0.037m)²
            fluid_density=1.225,  # air
        )

        result = calculate_projectile_with_drag(request)

        # Baseball with drag should travel realistically
        # A 90 mph fastball with 10° angle travels ~50-60m
        assert result.range < 70  # meters (realistic with drag)
        assert result.range > 40  # meters (sanity check)
        assert result.time_of_flight > 0.5  # seconds
        assert result.time_of_flight < 3.0  # seconds
        assert result.max_height > 1.8  # above release height
        assert result.energy_lost_to_drag > 0  # energy dissipated
        assert len(result.trajectory_points) > 10  # has trajectory data

    def test_golf_drive(self):
        """Test golf ball drive."""
        # Pro golfer drive: 70 m/s (~155 mph)
        request = ProjectileWithDragRequest(
            initial_velocity=70.0,
            angle_degrees=12.0,
            initial_height=0.0,
            mass=0.0459,  # kg (golf ball)
            drag_coefficient=0.25,  # dimpled golf ball
            cross_sectional_area=0.00143,  # π * (0.0213m)²
            fluid_density=1.225,  # air
        )

        result = calculate_projectile_with_drag(request)

        # Golf ball with drag: ~120-150m realistic at 12° launch angle
        # (Professional drives use higher angles and spin for max distance)
        assert result.range > 100  # meters
        assert result.range < 180  # meters
        assert result.max_height > 6  # meters (12° is low angle)
        assert result.max_height < 15  # meters
        assert result.time_of_flight > 2  # seconds (low angle = less flight time)
        assert result.energy_lost_to_drag > 50  # significant energy loss

    def test_basketball_shot(self):
        """Test basketball 3-pointer."""
        request = ProjectileWithDragRequest(
            initial_velocity=7.5,
            angle_degrees=48.0,
            initial_height=2.0,
            mass=0.624,  # kg (basketball)
            drag_coefficient=0.55,  # basketball
            cross_sectional_area=0.0456,  # π * (0.12m)²
            fluid_density=1.225,  # air
        )

        result = calculate_projectile_with_drag(request)

        # 3-pointer is ~6.75m from hoop
        assert result.range > 5  # meters
        assert result.range < 10  # meters
        assert result.max_height > 2  # above release
        assert result.max_height < 6  # realistic arc

    def test_zero_drag_coefficient(self):
        """Test that zero drag gives vacuum-like trajectory."""
        request = ProjectileWithDragRequest(
            initial_velocity=20.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.0,  # No drag!
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        result = calculate_projectile_with_drag(request)

        # With no drag, should approach ideal projectile motion
        # R = v² sin(2θ) / g = 400 * 1.0 / 9.81 ≈ 40.8m
        expected_range = (20.0**2) * math.sin(math.radians(90)) / 9.81
        assert abs(result.range - expected_range) < 1.0  # Close to ideal

        # Energy should be conserved (minus potential energy change)
        # Since starts and ends at same height, KE should be same
        assert abs(result.initial_kinetic_energy - result.final_kinetic_energy) < 5.0

    def test_energy_dissipation(self):
        """Test that drag dissipates energy."""
        request = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=0.5,
            drag_coefficient=0.47,  # sphere
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        result = calculate_projectile_with_drag(request)

        # Energy should decrease due to drag
        assert result.energy_lost_to_drag > 0
        assert result.final_kinetic_energy < result.initial_kinetic_energy
        assert (
            result.initial_kinetic_energy
            == result.final_kinetic_energy + result.energy_lost_to_drag
        )

    def test_impact_velocity_and_angle(self):
        """Test impact velocity and angle calculations."""
        request = ProjectileWithDragRequest(
            initial_velocity=20.0,
            angle_degrees=30.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.4,
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        result = calculate_projectile_with_drag(request)

        # Impact velocity should be less than initial (energy lost)
        assert result.impact_velocity < 20.0
        assert result.impact_velocity > 0

        # Impact angle should be reasonable (0-90 degrees)
        assert result.impact_angle >= 0
        assert result.impact_angle <= 90

    def test_high_drag_short_range(self):
        """Test that high drag dramatically reduces range."""
        # Same initial conditions, different drag
        base_request = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.1,  # low drag
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        high_drag_request = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=2.0,  # high drag
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        low_drag_result = calculate_projectile_with_drag(base_request)
        high_drag_result = calculate_projectile_with_drag(high_drag_request)

        # High drag should give much shorter range
        assert high_drag_result.range < low_drag_result.range * 0.7

    def test_trajectory_points_populated(self):
        """Test that trajectory points are generated."""
        request = ProjectileWithDragRequest(
            initial_velocity=20.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        result = calculate_projectile_with_drag(request)

        # Should have many trajectory points
        assert len(result.trajectory_points) > 20

        # First point should be at origin
        assert result.trajectory_points[0][0] == 0.0
        assert result.trajectory_points[0][1] == 0.0

        # Last point should be on ground (y ≈ 0)
        assert abs(result.trajectory_points[-1][1]) < 0.5

        # X should increase monotonically
        for i in range(len(result.trajectory_points) - 1):
            assert result.trajectory_points[i + 1][0] >= result.trajectory_points[i][0]


class TestProjectileWithDragEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_very_small_time_step(self):
        """Test with small time step (higher accuracy)."""
        request = ProjectileWithDragRequest(
            initial_velocity=20.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            fluid_density=1.225,
            time_step=0.001,  # Very small
        )

        result = calculate_projectile_with_drag(request)
        assert result.range > 0

    def test_water_density(self):
        """Test projectile motion through water."""
        request = ProjectileWithDragRequest(
            initial_velocity=10.0,
            angle_degrees=30.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            fluid_density=1000.0,  # water!
        )

        result = calculate_projectile_with_drag(request)

        # Should have much shorter range in water
        assert result.range < 5.0  # meters
        # Most energy lost to drag
        assert result.energy_lost_to_drag > 0.9 * result.initial_kinetic_energy

    def test_initial_height_effect(self):
        """Test that initial height increases range."""
        ground_request = ProjectileWithDragRequest(
            initial_velocity=20.0,
            angle_degrees=30.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        elevated_request = ProjectileWithDragRequest(
            initial_velocity=20.0,
            angle_degrees=30.0,
            initial_height=10.0,  # 10m high
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        ground_result = calculate_projectile_with_drag(ground_request)
        elevated_result = calculate_projectile_with_drag(elevated_request)

        # Higher start should give longer range
        assert elevated_result.range > ground_result.range
        assert elevated_result.time_of_flight > ground_result.time_of_flight


class TestProjectileWithDragComparison:
    """Compare drag vs no-drag trajectories."""

    def test_drag_vs_nodrag_range(self):
        """Compare range with and without drag."""
        with_drag = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.47,
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        no_drag = ProjectileWithDragRequest(
            initial_velocity=30.0,
            angle_degrees=45.0,
            initial_height=0.0,
            mass=1.0,
            drag_coefficient=0.0,  # No drag
            cross_sectional_area=0.01,
            fluid_density=1.225,
        )

        drag_result = calculate_projectile_with_drag(with_drag)
        nodrag_result = calculate_projectile_with_drag(no_drag)

        # Drag should reduce range
        assert drag_result.range < nodrag_result.range * 0.95
        # More energy lost with drag
        assert drag_result.energy_lost_to_drag > nodrag_result.energy_lost_to_drag
