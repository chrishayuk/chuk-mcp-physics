"""Tests for analytic physics provider."""

import pytest
from chuk_mcp_physics.providers.analytic import AnalyticProvider
from chuk_mcp_physics.models import (
    ProjectileMotionRequest,
    CollisionCheckRequest,
    ForceCalculationRequest,
    KineticEnergyRequest,
    MomentumRequest,
    SimulationConfig,
    RigidBodyDefinition,
)


@pytest.fixture
def provider():
    """Create analytic provider instance."""
    return AnalyticProvider()


class TestProjectileMotion:
    """Test projectile motion calculations."""

    @pytest.mark.asyncio
    async def test_45_degree_maximum_range(self, provider, projectile_45deg):
        """45 degrees should give maximum range on flat ground."""
        request = ProjectileMotionRequest(**projectile_45deg)
        result = await provider.calculate_projectile_motion(request)

        # For 45° with v0=20 m/s, range = v0²/g
        expected_range = (20.0**2) / 9.81
        assert abs(result.range - expected_range) < 0.1

    @pytest.mark.asyncio
    async def test_90_degree_no_range(self, provider):
        """90 degrees (straight up) should give zero range."""
        request = ProjectileMotionRequest(
            initial_velocity=20.0,
            angle_degrees=90.0,
            initial_height=0.0,
            gravity=9.81,
        )
        result = await provider.calculate_projectile_motion(request)

        assert result.range < 0.1  # Essentially zero
        assert result.max_height > 0  # Should go up

    @pytest.mark.asyncio
    async def test_with_initial_height(self, provider):
        """Projectile from elevated position should have longer flight time."""
        request_ground = ProjectileMotionRequest(
            initial_velocity=20.0,
            angle_degrees=45.0,
            initial_height=0.0,
        )
        request_elevated = ProjectileMotionRequest(
            initial_velocity=20.0,
            angle_degrees=45.0,
            initial_height=10.0,
        )

        result_ground = await provider.calculate_projectile_motion(request_ground)
        result_elevated = await provider.calculate_projectile_motion(request_elevated)

        assert result_elevated.time_of_flight > result_ground.time_of_flight
        assert result_elevated.range > result_ground.range
        assert result_elevated.max_height > result_ground.max_height

    @pytest.mark.asyncio
    async def test_trajectory_points_count(self, provider, projectile_45deg):
        """Should return 50+ trajectory points."""
        request = ProjectileMotionRequest(**projectile_45deg)
        result = await provider.calculate_projectile_motion(request)

        assert len(result.trajectory_points) >= 50
        # First point should be at initial height
        assert result.trajectory_points[0][1] == pytest.approx(0.0, abs=0.1)
        # Last point should be near ground
        assert result.trajectory_points[-1][1] >= 0.0


class TestCollisionCheck:
    """Test collision detection."""

    @pytest.mark.asyncio
    async def test_head_on_collision(self, provider, collision_scenario):
        """Two objects moving toward each other should collide."""
        request = CollisionCheckRequest(**collision_scenario)
        result = await provider.check_collision(request)

        assert result.will_collide is True
        assert result.collision_time is not None
        assert 0 < result.collision_time < 2.0  # Should collide quickly
        assert result.impact_speed is not None
        assert result.impact_speed > 0

    @pytest.mark.asyncio
    async def test_no_collision(self, provider, no_collision_scenario):
        """Parallel objects should not collide."""
        request = CollisionCheckRequest(**no_collision_scenario)
        result = await provider.check_collision(request)

        assert result.will_collide is False
        assert result.collision_time is None
        assert result.closest_approach_distance > 0

    @pytest.mark.asyncio
    async def test_stationary_overlap(self, provider):
        """Already overlapping stationary objects."""
        request = CollisionCheckRequest(
            body1_position=[0.0, 0.0, 0.0],
            body1_velocity=[0.0, 0.0, 0.0],
            body1_radius=1.0,
            body2_position=[1.0, 0.0, 0.0],  # 1m apart, but combined radius = 2m
            body2_velocity=[0.0, 0.0, 0.0],
            body2_radius=1.0,
        )
        result = await provider.check_collision(request)

        assert result.will_collide is True
        assert result.collision_time == 0.0

    @pytest.mark.asyncio
    async def test_closest_approach(self, provider, no_collision_scenario):
        """Check closest approach calculation."""
        request = CollisionCheckRequest(**no_collision_scenario)
        result = await provider.check_collision(request)

        assert result.closest_approach_distance > 0
        assert result.closest_approach_time >= 0


class TestForceCalculation:
    """Test force calculations (F = ma)."""

    @pytest.mark.asyncio
    async def test_simple_force(self, provider):
        """F = ma with simple values."""
        request = ForceCalculationRequest(
            mass=10.0,
            acceleration=[2.0, 0.0, 0.0],
        )
        result = await provider.calculate_force(request)

        assert result.force == [20.0, 0.0, 0.0]
        assert result.magnitude == pytest.approx(20.0)

    @pytest.mark.asyncio
    async def test_force_magnitude(self, provider):
        """Force magnitude should be correct for 3D acceleration."""
        request = ForceCalculationRequest(
            mass=5.0,
            acceleration=[3.0, 4.0, 0.0],  # 3-4-5 triangle
        )
        result = await provider.calculate_force(request)

        expected_magnitude = 5.0 * 5.0  # mass × |a| where |a| = 5
        assert result.magnitude == pytest.approx(expected_magnitude)


class TestKineticEnergy:
    """Test kinetic energy calculations."""

    @pytest.mark.asyncio
    async def test_simple_kinetic_energy(self, provider):
        """KE = 0.5 * m * v²."""
        request = KineticEnergyRequest(
            mass=10.0,
            velocity=[10.0, 0.0, 0.0],
        )
        result = await provider.calculate_kinetic_energy(request)

        expected_ke = 0.5 * 10.0 * 10.0**2
        assert result.kinetic_energy == pytest.approx(expected_ke)
        assert result.speed == pytest.approx(10.0)

    @pytest.mark.asyncio
    async def test_3d_velocity(self, provider):
        """Kinetic energy with 3D velocity."""
        request = KineticEnergyRequest(
            mass=2.0,
            velocity=[3.0, 4.0, 0.0],  # Speed = 5 m/s
        )
        result = await provider.calculate_kinetic_energy(request)

        expected_ke = 0.5 * 2.0 * 5.0**2
        assert result.kinetic_energy == pytest.approx(expected_ke)
        assert result.speed == pytest.approx(5.0)


class TestMomentum:
    """Test momentum calculations."""

    @pytest.mark.asyncio
    async def test_simple_momentum(self, provider):
        """p = mv."""
        request = MomentumRequest(
            mass=5.0,
            velocity=[10.0, 0.0, 0.0],
        )
        result = await provider.calculate_momentum(request)

        assert result.momentum == [50.0, 0.0, 0.0]
        assert result.magnitude == pytest.approx(50.0)

    @pytest.mark.asyncio
    async def test_3d_momentum(self, provider):
        """Momentum magnitude with 3D velocity."""
        request = MomentumRequest(
            mass=2.0,
            velocity=[3.0, 4.0, 0.0],
        )
        result = await provider.calculate_momentum(request)

        expected_magnitude = 2.0 * 5.0  # mass × speed
        assert result.magnitude == pytest.approx(expected_magnitude)


class TestSimulationNotSupported:
    """Test that simulation methods raise NotImplementedError."""

    @pytest.mark.asyncio
    async def test_create_simulation_not_supported(self, provider):
        """Analytic provider doesn't support simulations."""
        config = SimulationConfig()
        with pytest.raises(NotImplementedError, match="Analytic provider"):
            await provider.create_simulation(config)

    @pytest.mark.asyncio
    async def test_add_body_not_supported(self, provider):
        """Analytic provider doesn't support adding bodies."""
        body = RigidBodyDefinition(id="test", shape="box", size=[1, 1, 1])
        with pytest.raises(NotImplementedError, match="Analytic provider"):
            await provider.add_body("sim_id", body)

    @pytest.mark.asyncio
    async def test_step_simulation_not_supported(self, provider):
        """Analytic provider doesn't support stepping."""
        with pytest.raises(NotImplementedError, match="Analytic provider"):
            await provider.step_simulation("sim_id")
