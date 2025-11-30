"""Tests for MCP server tools."""

import pytest
from chuk_mcp_physics.server import (
    calculate_projectile_motion,
    check_collision,
    calculate_force,
    calculate_kinetic_energy,
    calculate_momentum,
)


class TestProjectileMotionTool:
    """Test projectile motion MCP tool."""

    @pytest.mark.asyncio
    async def test_simple_projectile(self):
        """Test basic projectile motion."""
        result = await calculate_projectile_motion(
            initial_velocity=20.0, angle_degrees=45.0, initial_height=0.0, gravity=9.81
        )

        assert result.max_height > 0
        assert result.range > 0
        assert result.time_of_flight > 0
        assert len(result.trajectory_points) > 0

    @pytest.mark.asyncio
    async def test_projectile_with_defaults(self):
        """Test with default parameters."""
        result = await calculate_projectile_motion(initial_velocity=20.0, angle_degrees=45.0)

        # Should use default gravity and height
        assert result.max_height > 0

    @pytest.mark.asyncio
    async def test_vertical_throw(self):
        """Test 90 degree throw."""
        result = await calculate_projectile_motion(
            initial_velocity=20.0, angle_degrees=90.0, initial_height=0.0
        )

        # Should go straight up with minimal range
        assert result.range < 1.0
        assert result.max_height > 0


class TestCollisionTool:
    """Test collision detection MCP tool."""

    @pytest.mark.asyncio
    async def test_head_on_collision(self):
        """Test head-on collision."""
        result = await check_collision(
            body1_position=[0.0, 0.0, 0.0],
            body1_velocity=[10.0, 0.0, 0.0],
            body1_radius=1.0,
            body2_position=[20.0, 0.0, 0.0],
            body2_velocity=[-10.0, 0.0, 0.0],
            body2_radius=1.0,
        )

        assert result.will_collide is True
        assert result.collision_time is not None
        assert result.impact_speed is not None

    @pytest.mark.asyncio
    async def test_no_collision(self):
        """Test no collision scenario."""
        result = await check_collision(
            body1_position=[0.0, 0.0, 0.0],
            body1_velocity=[10.0, 0.0, 0.0],
            body1_radius=1.0,
            body2_position=[0.0, 100.0, 0.0],
            body2_velocity=[10.0, 0.0, 0.0],
            body2_radius=1.0,
        )

        assert result.will_collide is False
        assert result.collision_time is None


class TestForceTool:
    """Test force calculation MCP tool."""

    @pytest.mark.asyncio
    async def test_simple_force(self):
        """Test F = ma."""
        result = await calculate_force(
            mass=10.0, acceleration_x=2.0, acceleration_y=0.0, acceleration_z=0.0
        )

        assert result.force[0] == 20.0
        assert result.magnitude == 20.0


class TestKineticEnergyTool:
    """Test kinetic energy MCP tool."""

    @pytest.mark.asyncio
    async def test_simple_kinetic_energy(self):
        """Test KE = 0.5 * m * v²."""
        result = await calculate_kinetic_energy(
            mass=10.0, velocity_x=10.0, velocity_y=0.0, velocity_z=0.0
        )

        expected_ke = 0.5 * 10.0 * 10.0**2
        assert result.kinetic_energy == pytest.approx(expected_ke)
        assert result.speed == 10.0


class TestMomentumTool:
    """Test momentum MCP tool."""

    @pytest.mark.asyncio
    async def test_simple_momentum(self):
        """Test p = mv."""
        result = await calculate_momentum(mass=5.0, velocity_x=10.0, velocity_y=0.0, velocity_z=0.0)

        assert result.momentum == [50.0, 0.0, 0.0]
        assert result.magnitude == pytest.approx(50.0)

    @pytest.mark.asyncio
    async def test_3d_momentum(self):
        """Test 3D momentum."""
        result = await calculate_momentum(mass=2.0, velocity_x=3.0, velocity_y=4.0, velocity_z=0.0)

        # Magnitude should be 2 * sqrt(3² + 4²) = 2 * 5 = 10
        assert result.magnitude == pytest.approx(10.0)
