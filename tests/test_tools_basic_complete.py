"""Comprehensive tests for tools/basic.py module."""

import json
import pytest


class TestBasicToolsComplete:
    """Complete test coverage for basic physics tools."""

    @pytest.mark.asyncio
    async def test_calculate_potential_energy_direct(self):
        """Test potential energy calculation directly."""
        from chuk_mcp_physics.tools.basic import calculate_potential_energy

        result = await calculate_potential_energy(mass=10.0, height=5.0, gravity=9.81)

        assert result["potential_energy"] > 0
        assert result["potential_energy"] == pytest.approx(490.5, rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_work_power_with_json_strings(self):
        """Test work/power with JSON string inputs."""
        from chuk_mcp_physics.tools.basic import calculate_work_power

        result = await calculate_work_power(
            force=json.dumps([10.0, 0.0, 0.0]),
            displacement=json.dumps([5.0, 0.0, 0.0]),
            time=2.0,
        )

        assert result["work"] == 50.0
        assert result["power"] == 25.0

    @pytest.mark.asyncio
    async def test_calculate_work_power_without_time(self):
        """Test work calculation without time (no power)."""
        from chuk_mcp_physics.tools.basic import calculate_work_power

        result = await calculate_work_power(
            force=[10.0, 0.0, 0.0], displacement=[5.0, 0.0, 0.0], time=None
        )

        assert result["work"] == 50.0
        assert result["power"] is None

    @pytest.mark.asyncio
    async def test_calculate_elastic_collision_direct(self):
        """Test elastic collision calculation."""
        from chuk_mcp_physics.tools.basic import calculate_elastic_collision

        result = await calculate_elastic_collision(
            mass1=1.0, velocity1=10.0, mass2=1.0, velocity2=0.0
        )

        assert result["final_velocity1"] is not None
        assert result["final_velocity2"] is not None
        # For equal masses, velocities should be exchanged
        assert abs(result["final_velocity1"]) < 0.1
        assert abs(result["final_velocity2"] - 10.0) < 0.1
