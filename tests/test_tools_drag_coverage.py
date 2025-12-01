"""Tests for projectile with drag tool wrapper to improve coverage."""

import pytest
from chuk_mcp_physics.tools.kinematics_tools import calculate_projectile_with_drag


class TestProjectileDragToolWrapper:
    """Test the MCP tool wrapper for projectile with drag."""

    @pytest.mark.asyncio
    async def test_basic_drag_tool(self):
        """Test basic tool call without enhancements."""
        result = await calculate_projectile_with_drag(
            initial_velocity=30.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
        )

        assert "range" in result
        assert "max_height" in result
        assert result["range"] > 0
        assert result["max_height"] > 0

    @pytest.mark.asyncio
    async def test_tool_with_spin_json_string(self):
        """Test tool with spin parameters as JSON string."""
        result = await calculate_projectile_with_drag(
            initial_velocity=40.0,
            angle_degrees=15.0,
            mass=0.145,
            cross_sectional_area=0.0043,
            drag_coefficient=0.4,
            spin_rate=200.0,
            spin_axis="[0, 0, 1]",  # JSON string
        )

        assert result["magnus_force_max"] > 0
        assert result["range"] > 0

    @pytest.mark.asyncio
    async def test_tool_with_wind_json_string(self):
        """Test tool with wind as JSON string."""
        result = await calculate_projectile_with_drag(
            initial_velocity=30.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            wind_velocity="[5.0, 0.0]",  # JSON string
        )

        assert result["wind_drift"] != 0
        assert result["range"] > 0

    @pytest.mark.asyncio
    async def test_tool_with_spin_list(self):
        """Test tool with spin parameters as list."""
        result = await calculate_projectile_with_drag(
            initial_velocity=35.0,
            angle_degrees=10.0,
            mass=0.145,
            cross_sectional_area=0.0043,
            drag_coefficient=0.4,
            spin_rate=150.0,
            spin_axis=[0, 1, 0],  # List
        )

        assert result["magnus_force_max"] > 0
        assert result["lateral_deflection"] > 0

    @pytest.mark.asyncio
    async def test_tool_with_wind_list(self):
        """Test tool with wind as list."""
        result = await calculate_projectile_with_drag(
            initial_velocity=25.0,
            angle_degrees=30.0,
            mass=0.43,
            cross_sectional_area=0.0388,
            drag_coefficient=0.25,
            wind_velocity=[3.0, 0.0],  # List
        )

        assert result["wind_drift"] > 0

    @pytest.mark.asyncio
    async def test_tool_with_altitude(self):
        """Test tool with altitude parameter."""
        result = await calculate_projectile_with_drag(
            initial_velocity=70.0,
            angle_degrees=12.0,
            mass=0.0459,
            cross_sectional_area=0.00143,
            drag_coefficient=0.25,
            altitude=1600.0,  # Denver
            temperature=20.0,
        )

        assert result["effective_air_density"] < 1.225  # Less than sea level
        assert result["range"] > 0

    @pytest.mark.asyncio
    async def test_tool_with_all_enhancements(self):
        """Test tool with all enhancement parameters."""
        result = await calculate_projectile_with_drag(
            initial_velocity=50.0,
            angle_degrees=20.0,
            mass=0.5,
            cross_sectional_area=0.02,
            drag_coefficient=0.4,
            initial_height=2.0,
            spin_rate=100.0,
            spin_axis=[0, 0, 1],
            wind_velocity=[2.0, 0.5],
            altitude=500.0,
            temperature=25.0,
            time_step=0.01,
            max_time=30.0,
        )

        # Check all enhancement fields are present
        assert "magnus_force_max" in result
        assert "wind_drift" in result
        assert "lateral_deflection" in result
        assert "effective_air_density" in result
        assert result["magnus_force_max"] > 0
        assert result["effective_air_density"] < 1.225

    @pytest.mark.asyncio
    async def test_tool_custom_gravity(self):
        """Test tool with custom gravity."""
        result = await calculate_projectile_with_drag(
            initial_velocity=20.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            gravity=3.71,  # Mars gravity
        )

        assert result["range"] > 0
        assert result["time_of_flight"] > 0

    @pytest.mark.asyncio
    async def test_tool_different_fluid_density(self):
        """Test tool with different fluid density (water)."""
        result = await calculate_projectile_with_drag(
            initial_velocity=10.0,
            angle_degrees=30.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            fluid_density=1000.0,  # Water
        )

        # In water, drag is much higher
        assert result["range"] < 5.0
        assert result["energy_lost_to_drag"] > 0.5 * result["initial_kinetic_energy"]

    @pytest.mark.asyncio
    async def test_tool_high_temperature(self):
        """Test with high temperature."""
        result = await calculate_projectile_with_drag(
            initial_velocity=30.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            temperature=40.0,  # Hot day
        )

        assert result["effective_air_density"] < 1.225

    @pytest.mark.asyncio
    async def test_tool_low_temperature(self):
        """Test with low temperature."""
        result = await calculate_projectile_with_drag(
            initial_velocity=30.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            temperature=-10.0,  # Cold day
        )

        assert result["effective_air_density"] > 1.225

    @pytest.mark.asyncio
    async def test_tool_zero_spin(self):
        """Test with explicitly zero spin."""
        result = await calculate_projectile_with_drag(
            initial_velocity=30.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            spin_rate=0.0,
        )

        assert result["magnus_force_max"] == 0.0
        assert result["lateral_deflection"] == 0.0

    @pytest.mark.asyncio
    async def test_tool_zero_wind(self):
        """Test with explicitly zero wind."""
        result = await calculate_projectile_with_drag(
            initial_velocity=30.0,
            angle_degrees=45.0,
            mass=1.0,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            wind_velocity=[0.0, 0.0],
        )

        assert result["wind_drift"] == 0.0
