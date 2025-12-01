"""Comprehensive tests for tools/fluid.py module."""

import json
import pytest


class TestFluidToolsComplete:
    """Complete test coverage for fluid dynamics tools."""

    @pytest.mark.asyncio
    async def test_calculate_drag_force_direct(self):
        """Test drag force calculation with list input."""
        from chuk_mcp_physics.tools.fluid import calculate_drag_force

        result = await calculate_drag_force(
            velocity=[0, -5.0, 0],
            cross_sectional_area=0.00785,
            fluid_density=1000,
            drag_coefficient=0.47,
        )

        assert "drag_force" in result
        assert "magnitude" in result
        assert result["magnitude"] > 0

    @pytest.mark.asyncio
    async def test_calculate_drag_force_with_json_string(self):
        """Test drag force with JSON string velocity."""
        from chuk_mcp_physics.tools.fluid import calculate_drag_force

        result = await calculate_drag_force(
            velocity=json.dumps([10.0, 0.0, 0.0]),
            cross_sectional_area=0.5,
            fluid_density=1.225,
            drag_coefficient=0.47,
        )

        assert "drag_force" in result
        assert "magnitude" in result

    @pytest.mark.asyncio
    async def test_calculate_buoyancy_full_submerge(self):
        """Test buoyancy calculation fully submerged."""
        from chuk_mcp_physics.tools.fluid import calculate_buoyancy

        result = await calculate_buoyancy(
            volume=0.001,
            fluid_density=1000,
            gravity=9.81,
            submerged_fraction=1.0,
        )

        assert "buoyant_force" in result
        assert "displaced_mass" in result
        assert result["buoyant_force"] == pytest.approx(9.81, rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_buoyancy_partial_submerge(self):
        """Test buoyancy calculation partially submerged."""
        from chuk_mcp_physics.tools.fluid import calculate_buoyancy

        result = await calculate_buoyancy(
            volume=0.001,
            fluid_density=1000,
            gravity=9.81,
            submerged_fraction=0.5,
        )

        assert "buoyant_force" in result
        assert result["buoyant_force"] == pytest.approx(4.905, rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_terminal_velocity(self):
        """Test terminal velocity calculation."""
        from chuk_mcp_physics.tools.fluid import calculate_terminal_velocity

        result = await calculate_terminal_velocity(
            mass=70,
            cross_sectional_area=0.7,
            fluid_density=1.225,
            drag_coefficient=1.0,
            gravity=9.81,
        )

        assert "terminal_velocity" in result
        assert result["terminal_velocity"] > 0
        # Terminal velocity should be around 40 m/s
        assert 35 < result["terminal_velocity"] < 45

    @pytest.mark.asyncio
    async def test_simulate_underwater_motion_direct(self):
        """Test underwater motion simulation with lists."""
        from chuk_mcp_physics.tools.fluid import simulate_underwater_motion

        result = await simulate_underwater_motion(
            initial_velocity=[5.0, 0.0, 0.0],
            mass=10.0,
            volume=0.01,
            cross_sectional_area=0.05,
            fluid_density=1000,
            fluid_viscosity=1.002e-3,
            initial_position=[0.0, 0.0, 0.0],
            drag_coefficient=0.47,
            gravity=9.81,
            duration=2.0,
            dt=0.1,
        )

        assert "trajectory" in result
        assert "final_position" in result
        assert "final_velocity" in result

    @pytest.mark.asyncio
    async def test_simulate_underwater_motion_with_json_strings(self):
        """Test underwater motion with JSON string inputs."""
        from chuk_mcp_physics.tools.fluid import simulate_underwater_motion

        result = await simulate_underwater_motion(
            initial_velocity=json.dumps([5.0, 0.0, 0.0]),
            mass=10.0,
            volume=0.01,
            cross_sectional_area=0.05,
            initial_position=json.dumps([0.0, 0.0, 0.0]),
        )

        assert "trajectory" in result
        assert "final_position" in result

    @pytest.mark.asyncio
    async def test_calculate_lift_force(self):
        """Test lift force calculation."""
        from chuk_mcp_physics.tools.fluid import calculate_lift_force

        result = await calculate_lift_force(
            velocity=70,
            wing_area=20.0,
            lift_coefficient=1.2,
            fluid_density=1.225,
        )

        assert "lift_force" in result
        assert "dynamic_pressure" in result
        assert result["lift_force"] > 0

    @pytest.mark.asyncio
    async def test_calculate_magnus_force_with_lists(self):
        """Test Magnus force with list inputs."""
        from chuk_mcp_physics.tools.fluid import calculate_magnus_force

        result = await calculate_magnus_force(
            velocity=[20.0, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 50.0],
            radius=0.11,
            fluid_density=1.225,
        )

        assert "magnus_force" in result
        assert "magnus_force_magnitude" in result
        assert "spin_rate" in result

    @pytest.mark.asyncio
    async def test_calculate_magnus_force_with_json_strings(self):
        """Test Magnus force with JSON string inputs."""
        from chuk_mcp_physics.tools.fluid import calculate_magnus_force

        result = await calculate_magnus_force(
            velocity=json.dumps([20.0, 0.0, 0.0]),
            angular_velocity=json.dumps([0.0, 0.0, 50.0]),
            radius=0.11,
            fluid_density=1.225,
        )

        assert "magnus_force" in result
        assert "spin_rate" in result

    @pytest.mark.asyncio
    async def test_calculate_bernoulli_single_point(self):
        """Test Bernoulli equation at single point."""
        from chuk_mcp_physics.tools.fluid import calculate_bernoulli

        result = await calculate_bernoulli(
            pressure1=101325,
            velocity1=0,
            height1=10,
            fluid_density=1000,
            gravity=9.81,
        )

        assert "total_pressure_1" in result
        assert "static_pressure_1" in result
        assert "dynamic_pressure_1" in result

    @pytest.mark.asyncio
    async def test_calculate_bernoulli_two_points(self):
        """Test Bernoulli equation between two points."""
        from chuk_mcp_physics.tools.fluid import calculate_bernoulli

        result = await calculate_bernoulli(
            pressure1=101325,
            velocity1=0,
            height1=10,
            velocity2=14,
            height2=0,
            fluid_density=1000,
            gravity=9.81,
        )

        assert "total_pressure_1" in result
        assert "pressure2" in result

    @pytest.mark.asyncio
    async def test_calculate_pressure_at_depth(self):
        """Test pressure at depth calculation."""
        from chuk_mcp_physics.tools.fluid import calculate_pressure_at_depth

        result = await calculate_pressure_at_depth(
            depth=30,
            fluid_density=1025,
            atmospheric_pressure=101325,
            gravity=9.81,
        )

        assert "total_pressure" in result
        assert "gauge_pressure" in result
        assert "pressure_atmospheres" in result
        assert result["pressure_atmospheres"] > 1

    @pytest.mark.asyncio
    async def test_calculate_reynolds_number(self):
        """Test Reynolds number calculation."""
        from chuk_mcp_physics.tools.fluid import calculate_reynolds_number

        result = await calculate_reynolds_number(
            velocity=2.0,
            characteristic_length=0.05,
            fluid_density=1000,
            dynamic_viscosity=0.001,
        )

        assert "reynolds_number" in result
        assert "flow_regime" in result
        assert result["reynolds_number"] > 0

    @pytest.mark.asyncio
    async def test_calculate_venturi_effect(self):
        """Test Venturi effect calculation."""
        from chuk_mcp_physics.tools.fluid import calculate_venturi_effect

        result = await calculate_venturi_effect(
            inlet_diameter=0.1,
            throat_diameter=0.05,
            inlet_velocity=2.0,
            fluid_density=1000,
        )

        assert "throat_velocity" in result
        assert "pressure_drop" in result
        assert "flow_rate" in result
        assert result["throat_velocity"] > 2.0
