"""Comprehensive tests for tools/kinematics_tools.py module."""

import json
import pytest


class TestKinematicsToolsComplete:
    """Complete test coverage for kinematics tools."""

    @pytest.mark.asyncio
    async def test_calculate_acceleration_from_position_with_lists(self):
        """Test acceleration calculation from position data."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_acceleration_from_position

        result = await calculate_acceleration_from_position(
            times=[0, 1, 2, 3, 4],
            positions=[[0, 0, 0], [5, 0, 0], [10, 0, 0], [15, 0, 0], [20, 0, 0]],
        )

        assert "velocities" in result
        assert "accelerations" in result
        assert "average_velocity" in result
        assert "average_acceleration" in result

    @pytest.mark.asyncio
    async def test_calculate_acceleration_from_position_with_json_strings(self):
        """Test acceleration calculation with JSON string inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_acceleration_from_position

        result = await calculate_acceleration_from_position(
            times=json.dumps([0, 1, 2, 3]),
            positions=json.dumps([[0, 0, 0], [5, 0, 0], [10, 0, 0], [15, 0, 0]]),
        )

        assert "velocities" in result
        assert "accelerations" in result

    @pytest.mark.asyncio
    async def test_calculate_jerk_with_lists(self):
        """Test jerk calculation with list inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_jerk

        result = await calculate_jerk(
            times=[0, 1, 2, 3, 4],
            accelerations=[[0, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0], [8, 0, 0]],
        )

        assert "jerks" in result
        assert "average_jerk" in result
        assert "max_jerk_magnitude" in result

    @pytest.mark.asyncio
    async def test_calculate_jerk_with_json_strings(self):
        """Test jerk calculation with JSON string inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_jerk

        result = await calculate_jerk(
            times=json.dumps([0, 1, 2, 3]),
            accelerations=json.dumps([[0, 0, 0], [2, 0, 0], [4, 0, 0], [6, 0, 0]]),
        )

        assert "jerks" in result
        assert "average_jerk" in result

    @pytest.mark.asyncio
    async def test_fit_trajectory_quadratic(self):
        """Test quadratic trajectory fitting."""
        from chuk_mcp_physics.tools.kinematics_tools import fit_trajectory

        result = await fit_trajectory(
            times=[0, 1, 2, 3],
            positions=[[0, 0, 0], [10, 15, 0], [20, 20, 0], [30, 15, 0]],
            fit_type="quadratic",
        )

        assert "coefficients_x" in result
        assert "coefficients_y" in result
        assert "coefficients_z" in result
        assert "r_squared" in result
        assert "predicted_positions" in result

    @pytest.mark.asyncio
    async def test_fit_trajectory_linear(self):
        """Test linear trajectory fitting."""
        from chuk_mcp_physics.tools.kinematics_tools import fit_trajectory

        result = await fit_trajectory(
            times=[0, 1, 2, 3],
            positions=[[0, 0, 0], [5, 0, 0], [10, 0, 0], [15, 0, 0]],
            fit_type="linear",
        )

        assert "coefficients_x" in result
        assert "r_squared" in result

    @pytest.mark.asyncio
    async def test_fit_trajectory_cubic(self):
        """Test cubic trajectory fitting."""
        from chuk_mcp_physics.tools.kinematics_tools import fit_trajectory

        result = await fit_trajectory(
            times=[0, 1, 2, 3, 4],
            positions=[[0, 0, 0], [1, 0, 0], [4, 0, 0], [9, 0, 0], [16, 0, 0]],
            fit_type="cubic",
        )

        assert "coefficients_x" in result
        assert "r_squared" in result

    @pytest.mark.asyncio
    async def test_fit_trajectory_with_json_strings(self):
        """Test trajectory fitting with JSON string inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import fit_trajectory

        result = await fit_trajectory(
            times=json.dumps([0, 1, 2, 3]),
            positions=json.dumps([[0, 0, 0], [5, 0, 0], [10, 0, 0], [15, 0, 0]]),
            fit_type="linear",
        )

        assert "coefficients_x" in result
        assert "r_squared" in result

    @pytest.mark.asyncio
    async def test_generate_motion_graph_magnitude(self):
        """Test motion graph generation for magnitude."""
        from chuk_mcp_physics.tools.kinematics_tools import generate_motion_graph

        result = await generate_motion_graph(
            times=[0, 1, 2, 3],
            positions=[[0, 0, 0], [5, 0, 0], [20, 0, 0], [45, 0, 0]],
            component="magnitude",
        )

        assert "times" in result
        assert "positions" in result
        assert "velocities" in result
        assert "accelerations" in result
        assert "component" in result

    @pytest.mark.asyncio
    async def test_generate_motion_graph_x_component(self):
        """Test motion graph generation for x component."""
        from chuk_mcp_physics.tools.kinematics_tools import generate_motion_graph

        result = await generate_motion_graph(
            times=[0, 1, 2, 3],
            positions=[[0, 0, 0], [5, 3, 0], [10, 6, 0], [15, 9, 0]],
            component="x",
        )

        assert "component" in result
        assert result["component"] == "x"

    @pytest.mark.asyncio
    async def test_generate_motion_graph_with_json_strings(self):
        """Test motion graph with JSON string inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import generate_motion_graph

        result = await generate_motion_graph(
            times=json.dumps([0, 1, 2, 3]),
            positions=json.dumps([[0, 0, 0], [5, 0, 0], [10, 0, 0], [15, 0, 0]]),
            component="x",
        )

        assert "times" in result
        assert "positions" in result

    @pytest.mark.asyncio
    async def test_calculate_average_speed_with_lists(self):
        """Test average speed calculation with list inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_average_speed

        result = await calculate_average_speed(
            positions=[[0, 0, 0], [10, 5, 0], [20, 10, 0], [15, 20, 0]],
            times=[0, 10, 20, 30],
        )

        assert "average_speed" in result
        assert "total_distance" in result
        assert "total_time" in result
        assert "displacement_magnitude" in result
        assert "displacement" in result

    @pytest.mark.asyncio
    async def test_calculate_average_speed_with_json_strings(self):
        """Test average speed with JSON string inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_average_speed

        result = await calculate_average_speed(
            positions=json.dumps([[0, 0, 0], [10, 0, 0], [20, 0, 0]]),
            times=json.dumps([0, 10, 20]),
        )

        assert "average_speed" in result
        assert "total_distance" in result

    @pytest.mark.asyncio
    async def test_calculate_instantaneous_velocity_with_lists(self):
        """Test instantaneous velocity with list inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_instantaneous_velocity

        result = await calculate_instantaneous_velocity(
            positions=[[0, 0, 0], [3, 4, 0], [6, 8, 0]],
            times=[0, 1, 2],
            target_time=1.0,
        )

        assert "velocity" in result
        assert "speed" in result
        assert "interpolated" in result

    @pytest.mark.asyncio
    async def test_calculate_instantaneous_velocity_with_json_strings(self):
        """Test instantaneous velocity with JSON string inputs."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_instantaneous_velocity

        result = await calculate_instantaneous_velocity(
            positions=json.dumps([[0, 0, 0], [3, 4, 0], [6, 8, 0]]),
            times=json.dumps([0, 1, 2]),
            target_time=1.0,
        )

        assert "velocity" in result
        assert "speed" in result

    @pytest.mark.asyncio
    async def test_calculate_instantaneous_velocity_interpolation(self):
        """Test instantaneous velocity with interpolation."""
        from chuk_mcp_physics.tools.kinematics_tools import calculate_instantaneous_velocity

        result = await calculate_instantaneous_velocity(
            positions=[[0, 0, 0], [10, 0, 0], [20, 0, 0]],
            times=[0, 1, 2],
            target_time=0.5,
        )

        assert "velocity" in result
        assert "interpolated" in result
