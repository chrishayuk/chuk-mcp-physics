"""Tests for simulation MCP tools (create_simulation, add_body, add_joint, etc.)."""

import pytest
from unittest.mock import AsyncMock, patch

from chuk_mcp_physics.server import (
    create_simulation,
    add_rigid_body,
    add_joint,
    step_simulation,
    record_trajectory,
    record_trajectory_with_events,
    destroy_simulation,
)
from chuk_mcp_physics.models import (
    SimulationConfig,
    SimulationCreateResponse,
    JointDefinition,
    JointType,
    SimulationStepResponse,
    RigidBodyState,
    TrajectoryResponse,
    TrajectoryFrame,
    TrajectoryMeta,
)


class TestSimulationTools:
    """Test simulation MCP tools."""

    @pytest.mark.asyncio
    async def test_create_simulation(self):
        """Test create_simulation tool."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.create_simulation.return_value = SimulationCreateResponse(
                sim_id="test-sim-123",
                config=SimulationConfig(
                    gravity=[0.0, -9.81, 0.0],
                    dimensions=3,
                    dt=0.016,
                ),
            )
            mock_get_provider.return_value = mock_provider

            result = await create_simulation(
                gravity_x=0.0,
                gravity_y=-9.81,
                gravity_z=0.0,
                dimensions=3,
                dt=0.016,
            )

            assert result.sim_id == "test-sim-123"
            assert result.config.gravity == [0.0, -9.81, 0.0]
            mock_provider.create_simulation.assert_called_once()

    @pytest.mark.asyncio
    async def test_add_rigid_body_basic(self):
        """Test add_rigid_body tool with basic parameters."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.add_body.return_value = "ball"
            mock_get_provider.return_value = mock_provider

            result = await add_rigid_body(
                sim_id="test-sim",
                body_id="ball",
                body_type="dynamic",
                shape="sphere",
                size=[0.5],
                mass=1.0,
                position=[0.0, 5.0, 0.0],
            )

            assert result == "ball"
            mock_provider.add_body.assert_called_once()

            # Verify the body definition passed to provider
            call_args = mock_provider.add_body.call_args
            body_def = call_args[0][1]  # Second argument is the body definition
            assert body_def.id == "ball"
            assert body_def.mass == 1.0
            assert body_def.size == [0.5]

    @pytest.mark.asyncio
    async def test_add_rigid_body_with_damping(self):
        """Test add_rigid_body tool with damping parameters."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.add_body.return_value = "damped_ball"
            mock_get_provider.return_value = mock_provider

            result = await add_rigid_body(
                sim_id="test-sim",
                body_id="damped_ball",
                body_type="dynamic",
                shape="sphere",
                size=[0.5],
                mass=1.0,
                linear_damping=0.5,
                angular_damping=0.3,
            )

            assert result == "damped_ball"

            # Verify damping was passed
            call_args = mock_provider.add_body.call_args
            body_def = call_args[0][1]
            assert body_def.linear_damping == 0.5
            assert body_def.angular_damping == 0.3

    @pytest.mark.asyncio
    async def test_add_rigid_body_with_lists_as_strings(self):
        """Test add_rigid_body with list parameters as JSON strings (MCP CLI format)."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.add_body.return_value = "ball"
            mock_get_provider.return_value = mock_provider

            result = await add_rigid_body(
                sim_id="test-sim",
                body_id="ball",
                body_type="dynamic",
                shape="sphere",
                size="[0.5]",  # String instead of list
                position="[1.0, 2.0, 3.0]",  # String instead of list
                velocity="[0.0, -5.0, 0.0]",  # String instead of list
            )

            assert result == "ball"

            # Verify lists were parsed correctly
            call_args = mock_provider.add_body.call_args
            body_def = call_args[0][1]
            assert body_def.size == [0.5]
            assert body_def.position == [1.0, 2.0, 3.0]
            assert body_def.velocity == [0.0, -5.0, 0.0]

    @pytest.mark.asyncio
    async def test_add_joint_revolute(self):
        """Test add_joint tool with revolute joint."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.add_joint.return_value = "hinge"
            mock_get_provider.return_value = mock_provider

            joint = JointDefinition(
                id="hinge",
                joint_type=JointType.REVOLUTE,
                body_a="anchor",
                body_b="bob",
                anchor_a=[0.0, 0.0, 0.0],
                anchor_b=[0.0, 0.2, 0.0],
                axis=[0.0, 0.0, 1.0],
            )

            result = await add_joint(sim_id="test-sim", joint=joint)

            assert result == "hinge"
            mock_provider.add_joint.assert_called_once()

    @pytest.mark.asyncio
    async def test_add_joint_spherical(self):
        """Test add_joint tool with spherical joint."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.add_joint.return_value = "ball_joint"
            mock_get_provider.return_value = mock_provider

            joint = JointDefinition(
                id="ball_joint",
                joint_type=JointType.SPHERICAL,
                body_a="link1",
                body_b="link2",
            )

            result = await add_joint(sim_id="test-sim", joint=joint)

            assert result == "ball_joint"

    @pytest.mark.asyncio
    async def test_step_simulation(self):
        """Test step_simulation tool."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.step_simulation.return_value = SimulationStepResponse(
                sim_id="test-sim",
                time=1.0,
                bodies=[
                    RigidBodyState(
                        id="ball",
                        position=[0.0, 4.0, 0.0],
                        orientation=[0.0, 0.0, 0.0, 1.0],
                        velocity=[0.0, -9.81, 0.0],
                        angular_velocity=[0.0, 0.0, 0.0],
                        contacts=[],
                    )
                ],
            )
            mock_get_provider.return_value = mock_provider

            result = await step_simulation(sim_id="test-sim", steps=60)

            assert result.sim_id == "test-sim"
            assert result.time == 1.0
            assert len(result.bodies) == 1
            mock_provider.step_simulation.assert_called_once_with("test-sim", 60, None)

    @pytest.mark.asyncio
    async def test_record_trajectory_with_events(self):
        """Test record_trajectory_with_events tool."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.record_trajectory.return_value = TrajectoryResponse(
                dt=0.016,
                frames=[
                    TrajectoryFrame(
                        time=0.0,
                        position=[0.0, 5.0, 0.0],
                        orientation=[0.0, 0.0, 0.0, 1.0],
                        velocity=[0.0, 0.0, 0.0],
                    )
                ],
                meta=TrajectoryMeta(
                    body_id="rapier://test-sim/ball",
                    total_time=0.016,
                    num_frames=1,
                ),
                contact_events=[],
            )
            mock_get_provider.return_value = mock_provider

            result = await record_trajectory_with_events(sim_id="test-sim", body_id="ball", steps=1)

            assert result.dt == 0.016
            assert len(result.frames) == 1
            assert len(result.bounces) == 0  # No bounces in this simple case

    @pytest.mark.asyncio
    async def test_record_trajectory(self):
        """Test record_trajectory tool."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.record_trajectory.return_value = TrajectoryResponse(
                dt=0.016,
                frames=[
                    TrajectoryFrame(
                        time=0.0,
                        position=[0.0, 5.0, 0.0],
                        orientation=[0.0, 0.0, 0.0, 1.0],
                        velocity=[0.0, 0.0, 0.0],
                    )
                ],
                meta=TrajectoryMeta(
                    body_id="rapier://test-sim/ball",
                    total_time=0.016,
                    num_frames=1,
                ),
                contact_events=[],
            )
            mock_get_provider.return_value = mock_provider

            result = await record_trajectory(sim_id="test-sim", body_id="ball", steps=1)

            assert result.dt == 0.016
            assert len(result.frames) == 1
            mock_provider.record_trajectory.assert_called_once_with("test-sim", "ball", 1, None)

    @pytest.mark.asyncio
    async def test_destroy_simulation(self):
        """Test destroy_simulation tool."""
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.destroy_simulation.return_value = None
            mock_get_provider.return_value = mock_provider

            result = await destroy_simulation(sim_id="test-sim")

            assert isinstance(result, str)
            assert "test-sim" in result
            mock_provider.destroy_simulation.assert_called_once_with("test-sim")
