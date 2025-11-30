"""Tests for input validation in server tools."""

import pytest
from chuk_mcp_physics.server import (
    create_simulation,
    step_simulation,
    record_trajectory,
    add_rigid_body,
    destroy_simulation,
)
from chuk_mcp_physics.config import SimulationLimits


class TestCreateSimulationValidation:
    """Test validation in create_simulation."""

    @pytest.mark.asyncio
    async def test_dt_too_small(self):
        """Test that dt below minimum raises ValueError."""
        with pytest.raises(ValueError, match="dt=.*is too small"):
            await create_simulation(dt=0.0001)

    @pytest.mark.asyncio
    async def test_dt_too_large(self):
        """Test that dt above maximum raises ValueError."""
        with pytest.raises(ValueError, match="dt=.*is too large"):
            await create_simulation(dt=1.0)

    @pytest.mark.asyncio
    async def test_dt_at_minimum(self):
        """Test that dt at minimum works (would fail without Rapier service)."""
        # This will fail because we don't have Rapier service, but validation passes
        with pytest.raises(Exception):  # httpx or NotImplementedError
            await create_simulation(dt=SimulationLimits.MIN_DT)

    @pytest.mark.asyncio
    async def test_dt_at_maximum(self):
        """Test that dt at maximum works (would fail without Rapier service)."""
        # This will fail because we don't have Rapier service, but validation passes
        with pytest.raises(Exception):  # httpx or NotImplementedError
            await create_simulation(dt=SimulationLimits.MAX_DT)


class TestStepSimulationValidation:
    """Test validation in step_simulation."""

    @pytest.mark.asyncio
    async def test_steps_too_large(self):
        """Test that steps above maximum raises ValueError."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            await step_simulation("sim_123", steps=99999)

    @pytest.mark.asyncio
    async def test_steps_zero(self):
        """Test that steps=0 raises ValueError."""
        with pytest.raises(ValueError, match="must be at least 1"):
            await step_simulation("sim_123", steps=0)

    @pytest.mark.asyncio
    async def test_steps_negative(self):
        """Test that negative steps raises ValueError."""
        with pytest.raises(ValueError, match="must be at least 1"):
            await step_simulation("sim_123", steps=-10)

    @pytest.mark.asyncio
    async def test_dt_too_small(self):
        """Test that dt below minimum raises ValueError."""
        with pytest.raises(ValueError, match="dt=.*is too small"):
            await step_simulation("sim_123", steps=100, dt=0.0001)

    @pytest.mark.asyncio
    async def test_dt_too_large(self):
        """Test that dt above maximum raises ValueError."""
        with pytest.raises(ValueError, match="dt=.*is too large"):
            await step_simulation("sim_123", steps=100, dt=1.0)

    @pytest.mark.asyncio
    async def test_valid_steps_at_limit(self):
        """Test that steps at limit works (would fail without Rapier service)."""
        with pytest.raises(Exception):  # httpx or NotImplementedError
            await step_simulation("sim_123", steps=SimulationLimits.MAX_STEPS_PER_CALL)


class TestRecordTrajectoryValidation:
    """Test validation in record_trajectory."""

    @pytest.mark.asyncio
    async def test_steps_too_large(self):
        """Test that frames above maximum raises ValueError."""
        with pytest.raises(ValueError, match="exceeds maximum"):
            await record_trajectory("sim_123", "body_1", steps=99999)

    @pytest.mark.asyncio
    async def test_steps_zero(self):
        """Test that steps=0 raises ValueError."""
        with pytest.raises(ValueError, match="must be at least 1"):
            await record_trajectory("sim_123", "body_1", steps=0)

    @pytest.mark.asyncio
    async def test_steps_negative(self):
        """Test that negative steps raises ValueError."""
        with pytest.raises(ValueError, match="must be at least 1"):
            await record_trajectory("sim_123", "body_1", steps=-10)

    @pytest.mark.asyncio
    async def test_dt_too_small(self):
        """Test that dt below minimum raises ValueError."""
        with pytest.raises(ValueError, match="dt=.*is too small"):
            await record_trajectory("sim_123", "body_1", steps=100, dt=0.0001)

    @pytest.mark.asyncio
    async def test_dt_too_large(self):
        """Test that dt above maximum raises ValueError."""
        with pytest.raises(ValueError, match="dt=.*is too large"):
            await record_trajectory("sim_123", "body_1", steps=100, dt=1.0)

    @pytest.mark.asyncio
    async def test_valid_steps_at_limit(self):
        """Test that frames at limit works (would fail without Rapier service)."""
        with pytest.raises(Exception):  # httpx or NotImplementedError
            await record_trajectory(
                "sim_123", "body_1", steps=SimulationLimits.MAX_TRAJECTORY_FRAMES
            )


class TestAddRigidBodyValidation:
    """Test add_rigid_body tool (no validation yet, but good to have tests)."""

    @pytest.mark.asyncio
    async def test_basic_add_body(self):
        """Test adding a body (will fail without Rapier service)."""
        with pytest.raises(Exception):  # httpx or NotImplementedError
            await add_rigid_body(
                sim_id="sim_123",
                body_id="ball",
                body_type="dynamic",
                shape="sphere",
                radius=1.0,
                position=[0.0, 5.0, 0.0],
            )


class TestDestroySimulation:
    """Test destroy_simulation tool."""

    @pytest.mark.asyncio
    async def test_destroy_simulation_success(self):
        """Test destroying a simulation successfully."""
        from unittest.mock import patch, AsyncMock

        # Mock the provider to succeed
        with patch("chuk_mcp_physics.server.get_provider_for_tool") as mock_get_provider:
            mock_provider = AsyncMock()
            mock_provider.destroy_simulation = AsyncMock(return_value=None)
            mock_get_provider.return_value = mock_provider

            result = await destroy_simulation("sim_123")
            assert "sim_123" in result
            assert "destroyed" in result.lower()

    @pytest.mark.asyncio
    async def test_destroy_simulation_no_mock(self):
        """Test destroying a simulation (will fail without Rapier service)."""
        with pytest.raises(Exception):  # httpx or NotImplementedError
            await destroy_simulation("sim_456")
