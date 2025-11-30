"""Tests for Rapier provider (mocked HTTP client)."""

import pytest
from unittest.mock import AsyncMock, patch, MagicMock
from chuk_mcp_physics.providers.rapier import RapierProvider
from chuk_mcp_physics.models import (
    SimulationConfig,
    RigidBodyDefinition,
    BodyType,
    ShapeType,
    ProjectileMotionRequest,
)


@pytest.fixture
def rapier_provider():
    """Create Rapier provider instance."""
    return RapierProvider()


class TestRapierAnalyticDelegation:
    """Test that analytic calculations delegate correctly."""

    @pytest.mark.asyncio
    async def test_projectile_motion_delegation(self, rapier_provider):
        """Test projectile motion delegates to analytic provider."""
        request = ProjectileMotionRequest(
            initial_velocity=20.0, angle_degrees=45.0, initial_height=0.0
        )
        result = await rapier_provider.calculate_projectile_motion(request)

        # Should work via delegation
        assert result.max_height > 0
        assert result.range > 0

    @pytest.mark.asyncio
    async def test_collision_check_delegation(self, rapier_provider):
        """Test collision check delegates."""
        from chuk_mcp_physics.models import CollisionCheckRequest

        request = CollisionCheckRequest(
            body1_position=[0, 0, 0],
            body1_velocity=[10, 0, 0],
            body1_radius=1.0,
            body2_position=[20, 0, 0],
            body2_velocity=[-10, 0, 0],
            body2_radius=1.0,
        )
        result = await rapier_provider.check_collision(request)
        assert result.will_collide is True

    @pytest.mark.asyncio
    async def test_force_delegation(self, rapier_provider):
        """Test force calculation delegates."""
        from chuk_mcp_physics.models import ForceCalculationRequest

        request = ForceCalculationRequest(mass=10.0, acceleration=[2.0, 0.0, 0.0])
        result = await rapier_provider.calculate_force(request)
        assert result.magnitude == pytest.approx(20.0)

    @pytest.mark.asyncio
    async def test_kinetic_energy_delegation(self, rapier_provider):
        """Test kinetic energy delegates."""
        from chuk_mcp_physics.models import KineticEnergyRequest

        request = KineticEnergyRequest(mass=10.0, velocity=[10.0, 0.0, 0.0])
        result = await rapier_provider.calculate_kinetic_energy(request)
        assert result.kinetic_energy > 0

    @pytest.mark.asyncio
    async def test_momentum_delegation(self, rapier_provider):
        """Test momentum delegates."""
        from chuk_mcp_physics.models import MomentumRequest

        request = MomentumRequest(mass=5.0, velocity=[10.0, 0.0, 0.0])
        result = await rapier_provider.calculate_momentum(request)
        assert result.magnitude == pytest.approx(50.0)


class TestRapierSimulation:
    """Test Rapier simulation methods (mocked)."""

    @pytest.mark.asyncio
    async def test_create_simulation(self, rapier_provider):
        """Test simulation creation."""
        config = SimulationConfig(gravity=[0, -9.81, 0], dimensions=3, dt=0.016)

        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {
            "sim_id": "sim_123",
            "config": {
                "gravity": [0, -9.81, 0],
                "dimensions": 3,
                "dt": 0.016,
                "integrator": "verlet",
            },
        }
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.create_simulation(config)

            assert result.sim_id == "sim_123"

    @pytest.mark.asyncio
    async def test_add_body(self, rapier_provider):
        """Test adding body to simulation."""
        body = RigidBodyDefinition(
            id="test_body",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.BOX,
            size=[1.0, 1.0, 1.0],
            mass=10.0,
        )

        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {"body_id": "test_body"}
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.add_body("sim_123", body)

            assert result == "test_body"

    @pytest.mark.asyncio
    async def test_step_simulation(self, rapier_provider):
        """Test stepping simulation."""
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {
            "sim_id": "sim_123",
            "time": 0.016,
            "bodies": [
                {
                    "id": "test_body",
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                    "velocity": [0.0, 0.0, 0.0],
                    "angular_velocity": [0.0, 0.0, 0.0],
                    "contacts": [],
                }
            ],
        }
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.step_simulation("sim_123", steps=1)

            assert result.sim_id == "sim_123"
            assert len(result.bodies) == 1

    @pytest.mark.asyncio
    async def test_step_simulation_with_dt(self, rapier_provider):
        """Test stepping simulation with custom dt."""
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {
            "sim_id": "sim_123",
            "time": 0.032,
            "bodies": [],
        }
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.step_simulation("sim_123", steps=2, dt=0.016)

            assert result.sim_id == "sim_123"

    @pytest.mark.asyncio
    async def test_get_simulation_state(self, rapier_provider):
        """Test getting simulation state."""
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {
            "sim_id": "sim_123",
            "time": 0.0,
            "bodies": [],
        }
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.get = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.get_simulation_state("sim_123")

            assert result.sim_id == "sim_123"

    @pytest.mark.asyncio
    async def test_record_trajectory(self, rapier_provider):
        """Test recording trajectory."""
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {
            "body_id": "test_body",
            "frames": [
                {
                    "time": 0.0,
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                    "velocity": [0.0, 0.0, 0.0],
                }
            ],
            "total_time": 1.0,
            "num_frames": 1,
        }
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.record_trajectory("sim_123", "test_body", steps=100)

            assert result.body_id == "test_body"
            assert result.num_frames == 1

    @pytest.mark.asyncio
    async def test_record_trajectory_with_dt(self, rapier_provider):
        """Test recording trajectory with custom dt."""
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.json.return_value = {
            "body_id": "test_body",
            "frames": [],
            "total_time": 0.0,
            "num_frames": 0,
        }
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            result = await rapier_provider.record_trajectory(
                "sim_123", "test_body", steps=100, dt=0.008
            )

            assert result.body_id == "test_body"

    @pytest.mark.asyncio
    async def test_destroy_simulation(self, rapier_provider):
        """Test destroying simulation."""
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.raise_for_status = MagicMock()

        with patch("httpx.AsyncClient") as mock_client:
            mock_context = AsyncMock()
            mock_context.__aenter__.return_value.delete = AsyncMock(return_value=mock_response)
            mock_client.return_value = mock_context

            # Should not raise
            await rapier_provider.destroy_simulation("sim_123")
