"""Rapier physics provider - HTTP client for Rapier service.

This provider communicates with a separate Rapier microservice (written in Rust)
that provides full rigid-body physics simulation capabilities.

See RAPIER_SERVICE.md for details on the service interface.
"""

import logging
import httpx

from ..config import RapierConfig
from ..models import (
    CollisionCheckRequest,
    CollisionCheckResponse,
    ForceCalculationRequest,
    ForceCalculationResponse,
    KineticEnergyRequest,
    KineticEnergyResponse,
    MomentumRequest,
    MomentumResponse,
    ProjectileMotionRequest,
    ProjectileMotionResponse,
    RigidBodyDefinition,
    SimulationConfig,
    SimulationCreateResponse,
    SimulationStepResponse,
    TrajectoryResponse,
)
from .analytic import AnalyticProvider
from .base import PhysicsProvider

logger = logging.getLogger(__name__)


class RapierProvider(PhysicsProvider):
    """Physics provider using external Rapier service.

    This provider uses an external Rust microservice running Rapier physics engine
    for full rigid-body simulation. For simple analytic calculations, it delegates
    to the AnalyticProvider for efficiency.

    Architecture:
        LLM → MCP Server (Python) → Rapier Service (Rust) → Rapier3D library
    """

    def __init__(self) -> None:
        """Initialize Rapier provider."""
        self.name = "rapier"
        self.service_url = RapierConfig.SERVICE_URL
        self.timeout = RapierConfig.TIMEOUT
        self.max_retries = RapierConfig.MAX_RETRIES
        self.retry_delay = RapierConfig.RETRY_DELAY

        # Delegate analytic calculations to analytic provider for efficiency
        self._analytic = AnalyticProvider()

        logger.info(f"Initialized Rapier provider with service URL: {self.service_url}")

    # ========================================================================
    # Analytic Calculations (delegated to AnalyticProvider)
    # ========================================================================

    async def calculate_projectile_motion(
        self, request: ProjectileMotionRequest
    ) -> ProjectileMotionResponse:
        """Delegate to analytic provider."""
        return await self._analytic.calculate_projectile_motion(request)

    async def check_collision(self, request: CollisionCheckRequest) -> CollisionCheckResponse:
        """Delegate to analytic provider."""
        return await self._analytic.check_collision(request)

    async def calculate_force(self, request: ForceCalculationRequest) -> ForceCalculationResponse:
        """Delegate to analytic provider."""
        return await self._analytic.calculate_force(request)

    async def calculate_kinetic_energy(
        self, request: KineticEnergyRequest
    ) -> KineticEnergyResponse:
        """Delegate to analytic provider."""
        return await self._analytic.calculate_kinetic_energy(request)

    async def calculate_momentum(self, request: MomentumRequest) -> MomentumResponse:
        """Delegate to analytic provider."""
        return await self._analytic.calculate_momentum(request)

    # ========================================================================
    # Simulation Methods (Rapier service)
    # ========================================================================

    async def create_simulation(self, config: SimulationConfig) -> SimulationCreateResponse:
        """Create a new simulation in Rapier service.

        POST /simulations
        Body: { "gravity": [x, y, z], "dimensions": 3, "dt": 0.016 }
        Response: { "sim_id": "...", "config": {...} }
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            response = await client.post(
                f"{self.service_url}/simulations",
                json={
                    "gravity": config.gravity,
                    "dimensions": config.dimensions,
                    "dt": config.dt,
                    "integrator": config.integrator.value,
                },
            )
            response.raise_for_status()
            data = response.json()
            return SimulationCreateResponse(**data)

    async def add_body(self, sim_id: str, body: RigidBodyDefinition) -> str:
        """Add a body to a simulation.

        POST /simulations/{sim_id}/bodies
        Body: { "id": "...", "kind": "dynamic", ... }
        Response: { "body_id": "..." }
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            response = await client.post(
                f"{self.service_url}/simulations/{sim_id}/bodies",
                json=body.model_dump(),
            )
            response.raise_for_status()
            data = response.json()
            return data["body_id"]

    async def step_simulation(
        self, sim_id: str, steps: int = 1, dt: float | None = None
    ) -> SimulationStepResponse:
        """Step the simulation forward.

        POST /simulations/{sim_id}/step
        Body: { "steps": 600, "dt": 0.016 }
        Response: { "sim_id": "...", "time": 9.6, "bodies": [...] }
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            payload: dict[str, int | float] = {"steps": steps}
            if dt is not None:
                payload["dt"] = dt

            response = await client.post(
                f"{self.service_url}/simulations/{sim_id}/step",
                json=payload,
            )
            response.raise_for_status()
            data = response.json()
            return SimulationStepResponse(**data)

    async def get_simulation_state(self, sim_id: str) -> SimulationStepResponse:
        """Get current simulation state without stepping.

        GET /simulations/{sim_id}/state
        Response: { "sim_id": "...", "time": 0.0, "bodies": [...] }
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            response = await client.get(f"{self.service_url}/simulations/{sim_id}/state")
            response.raise_for_status()
            data = response.json()
            return SimulationStepResponse(**data)

    async def record_trajectory(
        self, sim_id: str, body_id: str, steps: int, dt: float | None = None
    ) -> TrajectoryResponse:
        """Record trajectory of a body.

        POST /simulations/{sim_id}/bodies/{body_id}/trajectory
        Body: { "steps": 1000, "dt": 0.016 }
        Response: { "body_id": "...", "frames": [...], "total_time": 16.0, "num_frames": 1000 }
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            payload: dict[str, int | float] = {"steps": steps}
            if dt is not None:
                payload["dt"] = dt

            response = await client.post(
                f"{self.service_url}/simulations/{sim_id}/bodies/{body_id}/trajectory",
                json=payload,
            )
            response.raise_for_status()
            data = response.json()
            return TrajectoryResponse(**data)

    async def destroy_simulation(self, sim_id: str) -> None:
        """Destroy a simulation.

        DELETE /simulations/{sim_id}
        Response: 204 No Content
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            response = await client.delete(f"{self.service_url}/simulations/{sim_id}")
            response.raise_for_status()
