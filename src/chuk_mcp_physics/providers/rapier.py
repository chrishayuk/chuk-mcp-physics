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
    ElasticCollisionResponse,
    ForceCalculationRequest,
    ForceCalculationResponse,
    JointDefinition,
    KineticEnergyRequest,
    KineticEnergyResponse,
    MomentumRequest,
    MomentumResponse,
    PotentialEnergyResponse,
    ProjectileMotionRequest,
    ProjectileMotionResponse,
    RigidBodyDefinition,
    SimulationConfig,
    SimulationCreateResponse,
    SimulationStepResponse,
    TrajectoryResponse,
    WorkPowerResponse,
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

    async def calculate_potential_energy(
        self, mass: float, height: float, gravity: float = 9.81
    ) -> "PotentialEnergyResponse":
        """Delegate to analytic provider."""
        return await self._analytic.calculate_potential_energy(mass, height, gravity)

    async def calculate_work_power(
        self, force: list[float], displacement: list[float], time: float | None = None
    ) -> "WorkPowerResponse":
        """Delegate to analytic provider."""
        return await self._analytic.calculate_work_power(force, displacement, time)

    async def calculate_elastic_collision(
        self, mass1: float, velocity1: float, mass2: float, velocity2: float
    ) -> "ElasticCollisionResponse":
        """Delegate to analytic provider."""
        return await self._analytic.calculate_elastic_collision(mass1, velocity1, mass2, velocity2)

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
            # Serialize to JSON-compatible format (enums → strings)
            payload = body.model_dump(exclude_none=True, mode="json")
            logger.debug(f"Adding body to {sim_id}: {payload}")
            response = await client.post(
                f"{self.service_url}/simulations/{sim_id}/bodies",
                json=payload,
            )
            if response.status_code != 200 and response.status_code != 201:
                logger.error(
                    f"Failed to add body. Status: {response.status_code}, Response: {response.text}"
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

            # Transform Rapier service response to TrajectoryResponse format
            # Rapier returns: {"body_id": str, "frames": [...], "total_time": float, "num_frames": int}
            # We need: {"dt": float, "frames": [...], "meta": {...}}
            from ..models import TrajectoryMeta

            trajectory_dt = dt if dt is not None else 0.016  # Default from config

            # Handle both response formats:
            # 1. Flat: {"body_id": "...", "frames": [...], "total_time": ..., "num_frames": ...}
            # 2. Nested: {"frames": [...], "meta": {"body_id": "...", "total_time": ..., "num_frames": ...}}
            if "meta" in data:
                # Nested format (from tests/future API)
                meta_data = data["meta"]
                response_body_id = meta_data.get("body_id", body_id)
                total_time = meta_data["total_time"]
                num_frames = meta_data["num_frames"]
            else:
                # Flat format (current Rapier service)
                response_body_id = data.get("body_id", body_id)
                total_time = data["total_time"]
                num_frames = data["num_frames"]

            # Parse contact events if present (Phase 1.2)
            from ..models import ContactEvent

            contact_events = []
            if "contact_events" in data:
                for event_data in data["contact_events"]:
                    contact_events.append(ContactEvent(**event_data))

            return TrajectoryResponse(
                dt=trajectory_dt,
                frames=data["frames"],
                meta=TrajectoryMeta(
                    body_id=f"rapier://{sim_id}/{response_body_id}",
                    total_time=total_time,
                    num_frames=num_frames,
                ),
                contact_events=contact_events,
            )

    async def add_joint(self, sim_id: str, joint: "JointDefinition") -> str:
        """Add a joint/constraint between two bodies.

        POST /simulations/{sim_id}/joints
        Body: { "id": "...", "joint_type": "revolute", "body_a": "...", "body_b": "...", ... }
        Response: { "joint_id": "..." }
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            # Serialize to JSON-compatible format
            payload = joint.model_dump(exclude_none=True, mode="json")
            logger.debug(f"Adding joint to {sim_id}: {payload}")

            response = await client.post(
                f"{self.service_url}/simulations/{sim_id}/joints",
                json=payload,
            )
            response.raise_for_status()
            data = response.json()
            return data["joint_id"]

    async def destroy_simulation(self, sim_id: str) -> None:
        """Destroy a simulation.

        DELETE /simulations/{sim_id}
        Response: 204 No Content
        """
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            response = await client.delete(f"{self.service_url}/simulations/{sim_id}")
            response.raise_for_status()
