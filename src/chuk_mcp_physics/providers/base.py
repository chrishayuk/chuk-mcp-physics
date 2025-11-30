"""Abstract base provider for physics calculations.

This module defines the interface that all providers must implement.
"""

from abc import ABC, abstractmethod

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


class PhysicsProvider(ABC):
    """Abstract base class for physics providers.

    All provider implementations (Analytic, Rapier, etc.) must implement
    these methods to provide a consistent interface.
    """

    # ========================================================================
    # Analytic Calculations (all providers should support these)
    # ========================================================================

    @abstractmethod
    async def calculate_projectile_motion(
        self, request: ProjectileMotionRequest
    ) -> ProjectileMotionResponse:
        """Calculate projectile motion trajectory.

        Args:
            request: Projectile motion parameters

        Returns:
            ProjectileMotionResponse with trajectory data
        """
        pass  # pragma: no cover

    @abstractmethod
    async def check_collision(self, request: CollisionCheckRequest) -> CollisionCheckResponse:
        """Check if two moving objects will collide.

        Args:
            request: Two bodies with positions, velocities, and radii

        Returns:
            CollisionCheckResponse with collision prediction
        """
        pass  # pragma: no cover

    @abstractmethod
    async def calculate_force(self, request: ForceCalculationRequest) -> ForceCalculationResponse:
        """Calculate force from mass and acceleration (F = ma).

        Args:
            request: Mass and acceleration

        Returns:
            ForceCalculationResponse with force vector
        """
        pass  # pragma: no cover

    @abstractmethod
    async def calculate_kinetic_energy(
        self, request: KineticEnergyRequest
    ) -> KineticEnergyResponse:
        """Calculate kinetic energy (KE = 0.5 * m * v²).

        Args:
            request: Mass and velocity

        Returns:
            KineticEnergyResponse with energy value
        """
        pass  # pragma: no cover

    @abstractmethod
    async def calculate_momentum(self, request: MomentumRequest) -> MomentumResponse:
        """Calculate momentum (p = mv).

        Args:
            request: Mass and velocity

        Returns:
            MomentumResponse with momentum vector
        """
        pass  # pragma: no cover

    # ========================================================================
    # Simulation Methods (may raise NotImplementedError for analytic provider)
    # ========================================================================

    @abstractmethod
    async def create_simulation(self, config: SimulationConfig) -> SimulationCreateResponse:
        """Create a new physics simulation.

        Args:
            config: Simulation configuration

        Returns:
            SimulationCreateResponse with simulation ID

        Raises:
            NotImplementedError: If provider doesn't support simulations
        """
        pass  # pragma: no cover

    @abstractmethod
    async def add_body(self, sim_id: str, body: RigidBodyDefinition) -> str:
        """Add a rigid body to a simulation.

        Args:
            sim_id: Simulation identifier
            body: Body definition

        Returns:
            Body ID (same as body.id)

        Raises:
            NotImplementedError: If provider doesn't support simulations
            ValueError: If simulation doesn't exist
        """
        pass  # pragma: no cover

    @abstractmethod
    async def step_simulation(
        self, sim_id: str, steps: int = 1, dt: float | None = None
    ) -> SimulationStepResponse:
        """Step the simulation forward in time.

        Args:
            sim_id: Simulation identifier
            steps: Number of simulation steps
            dt: Optional override for timestep (uses config default if None)

        Returns:
            SimulationStepResponse with current state

        Raises:
            NotImplementedError: If provider doesn't support simulations
            ValueError: If simulation doesn't exist
        """
        pass  # pragma: no cover

    @abstractmethod
    async def get_simulation_state(self, sim_id: str) -> SimulationStepResponse:
        """Get current simulation state without stepping.

        Args:
            sim_id: Simulation identifier

        Returns:
            SimulationStepResponse with current state

        Raises:
            NotImplementedError: If provider doesn't support simulations
            ValueError: If simulation doesn't exist
        """
        pass  # pragma: no cover

    @abstractmethod
    async def record_trajectory(
        self, sim_id: str, body_id: str, steps: int, dt: float | None = None
    ) -> TrajectoryResponse:
        """Record trajectory of a body over time.

        Args:
            sim_id: Simulation identifier
            body_id: Body to track
            steps: Number of simulation steps to record
            dt: Optional override for timestep

        Returns:
            TrajectoryResponse with recorded frames

        Raises:
            NotImplementedError: If provider doesn't support simulations
            ValueError: If simulation or body doesn't exist
        """
        pass  # pragma: no cover

    @abstractmethod
    async def destroy_simulation(self, sim_id: str) -> None:
        """Destroy a simulation and free resources.

        Args:
            sim_id: Simulation identifier

        Raises:
            NotImplementedError: If provider doesn't support simulations
            ValueError: If simulation doesn't exist
        """
        pass  # pragma: no cover
