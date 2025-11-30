"""Analytic physics provider using closed-form equations.

This provider implements physics calculations using analytic formulas
for simple cases. It does not support full rigid-body simulations.
"""

import math
import numpy as np

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
from .base import PhysicsProvider


class AnalyticProvider(PhysicsProvider):
    """Physics provider using analytic calculations.

    This provider implements exact closed-form solutions for simple physics problems.
    It does not support complex rigid-body simulations (those require Rapier provider).
    """

    def __init__(self) -> None:
        """Initialize analytic provider."""
        self.name = "analytic"

    # ========================================================================
    # Analytic Calculations
    # ========================================================================

    async def calculate_projectile_motion(
        self, request: ProjectileMotionRequest
    ) -> ProjectileMotionResponse:
        """Calculate projectile motion using kinematic equations."""
        v0 = request.initial_velocity
        theta_rad = math.radians(request.angle_degrees)
        h0 = request.initial_height
        g = request.gravity

        # Velocity components
        v0x = v0 * math.cos(theta_rad)
        v0y = v0 * math.sin(theta_rad)

        # Maximum height
        max_height = h0 + (v0y**2) / (2 * g)

        # Time of flight (solve quadratic: h = h0 + v0y*t - 0.5*g*t²)
        # 0 = h0 + v0y*t - 0.5*g*t²
        # 0.5*g*t² - v0y*t - h0 = 0
        a = 0.5 * g
        b = -v0y
        c = -h0

        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            # Should not happen for valid inputs
            time_of_flight = 0.0
        else:
            t1 = (-b + math.sqrt(discriminant)) / (2 * a)
            t2 = (-b - math.sqrt(discriminant)) / (2 * a)
            time_of_flight = max(t1, t2)  # Take positive root

        # Range (horizontal distance)
        range_distance = v0x * time_of_flight

        # Generate trajectory points (sample at regular intervals)
        num_samples = 50
        trajectory_points = []
        for i in range(num_samples + 1):
            t = (i / num_samples) * time_of_flight
            x = v0x * t
            y = h0 + v0y * t - 0.5 * g * t**2
            if y >= 0:  # Only include points above ground
                trajectory_points.append([x, y])

        return ProjectileMotionResponse(
            max_height=max_height,
            range=range_distance,
            time_of_flight=time_of_flight,
            trajectory_points=trajectory_points,
        )

    async def check_collision(self, request: CollisionCheckRequest) -> CollisionCheckResponse:
        """Check collision using relative motion analysis."""
        # Convert to numpy arrays for vector operations
        p1 = np.array(request.body1_position)
        v1 = np.array(request.body1_velocity)
        r1 = request.body1_radius

        p2 = np.array(request.body2_position)
        v2 = np.array(request.body2_velocity)
        r2 = request.body2_radius

        # Relative position and velocity
        rel_pos = p2 - p1
        rel_vel = v2 - v1

        # Combined radius
        combined_radius = r1 + r2

        # If relative velocity is very small, objects are moving together
        rel_speed = np.linalg.norm(rel_vel)
        if rel_speed < 1e-6:
            # Stationary case
            distance = np.linalg.norm(rel_pos)
            will_collide = distance <= combined_radius
            return CollisionCheckResponse(
                will_collide=will_collide,
                collision_time=0.0 if will_collide else None,
                collision_point=p1.tolist() if will_collide else None,
                impact_speed=0.0 if will_collide else None,
                closest_approach_distance=distance,
                closest_approach_time=0.0,
            )

        # Time of closest approach (minimize distance²)
        # d²(t) = |rel_pos + rel_vel*t|²
        # d(d²)/dt = 0 → 2*(rel_pos + rel_vel*t)·rel_vel = 0
        # t = -(rel_pos·rel_vel) / (rel_vel·rel_vel)
        t_closest = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)

        # Clamp to valid time range
        if t_closest < 0:
            t_closest = 0.0
        elif t_closest > request.max_time:
            t_closest = request.max_time

        # Distance at closest approach
        pos_at_closest = rel_pos + rel_vel * t_closest
        closest_distance = np.linalg.norm(pos_at_closest)

        # Check if collision occurs
        will_collide = closest_distance <= combined_radius

        if will_collide:
            # Solve for exact collision time
            # |rel_pos + rel_vel*t|² = combined_radius²
            # This is a quadratic equation in t
            a = np.dot(rel_vel, rel_vel)
            b = 2 * np.dot(rel_pos, rel_vel)
            c = np.dot(rel_pos, rel_pos) - combined_radius**2

            discriminant = b**2 - 4 * a * c
            if discriminant >= 0:
                t_coll_1 = (-b - math.sqrt(discriminant)) / (2 * a)
                t_coll_2 = (-b + math.sqrt(discriminant)) / (2 * a)

                # Take earliest positive time
                collision_time = None
                if 0 <= t_coll_1 <= request.max_time:
                    collision_time = t_coll_1
                elif 0 <= t_coll_2 <= request.max_time:
                    collision_time = t_coll_2

                if collision_time is not None:
                    # Collision point (approximate as midpoint between centers)
                    p1_at_coll = p1 + v1 * collision_time
                    p2_at_coll = p2 + v2 * collision_time
                    collision_point = ((p1_at_coll + p2_at_coll) / 2).tolist()

                    # Impact speed (relative velocity magnitude)
                    impact_speed = rel_speed

                    return CollisionCheckResponse(
                        will_collide=True,
                        collision_time=collision_time,
                        collision_point=collision_point,
                        impact_speed=impact_speed,
                        closest_approach_distance=closest_distance,
                        closest_approach_time=t_closest,
                    )

        # No collision
        return CollisionCheckResponse(
            will_collide=False,
            collision_time=None,
            collision_point=None,
            impact_speed=None,
            closest_approach_distance=closest_distance,
            closest_approach_time=t_closest,
        )

    async def calculate_force(self, request: ForceCalculationRequest) -> ForceCalculationResponse:
        """Calculate force using F = ma."""
        mass = request.mass
        acceleration = np.array(request.acceleration)

        force = mass * acceleration
        magnitude = np.linalg.norm(force)

        return ForceCalculationResponse(force=force.tolist(), magnitude=magnitude)

    async def calculate_kinetic_energy(
        self, request: KineticEnergyRequest
    ) -> KineticEnergyResponse:
        """Calculate kinetic energy using KE = 0.5 * m * v²."""
        mass = request.mass
        velocity = np.array(request.velocity)

        speed = np.linalg.norm(velocity)
        kinetic_energy = 0.5 * mass * speed**2

        return KineticEnergyResponse(kinetic_energy=kinetic_energy, speed=speed)

    async def calculate_momentum(self, request: MomentumRequest) -> MomentumResponse:
        """Calculate momentum using p = mv."""
        mass = request.mass
        velocity = np.array(request.velocity)

        momentum = mass * velocity
        magnitude = np.linalg.norm(momentum)

        return MomentumResponse(momentum=momentum.tolist(), magnitude=magnitude)

    # ========================================================================
    # Simulation Methods (Not Supported)
    # ========================================================================

    async def create_simulation(self, config: SimulationConfig) -> SimulationCreateResponse:
        """Not supported by analytic provider."""
        raise NotImplementedError(
            "Analytic provider does not support simulations. Use Rapier provider instead."
        )

    async def add_body(self, sim_id: str, body: RigidBodyDefinition) -> str:
        """Not supported by analytic provider."""
        raise NotImplementedError(
            "Analytic provider does not support simulations. Use Rapier provider instead."
        )

    async def step_simulation(
        self, sim_id: str, steps: int = 1, dt: float | None = None
    ) -> SimulationStepResponse:
        """Not supported by analytic provider."""
        raise NotImplementedError(
            "Analytic provider does not support simulations. Use Rapier provider instead."
        )

    async def get_simulation_state(self, sim_id: str) -> SimulationStepResponse:
        """Not supported by analytic provider."""
        raise NotImplementedError(
            "Analytic provider does not support simulations. Use Rapier provider instead."
        )

    async def record_trajectory(
        self, sim_id: str, body_id: str, steps: int, dt: float | None = None
    ) -> TrajectoryResponse:
        """Not supported by analytic provider."""
        raise NotImplementedError(
            "Analytic provider does not support simulations. Use Rapier provider instead."
        )

    async def destroy_simulation(self, sim_id: str) -> None:
        """Not supported by analytic provider."""
        raise NotImplementedError(
            "Analytic provider does not support simulations. Use Rapier provider instead."
        )
