"""Advanced collision calculations.

Implements 3D elastic and inelastic collisions with coefficient of restitution.
"""

import math

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class InelasticCollision3DRequest(BaseModel):
    """Request for 3D inelastic collision calculation."""

    mass1: float = Field(..., description="Mass of object 1 in kg", gt=0.0)
    velocity1: list[float] = Field(..., description="Velocity of object 1 [x, y, z] in m/s")
    mass2: float = Field(..., description="Mass of object 2 in kg", gt=0.0)
    velocity2: list[float] = Field(..., description="Velocity of object 2 [x, y, z] in m/s")
    coefficient_of_restitution: float = Field(
        default=0.0,
        description="Coefficient of restitution (0.0=perfectly inelastic, 1.0=perfectly elastic)",
        ge=0.0,
        le=1.0,
    )


class InelasticCollision3DResponse(BaseModel):
    """Response for 3D inelastic collision calculation."""

    final_velocity1: list[float] = Field(
        ..., description="Final velocity of object 1 [x, y, z] in m/s"
    )
    final_velocity2: list[float] = Field(
        ..., description="Final velocity of object 2 [x, y, z] in m/s"
    )
    initial_momentum: list[float] = Field(
        ..., description="Total initial momentum [x, y, z] in kg⋅m/s"
    )
    final_momentum: list[float] = Field(..., description="Total final momentum [x, y, z] in kg⋅m/s")
    initial_kinetic_energy: float = Field(..., description="Total initial kinetic energy in Joules")
    final_kinetic_energy: float = Field(..., description="Total final kinetic energy in Joules")
    energy_loss: float = Field(..., description="Kinetic energy lost in collision in Joules")
    energy_loss_percent: float = Field(..., description="Percentage of kinetic energy lost")


class ElasticCollision3DRequest(BaseModel):
    """Request for 3D elastic collision calculation."""

    mass1: float = Field(..., description="Mass of object 1 in kg", gt=0.0)
    velocity1: list[float] = Field(..., description="Velocity of object 1 [x, y, z] in m/s")
    mass2: float = Field(..., description="Mass of object 2 in kg", gt=0.0)
    velocity2: list[float] = Field(..., description="Velocity of object 2 [x, y, z] in m/s")


class ElasticCollision3DResponse(BaseModel):
    """Response for 3D elastic collision calculation."""

    final_velocity1: list[float] = Field(
        ..., description="Final velocity of object 1 [x, y, z] in m/s"
    )
    final_velocity2: list[float] = Field(
        ..., description="Final velocity of object 2 [x, y, z] in m/s"
    )
    initial_momentum: list[float] = Field(
        ..., description="Total initial momentum [x, y, z] in kg⋅m/s"
    )
    final_momentum: list[float] = Field(..., description="Total final momentum [x, y, z] in kg⋅m/s")
    initial_kinetic_energy: float = Field(..., description="Total initial kinetic energy in Joules")
    final_kinetic_energy: float = Field(..., description="Total final kinetic energy in Joules")


# ============================================================================
# Helper Functions
# ============================================================================


def _dot_product(a: list[float], b: list[float]) -> float:
    """Calculate dot product of two 3D vectors."""
    return sum(x * y for x, y in zip(a, b))


def _vector_magnitude(v: list[float]) -> float:
    """Calculate magnitude of a 3D vector."""
    return math.sqrt(sum(x * x for x in v))


def _vector_subtract(a: list[float], b: list[float]) -> list[float]:
    """Subtract vector b from vector a."""
    return [x - y for x, y in zip(a, b)]


def _vector_add(a: list[float], b: list[float]) -> list[float]:
    """Add two vectors."""
    return [x + y for x, y in zip(a, b)]


def _vector_scale(v: list[float], scalar: float) -> list[float]:
    """Multiply vector by scalar."""
    return [x * scalar for x in v]


# ============================================================================
# Calculation Functions
# ============================================================================


def calculate_inelastic_collision_3d(
    request: InelasticCollision3DRequest,
) -> InelasticCollision3DResponse:
    """Calculate 3D collision with coefficient of restitution.

    Uses conservation of momentum and coefficient of restitution to solve
    for final velocities in 3D.

    For coefficient of restitution e:
    - e = 0: Perfectly inelastic (objects stick together)
    - 0 < e < 1: Inelastic (some energy lost)
    - e = 1: Perfectly elastic (no energy lost)

    Args:
        request: Inelastic collision request

    Returns:
        Final velocities, momentum, and energy analysis
    """
    m1 = request.mass1
    v1 = request.velocity1
    m2 = request.mass2
    v2 = request.velocity2
    e = request.coefficient_of_restitution

    # Calculate initial momentum
    p1_initial = _vector_scale(v1, m1)
    p2_initial = _vector_scale(v2, m2)
    p_total = _vector_add(p1_initial, p2_initial)

    # Calculate initial kinetic energy
    ke1_initial = 0.5 * m1 * _dot_product(v1, v1)
    ke2_initial = 0.5 * m2 * _dot_product(v2, v2)
    ke_total_initial = ke1_initial + ke2_initial

    # For 3D collision, we need to work in the collision frame
    # Relative velocity
    v_rel = _vector_subtract(v1, v2)
    v_rel_mag = _vector_magnitude(v_rel)

    if v_rel_mag < 1e-10:
        # Objects already at same velocity
        return InelasticCollision3DResponse(
            final_velocity1=v1,
            final_velocity2=v2,
            initial_momentum=p_total,
            final_momentum=p_total,
            initial_kinetic_energy=ke_total_initial,
            final_kinetic_energy=ke_total_initial,
            energy_loss=0.0,
            energy_loss_percent=0.0,
        )

    # Unit vector in direction of collision (along relative velocity)
    n = _vector_scale(v_rel, 1.0 / v_rel_mag)

    # Velocity components along collision normal
    v1n = _dot_product(v1, n)
    v2n = _dot_product(v2, n)

    # Calculate final velocities along normal using 1D collision formulas with restitution
    # Conservation of momentum: m1*v1n + m2*v2n = m1*v1n' + m2*v2n'
    # Coefficient of restitution: e = (v2n' - v1n') / (v1n - v2n)

    # Solving these two equations:
    v1n_final = ((m1 - e * m2) * v1n + m2 * (1 + e) * v2n) / (m1 + m2)
    v2n_final = ((m2 - e * m1) * v2n + m1 * (1 + e) * v1n) / (m1 + m2)

    # Calculate change in normal velocity
    delta_v1n = v1n_final - v1n
    delta_v2n = v2n_final - v2n

    # Apply changes to original velocities
    v1_final = _vector_add(v1, _vector_scale(n, delta_v1n))
    v2_final = _vector_add(v2, _vector_scale(n, delta_v2n))

    # Calculate final momentum and energy
    p1_final = _vector_scale(v1_final, m1)
    p2_final = _vector_scale(v2_final, m2)
    p_final = _vector_add(p1_final, p2_final)

    ke1_final = 0.5 * m1 * _dot_product(v1_final, v1_final)
    ke2_final = 0.5 * m2 * _dot_product(v2_final, v2_final)
    ke_total_final = ke1_final + ke2_final

    energy_loss = ke_total_initial - ke_total_final
    energy_loss_percent = (energy_loss / ke_total_initial * 100.0) if ke_total_initial > 0 else 0.0

    return InelasticCollision3DResponse(
        final_velocity1=v1_final,
        final_velocity2=v2_final,
        initial_momentum=p_total,
        final_momentum=p_final,
        initial_kinetic_energy=ke_total_initial,
        final_kinetic_energy=ke_total_final,
        energy_loss=energy_loss,
        energy_loss_percent=energy_loss_percent,
    )


def calculate_elastic_collision_3d(
    request: ElasticCollision3DRequest,
) -> ElasticCollision3DResponse:
    """Calculate 3D elastic collision (perfect energy conservation).

    This is a special case of inelastic collision with e = 1.0.

    Args:
        request: Elastic collision request

    Returns:
        Final velocities, momentum, and energy
    """
    # Use inelastic collision with e = 1.0
    inelastic_request = InelasticCollision3DRequest(
        mass1=request.mass1,
        velocity1=request.velocity1,
        mass2=request.mass2,
        velocity2=request.velocity2,
        coefficient_of_restitution=1.0,
    )

    inelastic_result = calculate_inelastic_collision_3d(inelastic_request)

    return ElasticCollision3DResponse(
        final_velocity1=inelastic_result.final_velocity1,
        final_velocity2=inelastic_result.final_velocity2,
        initial_momentum=inelastic_result.initial_momentum,
        final_momentum=inelastic_result.final_momentum,
        initial_kinetic_energy=inelastic_result.initial_kinetic_energy,
        final_kinetic_energy=inelastic_result.final_kinetic_energy,
    )
