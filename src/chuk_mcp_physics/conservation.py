"""Conservation laws verification and tracking.

Tools to verify conservation of energy, momentum, and angular momentum over time.
"""

import math

from pydantic import BaseModel, Field

from .models import TrajectoryFrame


# ============================================================================
# Request/Response Models
# ============================================================================


class EnergyConservationCheckRequest(BaseModel):
    """Request for energy conservation verification."""

    initial_kinetic_energy: float = Field(..., description="Initial kinetic energy in Joules")
    final_kinetic_energy: float = Field(..., description="Final kinetic energy in Joules")
    initial_potential_energy: float = Field(..., description="Initial potential energy in Joules")
    final_potential_energy: float = Field(..., description="Final potential energy in Joules")
    expected_energy_loss: float = Field(
        default=0.0, description="Expected energy loss (e.g., from friction/damping) in Joules"
    )
    tolerance: float = Field(
        default=0.01, description="Tolerance for conservation check (fraction)", ge=0.0
    )


class EnergyConservationCheckResponse(BaseModel):
    """Response for energy conservation verification."""

    initial_total_energy: float = Field(..., description="Initial total energy in Joules")
    final_total_energy: float = Field(..., description="Final total energy in Joules")
    energy_difference: float = Field(..., description="Energy difference in Joules")
    energy_difference_percent: float = Field(..., description="Energy difference as percentage")
    is_conserved: bool = Field(..., description="Whether energy is conserved within tolerance")
    expected_loss: float = Field(..., description="Expected energy loss in Joules")
    actual_loss: float = Field(..., description="Actual energy loss in Joules")


class MomentumConservationCheckRequest(BaseModel):
    """Request for momentum conservation verification."""

    initial_momentum: list[float] = Field(
        ..., description="Initial total momentum [x, y, z] in kg⋅m/s"
    )
    final_momentum: list[float] = Field(..., description="Final total momentum [x, y, z] in kg⋅m/s")
    tolerance: float = Field(
        default=0.01, description="Tolerance for conservation check (fraction)", ge=0.0
    )


class MomentumConservationCheckResponse(BaseModel):
    """Response for momentum conservation verification."""

    initial_momentum_magnitude: float = Field(
        ..., description="Initial momentum magnitude in kg⋅m/s"
    )
    final_momentum_magnitude: float = Field(..., description="Final momentum magnitude in kg⋅m/s")
    momentum_difference: list[float] = Field(
        ..., description="Momentum difference [x, y, z] in kg⋅m/s"
    )
    momentum_difference_magnitude: float = Field(
        ..., description="Momentum difference magnitude in kg⋅m/s"
    )
    momentum_difference_percent: float = Field(..., description="Momentum difference as percentage")
    is_conserved: bool = Field(..., description="Whether momentum is conserved within tolerance")


class AngularMomentumConservationCheckRequest(BaseModel):
    """Request for angular momentum conservation verification."""

    initial_angular_momentum: list[float] = Field(
        ..., description="Initial angular momentum [x, y, z] in kg⋅m²/s"
    )
    final_angular_momentum: list[float] = Field(
        ..., description="Final angular momentum [x, y, z] in kg⋅m²/s"
    )
    tolerance: float = Field(
        default=0.01, description="Tolerance for conservation check (fraction)", ge=0.0
    )


class AngularMomentumConservationCheckResponse(BaseModel):
    """Response for angular momentum conservation verification."""

    initial_L_magnitude: float = Field(
        ..., description="Initial angular momentum magnitude in kg⋅m²/s"
    )
    final_L_magnitude: float = Field(..., description="Final angular momentum magnitude in kg⋅m²/s")
    L_difference: list[float] = Field(
        ..., description="Angular momentum difference [x, y, z] in kg⋅m²/s"
    )
    L_difference_magnitude: float = Field(
        ..., description="Angular momentum difference magnitude in kg⋅m²/s"
    )
    L_difference_percent: float = Field(
        ..., description="Angular momentum difference as percentage"
    )
    is_conserved: bool = Field(
        ..., description="Whether angular momentum is conserved within tolerance"
    )


class EnergyDissipationTrackingRequest(BaseModel):
    """Request for energy dissipation tracking over trajectory."""

    frames: list[TrajectoryFrame] = Field(..., description="Trajectory frames to analyze")
    mass: float = Field(..., description="Object mass in kg", gt=0.0)
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)
    reference_height: float = Field(
        default=0.0, description="Reference height for potential energy in meters"
    )


class EnergyDissipationFrame(BaseModel):
    """Energy data for a single frame."""

    time: float = Field(..., description="Time in seconds")
    kinetic_energy: float = Field(..., description="Kinetic energy in Joules")
    potential_energy: float = Field(..., description="Potential energy in Joules")
    total_energy: float = Field(..., description="Total mechanical energy in Joules")


class EnergyDissipationTrackingResponse(BaseModel):
    """Response for energy dissipation tracking."""

    frames: list[EnergyDissipationFrame] = Field(..., description="Energy data for each frame")
    initial_total_energy: float = Field(..., description="Initial total energy in Joules")
    final_total_energy: float = Field(..., description="Final total energy in Joules")
    total_energy_loss: float = Field(..., description="Total energy dissipated in Joules")
    total_energy_loss_percent: float = Field(..., description="Total energy loss as percentage")
    average_power_dissipated: float = Field(
        ..., description="Average power dissipated in Watts (J/s)"
    )


# ============================================================================
# Helper Functions
# ============================================================================


def _vector_magnitude(v: list[float]) -> float:
    """Calculate magnitude of a vector."""
    return math.sqrt(sum(x * x for x in v))


def _vector_subtract(a: list[float], b: list[float]) -> list[float]:
    """Subtract vector b from vector a."""
    return [x - y for x, y in zip(a, b)]


# ============================================================================
# Calculation Functions
# ============================================================================


def check_energy_conservation(
    request: EnergyConservationCheckRequest,
) -> EnergyConservationCheckResponse:
    """Verify conservation of energy.

    Args:
        request: Energy conservation check request

    Returns:
        Energy conservation analysis
    """
    E_initial = request.initial_kinetic_energy + request.initial_potential_energy
    E_final = request.final_kinetic_energy + request.final_potential_energy

    E_diff = E_initial - E_final
    E_diff_percent = (abs(E_diff) / E_initial * 100.0) if E_initial > 0 else 0.0

    # Account for expected energy loss
    actual_loss = E_diff
    expected_loss = request.expected_energy_loss

    # Check if conserved within tolerance
    is_conserved = abs(E_diff - expected_loss) <= request.tolerance * E_initial

    return EnergyConservationCheckResponse(
        initial_total_energy=E_initial,
        final_total_energy=E_final,
        energy_difference=E_diff,
        energy_difference_percent=E_diff_percent,
        is_conserved=is_conserved,
        expected_loss=expected_loss,
        actual_loss=actual_loss,
    )


def check_momentum_conservation(
    request: MomentumConservationCheckRequest,
) -> MomentumConservationCheckResponse:
    """Verify conservation of momentum.

    Args:
        request: Momentum conservation check request

    Returns:
        Momentum conservation analysis
    """
    p_initial = request.initial_momentum
    p_final = request.final_momentum

    p_initial_mag = _vector_magnitude(p_initial)
    p_final_mag = _vector_magnitude(p_final)

    p_diff = _vector_subtract(p_initial, p_final)
    p_diff_mag = _vector_magnitude(p_diff)

    p_diff_percent = (p_diff_mag / p_initial_mag * 100.0) if p_initial_mag > 0 else 0.0

    is_conserved = p_diff_mag <= request.tolerance * p_initial_mag

    return MomentumConservationCheckResponse(
        initial_momentum_magnitude=p_initial_mag,
        final_momentum_magnitude=p_final_mag,
        momentum_difference=p_diff,
        momentum_difference_magnitude=p_diff_mag,
        momentum_difference_percent=p_diff_percent,
        is_conserved=is_conserved,
    )


def check_angular_momentum_conservation(
    request: AngularMomentumConservationCheckRequest,
) -> AngularMomentumConservationCheckResponse:
    """Verify conservation of angular momentum.

    Args:
        request: Angular momentum conservation check request

    Returns:
        Angular momentum conservation analysis
    """
    L_initial = request.initial_angular_momentum
    L_final = request.final_angular_momentum

    L_initial_mag = _vector_magnitude(L_initial)
    L_final_mag = _vector_magnitude(L_final)

    L_diff = _vector_subtract(L_initial, L_final)
    L_diff_mag = _vector_magnitude(L_diff)

    L_diff_percent = (L_diff_mag / L_initial_mag * 100.0) if L_initial_mag > 0 else 0.0

    is_conserved = L_diff_mag <= request.tolerance * L_initial_mag

    return AngularMomentumConservationCheckResponse(
        initial_L_magnitude=L_initial_mag,
        final_L_magnitude=L_final_mag,
        L_difference=L_diff,
        L_difference_magnitude=L_diff_mag,
        L_difference_percent=L_diff_percent,
        is_conserved=is_conserved,
    )


def track_energy_dissipation(
    request: EnergyDissipationTrackingRequest,
) -> EnergyDissipationTrackingResponse:
    """Track energy dissipation over a trajectory.

    Args:
        request: Energy dissipation tracking request

    Returns:
        Energy dissipation analysis
    """
    m = request.mass
    g = request.gravity
    h_ref = request.reference_height

    energy_frames: list[EnergyDissipationFrame] = []

    for frame in request.frames:
        # Kinetic energy
        if frame.velocity:
            v_mag = _vector_magnitude(frame.velocity)
            ke = 0.5 * m * v_mag * v_mag
        else:
            ke = 0.0

        # Potential energy (using y-coordinate as height)
        h = frame.position[1] - h_ref
        pe = m * g * h

        # Total energy
        E_total = ke + pe

        energy_frames.append(
            EnergyDissipationFrame(
                time=frame.time, kinetic_energy=ke, potential_energy=pe, total_energy=E_total
            )
        )

    # Calculate total energy loss
    E_initial = energy_frames[0].total_energy
    E_final = energy_frames[-1].total_energy
    E_loss = E_initial - E_final
    E_loss_percent = (E_loss / E_initial * 100.0) if E_initial > 0 else 0.0

    # Calculate average power dissipated
    total_time = energy_frames[-1].time - energy_frames[0].time
    avg_power = E_loss / total_time if total_time > 0 else 0.0

    return EnergyDissipationTrackingResponse(
        frames=energy_frames,
        initial_total_energy=E_initial,
        final_total_energy=E_final,
        total_energy_loss=E_loss,
        total_energy_loss_percent=E_loss_percent,
        average_power_dissipated=avg_power,
    )
