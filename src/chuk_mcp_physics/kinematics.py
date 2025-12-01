"""2D Kinematics calculations.

Implements acceleration from position, jerk, trajectory fitting, and motion analysis.
"""

import math
from typing import Literal

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class AccelerationFromPositionRequest(BaseModel):
    """Request for acceleration calculation from position data."""

    positions: list[list[float]] = Field(
        ..., description="List of position vectors [x, y, z] in meters"
    )
    times: list[float] = Field(..., description="Time values in seconds")


class AccelerationFromPositionResponse(BaseModel):
    """Response for acceleration from position."""

    velocities: list[list[float]] = Field(
        ..., description="Calculated velocity vectors [x, y, z] in m/s"
    )
    accelerations: list[list[float]] = Field(
        ..., description="Calculated acceleration vectors [x, y, z] in m/s²"
    )
    average_velocity: list[float] = Field(..., description="Average velocity [x, y, z] in m/s")
    average_acceleration: list[float] = Field(
        ..., description="Average acceleration [x, y, z] in m/s²"
    )


class JerkCalculationRequest(BaseModel):
    """Request for jerk (rate of change of acceleration) calculation."""

    accelerations: list[list[float]] = Field(
        ..., description="List of acceleration vectors [x, y, z] in m/s²"
    )
    times: list[float] = Field(..., description="Time values in seconds")


class JerkCalculationResponse(BaseModel):
    """Response for jerk calculation."""

    jerks: list[list[float]] = Field(..., description="Calculated jerk vectors [x, y, z] in m/s³")
    average_jerk: list[float] = Field(..., description="Average jerk [x, y, z] in m/s³")
    max_jerk_magnitude: float = Field(..., description="Maximum jerk magnitude in m/s³")


class TrajectoryFitRequest(BaseModel):
    """Request for trajectory curve fitting."""

    positions: list[list[float]] = Field(
        ..., description="List of position vectors [x, y, z] in meters"
    )
    times: list[float] = Field(..., description="Time values in seconds")
    fit_type: Literal["linear", "quadratic", "cubic"] = Field(
        default="quadratic", description="Type of polynomial fit"
    )


class TrajectoryFitResponse(BaseModel):
    """Response for trajectory fitting."""

    coefficients_x: list[float] = Field(..., description="Polynomial coefficients for x(t)")
    coefficients_y: list[float] = Field(..., description="Polynomial coefficients for y(t)")
    coefficients_z: list[float] = Field(..., description="Polynomial coefficients for z(t)")
    r_squared: float = Field(..., description="R² goodness of fit (0-1)")
    predicted_positions: list[list[float]] = Field(
        ..., description="Fitted position values at input times"
    )


class MotionGraphRequest(BaseModel):
    """Request for motion graph data generation."""

    positions: list[list[float]] = Field(..., description="Position data [x, y, z] in meters")
    times: list[float] = Field(..., description="Time values in seconds")
    component: Literal["x", "y", "z", "magnitude"] = Field(
        default="magnitude", description="Which component to analyze"
    )


class MotionGraphResponse(BaseModel):
    """Response for motion graph data."""

    times: list[float] = Field(..., description="Time values in seconds")
    positions: list[float] = Field(..., description="Position values in meters")
    velocities: list[float] = Field(..., description="Velocity values in m/s")
    accelerations: list[float] = Field(..., description="Acceleration values in m/s²")
    component: str = Field(..., description="Component analyzed")


class AverageSpeedRequest(BaseModel):
    """Request for average speed calculation."""

    positions: list[list[float]] = Field(..., description="Position data [x, y, z] in meters")
    times: list[float] = Field(..., description="Time values in seconds")


class AverageSpeedResponse(BaseModel):
    """Response for average speed calculation."""

    average_speed: float = Field(..., description="Average speed in m/s")
    total_distance: float = Field(..., description="Total path length in meters")
    total_time: float = Field(..., description="Total time elapsed in seconds")
    displacement: list[float] = Field(
        ..., description="Net displacement vector [x, y, z] in meters"
    )
    displacement_magnitude: float = Field(..., description="Displacement magnitude in meters")


class InstantaneousVelocityRequest(BaseModel):
    """Request for instantaneous velocity at a specific time."""

    positions: list[list[float]] = Field(..., description="Position data [x, y, z] in meters")
    times: list[float] = Field(..., description="Time values in seconds")
    target_time: float = Field(..., description="Time at which to calculate velocity in seconds")


class InstantaneousVelocityResponse(BaseModel):
    """Response for instantaneous velocity."""

    velocity: list[float] = Field(..., description="Velocity at target time [x, y, z] in m/s")
    speed: float = Field(..., description="Speed magnitude in m/s")
    interpolated: bool = Field(..., description="Whether value was interpolated")


# ============================================================================
# Helper Functions
# ============================================================================


def _vector_magnitude(v: list[float]) -> float:
    """Calculate magnitude of a vector."""
    return math.sqrt(sum(x * x for x in v))


def _vector_subtract(a: list[float], b: list[float]) -> list[float]:
    """Subtract vector b from vector a."""
    return [x - y for x, y in zip(a, b)]


def _numerical_derivative(values: list[list[float]], times: list[float]) -> list[list[float]]:
    """Calculate numerical derivative using central differences."""
    if len(values) < 2:
        return [[0.0] * len(values[0])]

    derivatives = []

    for i in range(len(values)):
        if i == 0:
            # Forward difference for first point
            dt = times[1] - times[0]
            deriv = [(values[1][j] - values[0][j]) / dt for j in range(len(values[0]))]
        elif i == len(values) - 1:
            # Backward difference for last point
            dt = times[-1] - times[-2]
            deriv = [(values[-1][j] - values[-2][j]) / dt for j in range(len(values[0]))]
        else:
            # Central difference for interior points
            dt = times[i + 1] - times[i - 1]
            deriv = [(values[i + 1][j] - values[i - 1][j]) / dt for j in range(len(values[0]))]

        derivatives.append(deriv)

    return derivatives


def _polyfit(x: list[float], y: list[float], degree: int) -> list[float]:
    """Simple polynomial fitting using least squares.

    Returns coefficients [a0, a1, a2, ...] for y = a0 + a1*x + a2*x^2 + ...
    """
    n = len(x)
    if n < degree + 1:
        raise ValueError(f"Need at least {degree + 1} points for degree {degree} fit")

    # Build the Vandermonde matrix
    A = [[x[i] ** j for j in range(degree + 1)] for i in range(n)]

    # Solve using normal equations: (A^T A) c = A^T y
    # This is a simplified implementation - not numerically stable for large degrees
    AtA = [
        [sum(A[k][i] * A[k][j] for k in range(n)) for j in range(degree + 1)]
        for i in range(degree + 1)
    ]
    Aty = [sum(A[k][i] * y[k] for k in range(n)) for i in range(degree + 1)]

    # Gaussian elimination
    coeffs = _gaussian_elimination(AtA, Aty)
    return coeffs


def _gaussian_elimination(A: list[list[float]], b: list[float]) -> list[float]:
    """Solve Ax = b using Gaussian elimination."""
    n = len(b)
    # Create augmented matrix
    aug = [A[i][:] + [b[i]] for i in range(n)]

    # Forward elimination
    for i in range(n):
        # Find pivot
        max_row = i
        for k in range(i + 1, n):
            if abs(aug[k][i]) > abs(aug[max_row][i]):
                max_row = k
        aug[i], aug[max_row] = aug[max_row], aug[i]

        # Eliminate below
        for k in range(i + 1, n):
            if aug[i][i] == 0:
                continue
            factor = aug[k][i] / aug[i][i]
            for j in range(i, n + 1):
                aug[k][j] -= factor * aug[i][j]

    # Back substitution
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = aug[i][n]
        for j in range(i + 1, n):
            x[i] -= aug[i][j] * x[j]
        if aug[i][i] != 0:
            x[i] /= aug[i][i]

    return x


def _calculate_r_squared(actual: list[float], predicted: list[float]) -> float:
    """Calculate R² goodness of fit."""
    if len(actual) != len(predicted):
        return 0.0

    mean_actual = sum(actual) / len(actual)
    ss_tot = sum((y - mean_actual) ** 2 for y in actual)
    ss_res = sum((actual[i] - predicted[i]) ** 2 for i in range(len(actual)))

    if ss_tot == 0:
        return 1.0 if ss_res == 0 else 0.0

    return 1.0 - (ss_res / ss_tot)


# ============================================================================
# Calculation Functions
# ============================================================================


def calculate_acceleration_from_position(
    request: AccelerationFromPositionRequest,
) -> AccelerationFromPositionResponse:
    """Calculate velocity and acceleration from position data using numerical differentiation.

    Args:
        request: Position and time data

    Returns:
        Velocities and accelerations
    """
    if len(request.positions) != len(request.times):
        raise ValueError("Number of positions must equal number of times")

    if len(request.positions) < 3:
        raise ValueError("Need at least 3 data points for acceleration calculation")

    # Calculate velocities
    velocities = _numerical_derivative(request.positions, request.times)

    # Calculate accelerations
    accelerations = _numerical_derivative(velocities, request.times)

    # Calculate averages
    avg_vel = [sum(v[i] for v in velocities) / len(velocities) for i in range(3)]
    avg_acc = [sum(a[i] for a in accelerations) / len(accelerations) for i in range(3)]

    return AccelerationFromPositionResponse(
        velocities=velocities,
        accelerations=accelerations,
        average_velocity=avg_vel,
        average_acceleration=avg_acc,
    )


def calculate_jerk(request: JerkCalculationRequest) -> JerkCalculationResponse:
    """Calculate jerk (da/dt) from acceleration data.

    Jerk is the rate of change of acceleration.

    Args:
        request: Acceleration and time data

    Returns:
        Jerk values
    """
    if len(request.accelerations) != len(request.times):
        raise ValueError("Number of accelerations must equal number of times")

    if len(request.accelerations) < 2:
        raise ValueError("Need at least 2 data points for jerk calculation")

    # Calculate jerks
    jerks = _numerical_derivative(request.accelerations, request.times)

    # Calculate average and max
    avg_jerk = [sum(j[i] for j in jerks) / len(jerks) for i in range(3)]
    max_jerk = max(_vector_magnitude(j) for j in jerks)

    return JerkCalculationResponse(
        jerks=jerks,
        average_jerk=avg_jerk,
        max_jerk_magnitude=max_jerk,
    )


def fit_trajectory(request: TrajectoryFitRequest) -> TrajectoryFitResponse:
    """Fit polynomial curves to trajectory data.

    Args:
        request: Position and time data with fit type

    Returns:
        Polynomial coefficients and fitted values
    """
    if len(request.positions) != len(request.times):
        raise ValueError("Number of positions must equal number of times")

    degree_map = {"linear": 1, "quadratic": 2, "cubic": 3}
    degree = degree_map[request.fit_type]

    if len(request.positions) < degree + 1:
        raise ValueError(f"Need at least {degree + 1} points for {request.fit_type} fit")

    # Extract x, y, z components
    x_vals = [p[0] for p in request.positions]
    y_vals = [p[1] for p in request.positions]
    z_vals = [p[2] for p in request.positions]

    # Fit polynomials
    coeffs_x = _polyfit(request.times, x_vals, degree)
    coeffs_y = _polyfit(request.times, y_vals, degree)
    coeffs_z = _polyfit(request.times, z_vals, degree)

    # Calculate predicted positions
    predicted = []
    for t in request.times:
        x_pred = sum(coeffs_x[i] * (t**i) for i in range(len(coeffs_x)))
        y_pred = sum(coeffs_y[i] * (t**i) for i in range(len(coeffs_y)))
        z_pred = sum(coeffs_z[i] * (t**i) for i in range(len(coeffs_z)))
        predicted.append([x_pred, y_pred, z_pred])

    # Calculate R² for each component and average
    r2_x = _calculate_r_squared(x_vals, [p[0] for p in predicted])
    r2_y = _calculate_r_squared(y_vals, [p[1] for p in predicted])
    r2_z = _calculate_r_squared(z_vals, [p[2] for p in predicted])
    r_squared = (r2_x + r2_y + r2_z) / 3.0

    return TrajectoryFitResponse(
        coefficients_x=coeffs_x,
        coefficients_y=coeffs_y,
        coefficients_z=coeffs_z,
        r_squared=r_squared,
        predicted_positions=predicted,
    )


def generate_motion_graph(request: MotionGraphRequest) -> MotionGraphResponse:
    """Generate motion graph data (position, velocity, acceleration vs time).

    Args:
        request: Motion graph generation request

    Returns:
        Graph data for specified component
    """
    if len(request.positions) != len(request.times):
        raise ValueError("Number of positions must equal number of times")

    # Get component index
    component_map = {"x": 0, "y": 1, "z": 2}

    # Calculate derivatives
    velocities_vec = _numerical_derivative(request.positions, request.times)
    accelerations_vec = _numerical_derivative(velocities_vec, request.times)

    if request.component == "magnitude":
        positions = [_vector_magnitude(p) for p in request.positions]
        velocities = [_vector_magnitude(v) for v in velocities_vec]
        accelerations = [_vector_magnitude(a) for a in accelerations_vec]
    else:
        idx = component_map[request.component]
        positions = [p[idx] for p in request.positions]
        velocities = [v[idx] for v in velocities_vec]
        accelerations = [a[idx] for a in accelerations_vec]

    return MotionGraphResponse(
        times=request.times,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
        component=request.component,
    )


def calculate_average_speed(request: AverageSpeedRequest) -> AverageSpeedResponse:
    """Calculate average speed (total distance / total time).

    Average speed is different from average velocity - speed uses total path length.

    Args:
        request: Average speed request

    Returns:
        Average speed and distance statistics
    """
    if len(request.positions) != len(request.times):
        raise ValueError("Number of positions must equal number of times")

    if len(request.positions) < 2:
        raise ValueError("Need at least 2 positions")

    # Calculate total distance (path length)
    total_distance = 0.0
    for i in range(1, len(request.positions)):
        delta = _vector_subtract(request.positions[i], request.positions[i - 1])
        total_distance += _vector_magnitude(delta)

    # Calculate displacement
    displacement = _vector_subtract(request.positions[-1], request.positions[0])
    displacement_mag = _vector_magnitude(displacement)

    # Total time
    total_time = request.times[-1] - request.times[0]

    # Average speed
    average_speed = total_distance / total_time if total_time > 0 else 0.0

    return AverageSpeedResponse(
        average_speed=average_speed,
        total_distance=total_distance,
        total_time=total_time,
        displacement=displacement,
        displacement_magnitude=displacement_mag,
    )


def calculate_instantaneous_velocity(
    request: InstantaneousVelocityRequest,
) -> InstantaneousVelocityResponse:
    """Calculate instantaneous velocity at a specific time.

    Uses linear interpolation if target time is between data points.

    Args:
        request: Instantaneous velocity request

    Returns:
        Velocity at target time
    """
    if len(request.positions) != len(request.times):
        raise ValueError("Number of positions must equal number of times")

    # Calculate all velocities
    velocities = _numerical_derivative(request.positions, request.times)

    # Find closest time indices
    interpolated = False
    velocity = [0.0, 0.0, 0.0]

    if request.target_time in request.times:
        # Exact match
        idx = request.times.index(request.target_time)
        velocity = velocities[idx]
    else:
        # Interpolate
        interpolated = True
        for i in range(len(request.times) - 1):
            if request.times[i] <= request.target_time <= request.times[i + 1]:
                # Linear interpolation
                t0, t1 = request.times[i], request.times[i + 1]
                v0, v1 = velocities[i], velocities[i + 1]
                alpha = (request.target_time - t0) / (t1 - t0)
                velocity = [v0[j] + alpha * (v1[j] - v0[j]) for j in range(3)]
                break

    speed = _vector_magnitude(velocity)

    return InstantaneousVelocityResponse(
        velocity=velocity,
        speed=speed,
        interpolated=interpolated,
    )
