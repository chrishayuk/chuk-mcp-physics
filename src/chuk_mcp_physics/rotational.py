"""Rotational dynamics calculations.

Implements torque, moment of inertia, angular momentum, and rotational kinematics.
"""

import math
from typing import Literal

from pydantic import BaseModel, Field


# ============================================================================
# Request/Response Models
# ============================================================================


class TorqueRequest(BaseModel):
    """Request for torque calculation."""

    force: list[float] = Field(..., description="Force vector [x, y, z] in Newtons")
    position: list[float] = Field(
        ..., description="Position vector from pivot to force application [x, y, z] in meters"
    )


class TorqueResponse(BaseModel):
    """Response for torque calculation."""

    torque: list[float] = Field(..., description="Torque vector [x, y, z] in N⋅m")
    magnitude: float = Field(..., description="Torque magnitude in N⋅m")


class MomentOfInertiaRequest(BaseModel):
    """Request for moment of inertia calculation."""

    shape: Literal["sphere", "solid_sphere", "hollow_sphere", "rod", "disk", "cylinder", "box"]
    mass: float = Field(..., description="Mass in kg", gt=0.0)
    # Dimensions depend on shape
    radius: float | None = Field(None, description="Radius for sphere/disk/cylinder (meters)")
    length: float | None = Field(None, description="Length for rod (meters)")
    width: float | None = Field(None, description="Width for box (meters)")
    height: float | None = Field(None, description="Height for box/cylinder (meters)")
    depth: float | None = Field(None, description="Depth for box (meters)")
    axis: str = Field(
        default="center",
        description="Rotation axis: 'center', 'end' (for rod), 'x', 'y', 'z' (for box)",
    )


class MomentOfInertiaResponse(BaseModel):
    """Response for moment of inertia calculation."""

    moment_of_inertia: float = Field(..., description="Moment of inertia in kg⋅m²")
    shape: str = Field(..., description="Shape type")
    axis: str = Field(..., description="Rotation axis")


class AngularMomentumRequest(BaseModel):
    """Request for angular momentum calculation."""

    moment_of_inertia: float = Field(..., description="Moment of inertia in kg⋅m²", gt=0.0)
    angular_velocity: list[float] = Field(..., description="Angular velocity [x, y, z] in rad/s")


class AngularMomentumResponse(BaseModel):
    """Response for angular momentum calculation."""

    angular_momentum: list[float] = Field(
        ..., description="Angular momentum vector [x, y, z] in kg⋅m²/s"
    )
    magnitude: float = Field(..., description="Angular momentum magnitude in kg⋅m²/s")


class RotationalKineticEnergyRequest(BaseModel):
    """Request for rotational kinetic energy calculation."""

    moment_of_inertia: float = Field(..., description="Moment of inertia in kg⋅m²", gt=0.0)
    angular_velocity: float = Field(..., description="Angular velocity magnitude in rad/s")


class RotationalKineticEnergyResponse(BaseModel):
    """Response for rotational kinetic energy calculation."""

    rotational_ke: float = Field(..., description="Rotational kinetic energy in Joules")


class AngularAccelerationRequest(BaseModel):
    """Request for angular acceleration calculation."""

    torque: float = Field(..., description="Torque magnitude in N⋅m")
    moment_of_inertia: float = Field(..., description="Moment of inertia in kg⋅m²", gt=0.0)


class AngularAccelerationResponse(BaseModel):
    """Response for angular acceleration calculation."""

    angular_acceleration: float = Field(..., description="Angular acceleration in rad/s²")


# ============================================================================
# Calculation Functions
# ============================================================================


def calculate_torque(request: TorqueRequest) -> TorqueResponse:
    """Calculate torque from force and position: τ = r × F (cross product).

    Args:
        request: Torque calculation request

    Returns:
        Torque vector and magnitude
    """
    r = request.position
    f = request.force

    # Cross product: r × F
    torque = [
        r[1] * f[2] - r[2] * f[1],  # x component
        r[2] * f[0] - r[0] * f[2],  # y component
        r[0] * f[1] - r[1] * f[0],  # z component
    ]

    magnitude = math.sqrt(sum(t * t for t in torque))

    return TorqueResponse(torque=torque, magnitude=magnitude)


def calculate_moment_of_inertia(request: MomentOfInertiaRequest) -> MomentOfInertiaResponse:
    """Calculate moment of inertia for various shapes.

    Formulas:
    - Solid sphere (center): I = (2/5) * m * r²
    - Hollow sphere (center): I = (2/3) * m * r²
    - Rod (center): I = (1/12) * m * L²
    - Rod (end): I = (1/3) * m * L²
    - Disk (center): I = (1/2) * m * r²
    - Cylinder (center): I = (1/2) * m * r²
    - Box (about center, axis through x): I = (1/12) * m * (h² + d²)

    Args:
        request: Moment of inertia request

    Returns:
        Moment of inertia value
    """
    m = request.mass
    shape = request.shape
    axis = request.axis

    if shape in ["sphere", "solid_sphere"]:
        if request.radius is None:
            raise ValueError("radius required for sphere")
        r = request.radius
        inertia = (2.0 / 5.0) * m * r * r

    elif shape == "hollow_sphere":
        if request.radius is None:
            raise ValueError("radius required for hollow sphere")
        r = request.radius
        inertia = (2.0 / 3.0) * m * r * r

    elif shape == "rod":
        if request.length is None:
            raise ValueError("length required for rod")
        L = request.length
        if axis == "end":
            inertia = (1.0 / 3.0) * m * L * L
        else:  # center
            inertia = (1.0 / 12.0) * m * L * L

    elif shape in ["disk", "cylinder"]:
        if request.radius is None:
            raise ValueError("radius required for disk/cylinder")
        r = request.radius
        inertia = (1.0 / 2.0) * m * r * r

    elif shape == "box":
        if request.width is None or request.height is None or request.depth is None:
            raise ValueError("width, height, depth required for box")
        w, h, d = request.width, request.height, request.depth

        # Rotation about different axes
        if axis == "x":
            inertia = (1.0 / 12.0) * m * (h * h + d * d)
        elif axis == "y":
            inertia = (1.0 / 12.0) * m * (w * w + d * d)
        elif axis == "z":
            inertia = (1.0 / 12.0) * m * (w * w + h * h)
        else:  # default to z-axis
            inertia = (1.0 / 12.0) * m * (w * w + h * h)

    else:
        raise ValueError(f"Unknown shape: {shape}")

    return MomentOfInertiaResponse(moment_of_inertia=inertia, shape=shape, axis=axis)


def calculate_angular_momentum(request: AngularMomentumRequest) -> AngularMomentumResponse:
    """Calculate angular momentum: L = I × ω.

    For simplicity, assumes angular velocity is about a principal axis.

    Args:
        request: Angular momentum request

    Returns:
        Angular momentum vector and magnitude
    """
    inertia = request.moment_of_inertia
    omega = request.angular_velocity

    # L = I * ω (scalar multiplication for simple case)
    L = [inertia * w for w in omega]
    magnitude = math.sqrt(sum(L_component * L_component for L_component in L))

    return AngularMomentumResponse(angular_momentum=L, magnitude=magnitude)


def calculate_rotational_kinetic_energy(
    request: RotationalKineticEnergyRequest,
) -> RotationalKineticEnergyResponse:
    """Calculate rotational kinetic energy: KE_rot = (1/2) * I * ω².

    Args:
        request: Rotational KE request

    Returns:
        Rotational kinetic energy
    """
    inertia = request.moment_of_inertia
    omega = request.angular_velocity
    KE_rot = 0.5 * inertia * omega * omega

    return RotationalKineticEnergyResponse(rotational_ke=KE_rot)


def calculate_angular_acceleration(
    request: AngularAccelerationRequest,
) -> AngularAccelerationResponse:
    """Calculate angular acceleration: α = τ / I.

    Args:
        request: Angular acceleration request

    Returns:
        Angular acceleration
    """
    tau = request.torque
    inertia = request.moment_of_inertia
    alpha = tau / inertia

    return AngularAccelerationResponse(angular_acceleration=alpha)
