"""Rotational dynamics MCP tool endpoints."""

from typing import Optional

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def calculate_torque(
    force_x: float,
    force_y: float,
    force_z: float,
    position_x: float,
    position_y: float,
    position_z: float,
) -> dict:
    """Calculate torque from force and position: τ = r × F (cross product).

    Torque is the rotational equivalent of force. It causes angular acceleration
    and depends on both the force magnitude and the distance from the pivot point.

    Args:
        force_x: X component of force in Newtons
        force_y: Y component of force in Newtons
        force_z: Z component of force in Newtons
        position_x: X component of position vector from pivot to force application (meters)
        position_y: Y component of position vector from pivot to force application (meters)
        position_z: Z component of position vector from pivot to force application (meters)

    Returns:
        Dict containing:
            - torque: Torque vector [x, y, z] in N⋅m
            - magnitude: Torque magnitude in N⋅m

    Tips for LLMs:
        - Torque direction follows right-hand rule (perpendicular to force and position)
        - Maximum torque when force is perpendicular to position vector
        - Zero torque when force is parallel to position vector
        - Use for: wrenches, door hinges, motors, gears

    Example - Opening a door:
        result = await calculate_torque(
            force_x=50.0,  # Push perpendicular to door
            force_y=0.0,
            force_z=0.0,
            position_x=0.0,
            position_y=0.0,
            position_z=0.8  # 0.8m from hinge
        )
        # Torque = 40 N⋅m
    """
    from ..rotational import TorqueRequest, calculate_torque as calc_torque

    request = TorqueRequest(
        force=[force_x, force_y, force_z],
        position=[position_x, position_y, position_z],
    )
    response = calc_torque(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_moment_of_inertia(
    shape: str,
    mass: float,
    radius: Optional[float] = None,
    length: Optional[float] = None,
    width: Optional[float] = None,
    height: Optional[float] = None,
    depth: Optional[float] = None,
    axis: str = "center",
) -> dict:
    """Calculate moment of inertia for various shapes.

    Moment of inertia (I) is the rotational equivalent of mass. It determines
    how difficult it is to change an object's rotation. Depends on both mass
    distribution and rotation axis.

    Args:
        shape: Shape type - "sphere", "solid_sphere", "hollow_sphere", "rod", "disk", "cylinder", "box"
        mass: Mass in kilograms
        radius: Radius for sphere/disk/cylinder (meters)
        length: Length for rod (meters)
        width: Width for box (meters)
        height: Height for box/cylinder (meters)
        depth: Depth for box (meters)
        axis: Rotation axis - "center", "end" (for rod), "x", "y", "z" (for box)

    Returns:
        Dict containing:
            - moment_of_inertia: I in kg⋅m²
            - shape: Shape type
            - axis: Rotation axis

    Common formulas:
        - Solid sphere (center): I = (2/5)mr²
        - Hollow sphere (center): I = (2/3)mr²
        - Rod (center): I = (1/12)mL²
        - Rod (end): I = (1/3)mL²
        - Disk (center): I = (1/2)mr²

    Example - Spinning wheel:
        result = await calculate_moment_of_inertia(
            shape="disk",
            mass=5.0,  # 5kg wheel
            radius=0.3  # 30cm radius
        )
        # I = 0.225 kg⋅m²
    """
    from ..rotational import MomentOfInertiaRequest, calculate_moment_of_inertia as calc_moi

    request = MomentOfInertiaRequest(
        shape=shape,  # type: ignore
        mass=mass,
        radius=radius,
        length=length,
        width=width,
        height=height,
        depth=depth,
        axis=axis,
    )
    response = calc_moi(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_angular_momentum(
    moment_of_inertia: float,
    angular_velocity_x: float,
    angular_velocity_y: float,
    angular_velocity_z: float,
) -> dict:
    """Calculate angular momentum: L = I × ω.

    Angular momentum is the rotational equivalent of linear momentum.
    It's conserved in the absence of external torques (like ice skater spinning).

    Args:
        moment_of_inertia: Moment of inertia in kg⋅m²
        angular_velocity_x: X component of angular velocity in rad/s
        angular_velocity_y: Y component of angular velocity in rad/s
        angular_velocity_z: Z component of angular velocity in rad/s

    Returns:
        Dict containing:
            - angular_momentum: L vector [x, y, z] in kg⋅m²/s
            - magnitude: L magnitude in kg⋅m²/s

    Tips for LLMs:
        - Angular momentum is conserved when no external torques act
        - Ice skater pulls arms in → I decreases → ω increases (L constant)
        - Gyroscopes resist changes in angular momentum direction

    Example - Spinning figure skater:
        # Arms extended: I = 3.0 kg⋅m², ω = 5 rad/s
        result = await calculate_angular_momentum(
            moment_of_inertia=3.0,
            angular_velocity_x=0.0,
            angular_velocity_y=5.0,
            angular_velocity_z=0.0
        )
        # L = 15 kg⋅m²/s (conserved when arms pulled in)
    """
    from ..rotational import AngularMomentumRequest, calculate_angular_momentum as calc_L

    request = AngularMomentumRequest(
        moment_of_inertia=moment_of_inertia,
        angular_velocity=[angular_velocity_x, angular_velocity_y, angular_velocity_z],
    )
    response = calc_L(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_rotational_kinetic_energy(
    moment_of_inertia: float,
    angular_velocity: float,
) -> dict:
    """Calculate rotational kinetic energy: KE_rot = (1/2) I ω².

    Energy of rotation. A spinning object has kinetic energy even if
    its center of mass is stationary.

    Args:
        moment_of_inertia: Moment of inertia in kg⋅m²
        angular_velocity: Angular velocity magnitude in rad/s

    Returns:
        Dict containing:
            - rotational_ke: Rotational kinetic energy in Joules

    Tips for LLMs:
        - Total KE = translational KE + rotational KE
        - Rolling object has both types of kinetic energy
        - Flywheel energy storage uses this principle

    Example - Car wheel at highway speed:
        result = await calculate_rotational_kinetic_energy(
            moment_of_inertia=0.5,  # kg⋅m²
            angular_velocity=100.0  # rad/s (fast spinning)
        )
        # KE_rot = 2500 J
    """
    from ..rotational import (
        RotationalKineticEnergyRequest,
        calculate_rotational_kinetic_energy as calc_ke_rot,
    )

    request = RotationalKineticEnergyRequest(
        moment_of_inertia=moment_of_inertia,
        angular_velocity=angular_velocity,
    )
    response = calc_ke_rot(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_angular_acceleration(
    torque: float,
    moment_of_inertia: float,
) -> dict:
    """Calculate angular acceleration: α = τ / I.

    Angular acceleration is the rotational equivalent of linear acceleration.
    Determined by net torque and moment of inertia.

    Args:
        torque: Torque magnitude in N⋅m
        moment_of_inertia: Moment of inertia in kg⋅m²

    Returns:
        Dict containing:
            - angular_acceleration: α in rad/s²

    Tips for LLMs:
        - Rotational version of F = ma → τ = Iα
        - Larger I means slower angular acceleration for same torque
        - Use for: motor acceleration, spinning up flywheels

    Example - Motor accelerating a wheel:
        result = await calculate_angular_acceleration(
            torque=10.0,  # N⋅m
            moment_of_inertia=0.5  # kg⋅m²
        )
        # α = 20 rad/s²
    """
    from ..rotational import (
        AngularAccelerationRequest,
        calculate_angular_acceleration as calc_alpha,
    )

    request = AngularAccelerationRequest(
        torque=torque,
        moment_of_inertia=moment_of_inertia,
    )
    response = calc_alpha(request)
    return response.model_dump()
