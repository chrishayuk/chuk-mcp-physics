"""Fluid dynamics calculations for drag, buoyancy, and underwater motion.

This module provides analytical calculations for fluid dynamics effects including:
- Drag force (quadratic air/water resistance)
- Buoyancy force (Archimedes principle)
- Terminal velocity
- Underwater projectile motion with drag and buoyancy

These calculations are based on classical fluid mechanics and assume:
- Incompressible fluids
- Steady-state flow
- Spherical or specified drag coefficients
"""

import math

from .models import (
    BuoyancyRequest,
    BuoyancyResponse,
    DragForceRequest,
    DragForceResponse,
    TerminalVelocityRequest,
    TerminalVelocityResponse,
    UnderwaterMotionRequest,
    UnderwaterMotionResponse,
)


def calculate_drag_force(request: DragForceRequest) -> DragForceResponse:
    """Calculate drag force using quadratic drag equation.

    The drag force is given by:
        F_drag = 0.5 * ρ * v² * C_d * A

    Where:
        ρ = fluid density (kg/m³)
        v = velocity magnitude (m/s)
        C_d = drag coefficient (dimensionless)
        A = cross-sectional area (m²)

    The force opposes the direction of motion.

    Args:
        request: Drag force calculation parameters

    Returns:
        Drag force vector and Reynolds number
    """
    vx, vy, vz = request.velocity
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    if speed < 1e-10:
        # No motion, no drag
        return DragForceResponse(
            drag_force=[0.0, 0.0, 0.0],
            magnitude=0.0,
            reynolds_number=0.0,
        )

    # Drag force magnitude: F = 0.5 * ρ * v² * C_d * A
    drag_magnitude = (
        0.5
        * request.fluid_density
        * speed
        * speed
        * request.drag_coefficient
        * request.cross_sectional_area
    )

    # Direction opposite to velocity
    drag_force = [
        -drag_magnitude * vx / speed,
        -drag_magnitude * vy / speed,
        -drag_magnitude * vz / speed,
    ]

    # Calculate Reynolds number for flow regime estimation
    # Re = ρ * v * L / μ where L is characteristic length (use √A as approximation)
    characteristic_length = math.sqrt(request.cross_sectional_area)
    # We need viscosity for Reynolds number - using a default for now
    # This is a limitation - ideally we'd pass FluidEnvironment instead
    assumed_viscosity = 1.0e-3 if request.fluid_density > 100 else 1.8e-5
    reynolds_number = request.fluid_density * speed * characteristic_length / assumed_viscosity

    return DragForceResponse(
        drag_force=drag_force,
        magnitude=drag_magnitude,
        reynolds_number=reynolds_number,
    )


def calculate_buoyancy(request: BuoyancyRequest) -> BuoyancyResponse:
    """Calculate buoyancy force using Archimedes' principle.

    The buoyant force is equal to the weight of displaced fluid:
        F_b = ρ_fluid * V_submerged * g

    Where:
        ρ_fluid = fluid density (kg/m³)
        V_submerged = submerged volume (m³)
        g = gravitational acceleration (m/s²)

    Args:
        request: Buoyancy calculation parameters

    Returns:
        Buoyant force magnitude and displaced mass
    """
    # Calculate submerged volume
    submerged_volume = request.volume * request.submerged_fraction

    # Buoyant force: F = ρ * V * g
    buoyant_force = request.fluid_density * submerged_volume * request.gravity

    # Mass of displaced fluid
    displaced_mass = request.fluid_density * submerged_volume

    return BuoyancyResponse(
        buoyant_force=buoyant_force,
        displaced_mass=displaced_mass,
        will_float=None,
        equilibrium_depth_fraction=None,
    )


def calculate_terminal_velocity(request: TerminalVelocityRequest) -> TerminalVelocityResponse:
    """Calculate terminal velocity when drag force equals weight.

    At terminal velocity, the drag force equals the gravitational force:
        F_drag = F_gravity
        0.5 * ρ * v_t² * C_d * A = m * g

    Solving for v_t:
        v_t = √(2 * m * g / (ρ * C_d * A))

    Args:
        request: Terminal velocity calculation parameters

    Returns:
        Terminal velocity, time to reach 95%, and drag force at terminal
    """
    # Terminal velocity: v_t = √(2mg / ρCA)
    terminal_velocity = math.sqrt(
        (2.0 * request.mass * request.gravity)
        / (request.fluid_density * request.drag_coefficient * request.cross_sectional_area)
    )

    # Drag force at terminal velocity equals weight
    drag_force_at_terminal = request.mass * request.gravity

    # Time constant for exponential approach: τ = m / (0.5 * ρ * C_d * A * v_t)
    # Time to 95%: t ≈ 3τ (since e^(-3) ≈ 0.05)
    tau = request.mass / (
        0.5
        * request.fluid_density
        * request.drag_coefficient
        * request.cross_sectional_area
        * terminal_velocity
    )
    time_to_95_percent = 3.0 * tau

    return TerminalVelocityResponse(
        terminal_velocity=terminal_velocity,
        time_to_95_percent=time_to_95_percent,
        drag_force_at_terminal=drag_force_at_terminal,
    )


def simulate_underwater_motion(request: UnderwaterMotionRequest) -> UnderwaterMotionResponse:
    """Simulate underwater projectile motion with drag and buoyancy.

    This uses numerical integration (Euler method) to simulate motion under:
    - Gravity (downward force)
    - Buoyancy (upward force from displaced fluid)
    - Drag (opposing motion)

    The equations of motion are:
        F_total = F_gravity + F_buoyancy + F_drag
        a = F_total / m
        v(t+dt) = v(t) + a * dt
        x(t+dt) = x(t) + v(t) * dt

    Args:
        request: Underwater motion parameters

    Returns:
        Complete trajectory with positions and final state
    """
    # Initialize state
    pos = list(request.initial_position)
    vel = list(request.initial_velocity)

    # Calculate constant forces
    weight = request.mass * request.gravity  # Downward
    buoyant_force = request.fluid.density * request.volume * request.gravity  # Upward
    net_vertical_force = buoyant_force - weight  # Net constant vertical force

    trajectory = []
    t = 0.0
    max_depth = pos[1]
    total_distance = 0.0
    prev_pos = list(pos)

    steps = int(request.duration / request.dt)

    for _ in range(steps):
        # Record position
        trajectory.append([t, pos[0], pos[1], pos[2]])

        # Calculate speed
        speed = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)

        # Calculate drag force (opposes velocity)
        if speed > 1e-10:
            drag_magnitude = (
                0.5
                * request.fluid.density
                * speed
                * speed
                * request.drag_coefficient
                * request.cross_sectional_area
            )
            drag_force = [
                -drag_magnitude * vel[0] / speed,
                -drag_magnitude * vel[1] / speed,
                -drag_magnitude * vel[2] / speed,
            ]
        else:
            drag_force = [0.0, 0.0, 0.0]

        # Total acceleration
        ax = drag_force[0] / request.mass
        ay = (net_vertical_force + drag_force[1]) / request.mass
        az = drag_force[2] / request.mass

        # Update velocity
        vel[0] += ax * request.dt
        vel[1] += ay * request.dt
        vel[2] += az * request.dt

        # Update position
        prev_pos = list(pos)
        pos[0] += vel[0] * request.dt
        pos[1] += vel[1] * request.dt
        pos[2] += vel[2] * request.dt

        # Track max depth (minimum y)
        max_depth = min(max_depth, pos[1])

        # Track total distance
        dx = pos[0] - prev_pos[0]
        dy = pos[1] - prev_pos[1]
        dz = pos[2] - prev_pos[2]
        total_distance += math.sqrt(dx * dx + dy * dy + dz * dz)

        t += request.dt

    # Check if object has settled (very low velocity)
    final_speed = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)
    settled = final_speed < 0.01  # Less than 1 cm/s

    return UnderwaterMotionResponse(
        trajectory=trajectory,
        final_position=pos,
        final_velocity=vel,
        max_depth=max_depth,
        total_distance=total_distance,
        settled=settled,
    )
