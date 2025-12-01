"""Fluid dynamics MCP tool endpoints (basic + advanced)."""

import json
from typing import Optional, Union

from chuk_mcp_server import tool

from .. import fluid as fluid_module
from ..models import (
    DragForceRequest,
    BuoyancyRequest,
    TerminalVelocityRequest,
    FluidEnvironment,
    UnderwaterMotionRequest,
)


# ============================================================================
# Basic Fluid Dynamics Tools
# ============================================================================


@tool  # type: ignore[arg-type]
async def calculate_drag_force(
    velocity: Union[list[float], str],
    cross_sectional_area: float,
    fluid_density: float,
    drag_coefficient: float = 0.47,
) -> dict:
    """Calculate drag force for an object moving through a fluid.

    The drag force opposes motion and is given by:
        F_drag = 0.5 * ρ * v² * C_d * A

    Common drag coefficients:
        - Sphere: 0.47
        - Streamlined shape: 0.04
        - Flat plate (perpendicular): 1.28
        - Human (standing): 1.0-1.3
        - Car: 0.25-0.35

    Args:
        velocity: Velocity vector [x, y, z] in m/s (or JSON string)
        cross_sectional_area: Area perpendicular to flow in m²
        fluid_density: Fluid density in kg/m³ (water=1000, air=1.225)
        drag_coefficient: Drag coefficient (default 0.47 for sphere)

    Returns:
        Drag force vector, magnitude, and Reynolds number

    Example - Ball falling through water:
        result = await calculate_drag_force(
            velocity=[0, -5.0, 0],
            cross_sectional_area=0.00785,  # π * (0.05m)² for 10cm diameter
            fluid_density=1000,  # water
            drag_coefficient=0.47
        )
        # Returns upward drag force opposing downward motion
    """
    # Parse velocity if string
    parsed_velocity = json.loads(velocity) if isinstance(velocity, str) else velocity

    request = DragForceRequest(
        velocity=parsed_velocity,
        cross_sectional_area=cross_sectional_area,
        fluid_density=fluid_density,
        drag_coefficient=drag_coefficient,
    )

    response = fluid_module.calculate_drag_force(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_buoyancy(
    volume: float,
    fluid_density: float,
    gravity: float = 9.81,
    submerged_fraction: float = 1.0,
) -> dict:
    """Calculate buoyancy force using Archimedes' principle.

    The buoyant force equals the weight of displaced fluid:
        F_b = ρ_fluid * V_submerged * g

    Args:
        volume: Object volume in m³
        fluid_density: Fluid density in kg/m³ (water=1000, air=1.225)
        gravity: Gravitational acceleration in m/s² (default 9.81)
        submerged_fraction: Fraction submerged 0.0-1.0 (default 1.0 = fully submerged)

    Returns:
        Buoyant force (upward) and displaced mass

    Example - Checking if a 1kg ball will float:
        # 10cm diameter sphere: V = (4/3)πr³ = 0.000524 m³
        result = await calculate_buoyancy(
            volume=0.000524,
            fluid_density=1000  # water
        )
        # buoyant_force = 5.14 N
        # If weight (mg) < buoyant force, it floats
        # 1kg * 9.81 = 9.81 N > 5.14 N, so it sinks
    """
    request = BuoyancyRequest(
        volume=volume,
        fluid_density=fluid_density,
        gravity=gravity,
        submerged_fraction=submerged_fraction,
    )

    response = fluid_module.calculate_buoyancy(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_terminal_velocity(
    mass: float,
    cross_sectional_area: float,
    fluid_density: float,
    drag_coefficient: float = 0.47,
    gravity: float = 9.81,
) -> dict:
    """Calculate terminal velocity when drag equals weight.

    At terminal velocity, forces balance:
        F_drag = F_weight
        v_terminal = √(2mg / ρC_dA)

    Args:
        mass: Object mass in kg
        cross_sectional_area: Area perpendicular to fall direction in m²
        fluid_density: Fluid density in kg/m³ (air=1.225, water=1000)
        drag_coefficient: Drag coefficient (sphere=0.47, skydiver=1.0)
        gravity: Gravitational acceleration in m/s² (default 9.81)

    Returns:
        Terminal velocity, time to 95%, and drag force at terminal

    Example - Skydiver terminal velocity:
        result = await calculate_terminal_velocity(
            mass=70,  # kg
            cross_sectional_area=0.7,  # m² (belly-down position)
            fluid_density=1.225,  # air
            drag_coefficient=1.0,  # human
        )
        # v_terminal ≈ 54 m/s (120 mph)
    """
    request = TerminalVelocityRequest(
        mass=mass,
        cross_sectional_area=cross_sectional_area,
        fluid_density=fluid_density,
        drag_coefficient=drag_coefficient,
        gravity=gravity,
    )

    response = fluid_module.calculate_terminal_velocity(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def simulate_underwater_motion(
    initial_velocity: Union[list[float], str],
    mass: float,
    volume: float,
    cross_sectional_area: float,
    fluid_density: float = 1000.0,
    fluid_viscosity: float = 1.002e-3,
    initial_position: Union[list[float], str, None] = None,
    drag_coefficient: float = 0.47,
    gravity: float = 9.81,
    duration: float = 10.0,
    dt: float = 0.01,
) -> dict:
    """Simulate underwater projectile motion with drag and buoyancy.

    Uses numerical integration to simulate motion under:
    - Gravity (downward)
    - Buoyancy (upward, from displaced fluid)
    - Drag (opposes motion)

    Args:
        initial_velocity: Initial velocity [x, y, z] in m/s
        mass: Object mass in kg
        volume: Object volume in m³
        cross_sectional_area: Cross-sectional area in m²
        fluid_density: Fluid density in kg/m³ (default 1000 for water)
        fluid_viscosity: Fluid viscosity in Pa·s (default 1.002e-3 for water)
        initial_position: Initial position [x, y, z] in m (default [0,0,0])
        drag_coefficient: Drag coefficient (default 0.47 for sphere)
        gravity: Gravitational acceleration in m/s² (default 9.81)
        duration: Simulation duration in seconds (default 10.0)
        dt: Time step in seconds (default 0.01)

    Returns:
        Complete trajectory, final state, max depth, and total distance

    Example - Torpedo launch:
        result = await simulate_underwater_motion(
            initial_velocity=[20, 0, 0],  # 20 m/s forward
            mass=100,  # kg
            volume=0.05,  # m³
            cross_sectional_area=0.03,  # m²
            fluid_density=1000,  # water
            drag_coefficient=0.04,  # streamlined
            duration=30.0
        )
    """
    # Parse inputs
    parsed_velocity = (
        json.loads(initial_velocity) if isinstance(initial_velocity, str) else initial_velocity
    )
    parsed_position = [0.0, 0.0, 0.0]
    if initial_position is not None:
        parsed_position = (
            json.loads(initial_position) if isinstance(initial_position, str) else initial_position
        )

    fluid_env = FluidEnvironment(
        density=fluid_density,
        viscosity=fluid_viscosity,
        name=None,
    )

    request = UnderwaterMotionRequest(
        initial_position=parsed_position,
        initial_velocity=parsed_velocity,
        mass=mass,
        volume=volume,
        drag_coefficient=drag_coefficient,
        cross_sectional_area=cross_sectional_area,
        fluid=fluid_env,
        gravity=gravity,
        duration=duration,
        dt=dt,
    )

    response = fluid_module.simulate_underwater_motion(request)
    return response.model_dump()


# ============================================================================
# Advanced Fluid Dynamics Tools (Phase 3)
# ============================================================================


@tool  # type: ignore[arg-type]
async def calculate_lift_force(
    velocity: float,
    wing_area: float,
    lift_coefficient: float,
    fluid_density: float = 1.225,
) -> dict:
    """Calculate lift force using: L = (1/2) ρ v² C_L A.

    Based on Bernoulli's principle and wing aerodynamics.

    Args:
        velocity: Flow velocity in m/s
        wing_area: Wing area in m²
        lift_coefficient: Lift coefficient C_L (dimensionless)
        fluid_density: Fluid density in kg/m³ (air=1.225)

    Returns:
        Dict containing:
            - lift_force: Lift force in Newtons
            - dynamic_pressure: Dynamic pressure (q) in Pascals

    Example - Aircraft wing:
        result = await calculate_lift_force(
            velocity=70,  # m/s (~250 km/h)
            wing_area=20.0,  # m²
            lift_coefficient=1.2,
            fluid_density=1.225
        )
    """
    from ..fluid_advanced import LiftForceRequest, calculate_lift_force as calc_lift

    request = LiftForceRequest(
        velocity=velocity,
        wing_area=wing_area,
        lift_coefficient=lift_coefficient,
        fluid_density=fluid_density,
    )
    response = calc_lift(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_magnus_force(
    velocity: Union[list[float], str],
    angular_velocity: Union[list[float], str],
    radius: float,
    fluid_density: float = 1.225,
) -> dict:
    """Calculate Magnus force on a spinning ball.

    The Magnus force is perpendicular to both velocity and spin axis.
    Causes curve balls in sports.

    Args:
        velocity: Ball velocity [x, y, z] in m/s (or JSON string)
        angular_velocity: Angular velocity [x, y, z] in rad/s (or JSON string)
        radius: Ball radius in meters
        fluid_density: Fluid density in kg/m³ (air=1.225)

    Returns:
        Dict containing:
            - magnus_force: Magnus force vector [x, y, z] in Newtons
            - magnus_force_magnitude: Force magnitude in Newtons
            - spin_rate: Spin rate (angular velocity magnitude) in rad/s

    Example - Soccer ball curve:
        result = await calculate_magnus_force(
            velocity=[20, 0, 0],  # 20 m/s forward
            angular_velocity=[0, 0, 50],  # 50 rad/s topspin
            radius=0.11,  # Soccer ball
            fluid_density=1.225
        )
    """
    from ..fluid_advanced import MagnusForceRequest, calculate_magnus_force as calc_magnus

    parsed_velocity = json.loads(velocity) if isinstance(velocity, str) else velocity
    parsed_angular_velocity = (
        json.loads(angular_velocity) if isinstance(angular_velocity, str) else angular_velocity
    )

    request = MagnusForceRequest(
        velocity=parsed_velocity,
        angular_velocity=parsed_angular_velocity,
        radius=radius,
        fluid_density=fluid_density,
    )
    response = calc_magnus(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_bernoulli(
    pressure1: float,
    velocity1: float,
    height1: float,
    velocity2: Optional[float] = None,
    height2: Optional[float] = None,
    fluid_density: float = 1000.0,
    gravity: float = 9.81,
) -> dict:
    """Calculate Bernoulli's equation: P + (1/2)ρv² + ρgh = constant.

    Energy conservation for flowing fluids.

    Args:
        pressure1: Pressure at point 1 in Pascals
        velocity1: Flow velocity at point 1 in m/s
        height1: Height at point 1 in meters
        velocity2: Flow velocity at point 2 in m/s (optional)
        height2: Height at point 2 in meters (optional)
        fluid_density: Fluid density in kg/m³ (default 1000 for water)
        gravity: Gravitational acceleration in m/s² (default 9.81)

    Returns:
        Dict containing:
            - total_pressure_1: Total pressure at point 1
            - static_pressure_1: Static pressure component
            - dynamic_pressure_1: Dynamic pressure component
            - hydrostatic_pressure_1: Hydrostatic pressure component
            - pressure2: Pressure at point 2 (if velocity2/height2 given)

    Example - Water tank with outlet:
        result = await calculate_bernoulli(
            pressure1=101325,  # Atmospheric at top
            velocity1=0,  # Still water
            height1=10,  # 10m height
            velocity2=14,  # Exit velocity
            height2=0,  # Ground level
            fluid_density=1000
        )
    """
    from ..fluid_advanced import BernoulliRequest, calculate_bernoulli as calc_bernoulli

    request = BernoulliRequest(
        pressure1=pressure1,
        velocity1=velocity1,
        height1=height1,
        velocity2=velocity2,
        height2=height2,
        fluid_density=fluid_density,
        gravity=gravity,
    )
    response = calc_bernoulli(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_pressure_at_depth(
    depth: float,
    fluid_density: float,
    atmospheric_pressure: float = 101325.0,
    gravity: float = 9.81,
) -> dict:
    """Calculate pressure at depth: P = P_atm + ρgh.

    Hydrostatic pressure increases with depth.

    Args:
        depth: Depth below surface in meters
        fluid_density: Fluid density in kg/m³ (water=1000, seawater=1025)
        atmospheric_pressure: Pressure at surface in Pascals (default 101325)
        gravity: Gravitational acceleration in m/s² (default 9.81)

    Returns:
        Dict containing:
            - total_pressure: Total pressure in Pascals
            - gauge_pressure: Pressure above atmospheric in Pascals
            - pressure_atmospheres: Pressure in atmospheres (1 atm = 101325 Pa)

    Example - Scuba diving at 30m:
        result = await calculate_pressure_at_depth(
            depth=30,  # meters
            fluid_density=1025,  # seawater
            atmospheric_pressure=101325
        )
        # Result: ~4 atmospheres
    """
    from ..fluid_advanced import (
        PressureAtDepthRequest,
        calculate_pressure_at_depth as calc_pressure,
    )

    request = PressureAtDepthRequest(
        depth=depth,
        fluid_density=fluid_density,
        atmospheric_pressure=atmospheric_pressure,
        gravity=gravity,
    )
    response = calc_pressure(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_reynolds_number(
    velocity: float,
    characteristic_length: float,
    fluid_density: float,
    dynamic_viscosity: float,
) -> dict:
    """Calculate Reynolds number: Re = ρvL/μ.

    Determines flow regime (laminar, transitional, turbulent).

    Args:
        velocity: Flow velocity in m/s
        characteristic_length: Characteristic length in meters (pipe diameter, etc.)
        fluid_density: Fluid density in kg/m³
        dynamic_viscosity: Dynamic viscosity in Pa·s (water=0.001, air=1.8e-5)

    Returns:
        Dict containing:
            - reynolds_number: Re (dimensionless)
            - flow_regime: "laminar" (Re<2300), "transitional" (2300-4000), "turbulent" (Re>4000)

    Example - Water in pipe:
        result = await calculate_reynolds_number(
            velocity=2.0,  # m/s
            characteristic_length=0.05,  # 5cm diameter
            fluid_density=1000,  # water
            dynamic_viscosity=0.001
        )
        # Re = 100,000 → turbulent
    """
    from ..fluid_advanced import ReynoldsNumberRequest, calculate_reynolds_number as calc_reynolds

    request = ReynoldsNumberRequest(
        velocity=velocity,
        characteristic_length=characteristic_length,
        fluid_density=fluid_density,
        dynamic_viscosity=dynamic_viscosity,
    )
    response = calc_reynolds(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_venturi_effect(
    inlet_diameter: float,
    throat_diameter: float,
    inlet_velocity: float,
    fluid_density: float,
) -> dict:
    """Calculate Venturi effect (flow through constriction).

    Uses continuity equation and Bernoulli's principle.

    Args:
        inlet_diameter: Inlet diameter in meters
        throat_diameter: Throat (constriction) diameter in meters
        inlet_velocity: Inlet velocity in m/s
        fluid_density: Fluid density in kg/m³

    Returns:
        Dict containing:
            - throat_velocity: Velocity at throat in m/s
            - pressure_drop: Pressure drop from inlet to throat in Pascals
            - flow_rate: Volumetric flow rate in m³/s

    Example - Venturi meter:
        result = await calculate_venturi_effect(
            inlet_diameter=0.1,  # 10 cm
            throat_diameter=0.05,  # 5 cm
            inlet_velocity=2.0,  # m/s
            fluid_density=1000  # water
        )
        # throat_velocity = 8 m/s (4x area reduction)
    """
    from ..fluid_advanced import VenturiEffectRequest, calculate_venturi_effect as calc_venturi

    request = VenturiEffectRequest(
        inlet_diameter=inlet_diameter,
        throat_diameter=throat_diameter,
        inlet_velocity=inlet_velocity,
        fluid_density=fluid_density,
    )
    response = calc_venturi(request)
    return response.model_dump()
