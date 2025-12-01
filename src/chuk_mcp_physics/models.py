"""Pydantic models for Physics MCP Server.

All responses are properly typed using Pydantic models for type safety,
validation, and better IDE support. No dictionary goop - everything is strongly typed.
"""

from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


# ============================================================================
# Enums - No Magic Strings
# ============================================================================


class BodyType(str, Enum):
    """Rigid body type enumeration."""

    STATIC = "static"
    DYNAMIC = "dynamic"
    KINEMATIC = "kinematic"


class ShapeType(str, Enum):
    """Collider shape types."""

    BOX = "box"
    SPHERE = "sphere"
    CAPSULE = "capsule"
    CYLINDER = "cylinder"
    PLANE = "plane"
    MESH = "mesh"


class IntegratorType(str, Enum):
    """Physics integrator types."""

    EULER = "euler"
    VERLET = "verlet"
    RK4 = "rk4"


class JointType(str, Enum):
    """Joint type enumeration."""

    FIXED = "fixed"
    REVOLUTE = "revolute"  # Hinge
    SPHERICAL = "spherical"  # Ball-and-socket
    PRISMATIC = "prismatic"  # Slider


# ============================================================================
# Vector Types
# ============================================================================


class Vector3(BaseModel):
    """3D vector representation."""

    x: float = Field(..., description="X component")
    y: float = Field(..., description="Y component")
    z: float = Field(..., description="Z component")

    def to_list(self) -> list[float]:
        """Convert to list [x, y, z]."""
        return [self.x, self.y, self.z]

    @classmethod
    def from_list(cls, vec: list[float]) -> "Vector3":
        """Create from list [x, y, z]."""
        return cls(x=vec[0], y=vec[1], z=vec[2])

    def __repr__(self) -> str:
        return f"Vector3({self.x}, {self.y}, {self.z})"


class Quaternion(BaseModel):
    """Quaternion for 3D rotations."""

    x: float = Field(..., description="X component")
    y: float = Field(..., description="Y component")
    z: float = Field(..., description="Z component")
    w: float = Field(..., description="W (scalar) component")

    def to_list(self) -> list[float]:
        """Convert to list [x, y, z, w]."""
        return [self.x, self.y, self.z, self.w]

    @classmethod
    def from_list(cls, quat: list[float]) -> "Quaternion":
        """Create from list [x, y, z, w]."""
        return cls(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    @classmethod
    def identity(cls) -> "Quaternion":
        """Create identity quaternion (no rotation)."""
        return cls(x=0.0, y=0.0, z=0.0, w=1.0)

    def __repr__(self) -> str:
        return f"Quaternion({self.x}, {self.y}, {self.z}, {self.w})"


# ============================================================================
# Collision & Contact Models
# ============================================================================


class ContactEvent(BaseModel):
    """Single contact event between two bodies."""

    time: float = Field(..., description="Simulation time when contact occurred")
    body_a: str = Field(..., description="First body ID")
    body_b: str = Field(..., description="Second body ID")
    contact_point: list[float] = Field(..., description="World space position [x, y, z]")
    normal: list[float] = Field(..., description="Contact normal (from A to B)")
    impulse_magnitude: float = Field(..., description="Collision impulse magnitude")
    relative_velocity: list[float] = Field(
        ..., description="Relative velocity at contact [x, y, z]"
    )
    event_type: str = Field(..., description="Event type: 'started', 'ongoing', or 'ended'")


class BounceEvent(BaseModel):
    """Detected bounce event (local minimum in height)."""

    time: float = Field(..., description="Time of bounce")
    bounce_number: int = Field(..., description="Bounce sequence number (1-indexed)")
    position: list[float] = Field(..., description="Position at bounce [x, y, z]")
    velocity_before: list[float] = Field(..., description="Velocity just before bounce [x, y, z]")
    velocity_after: list[float] = Field(..., description="Velocity just after bounce [x, y, z]")
    height_at_bounce: float = Field(..., description="Height (y-coordinate) at bounce")
    speed_before: float = Field(..., description="Speed magnitude before bounce")
    speed_after: float = Field(..., description="Speed magnitude after bounce")
    energy_loss_percent: float = Field(..., description="Percentage of kinetic energy lost")


# ============================================================================
# Rigid Body Models
# ============================================================================


class RigidBodyDefinition(BaseModel):
    """Definition for creating a rigid body in a simulation."""

    id: str = Field(..., description="Unique identifier for this body")
    kind: BodyType = Field(default=BodyType.DYNAMIC, description="Body type")
    shape: ShapeType = Field(..., description="Collider shape type")

    # Shape parameters (which ones are used depends on shape type)
    size: Optional[list[float]] = Field(
        None, description="Size parameters: [width, height, depth] for box, [radius] for sphere"
    )
    normal: Optional[list[float]] = Field(
        None, description="Normal vector [x, y, z] for plane (default [0, 1, 0])"
    )
    offset: Optional[float] = Field(None, description="Offset along normal for plane (default 0.0)")

    # Physics properties
    mass: float = Field(default=1.0, description="Mass in kilograms", gt=0.0)
    position: list[float] = Field(default=[0.0, 0.0, 0.0], description="Initial position [x, y, z]")
    orientation: list[float] = Field(
        default=[0.0, 0.0, 0.0, 1.0], description="Initial orientation quaternion [x, y, z, w]"
    )
    velocity: list[float] = Field(default=[0.0, 0.0, 0.0], description="Initial velocity [x, y, z]")
    angular_velocity: list[float] = Field(
        default=[0.0, 0.0, 0.0], description="Initial angular velocity [x, y, z]"
    )

    # Material properties
    restitution: float = Field(
        default=0.5, description="Coefficient of restitution (bounciness)", ge=0.0, le=1.0
    )
    friction: float = Field(default=0.5, description="Coefficient of friction", ge=0.0)

    # Sensor flag
    is_sensor: bool = Field(
        default=False, description="If true, body detects collisions but doesn't respond to them"
    )

    # Damping (Phase 1.4)
    linear_damping: float = Field(
        default=0.0,
        description="Linear velocity damping (0.0-1.0) - like air resistance",
        ge=0.0,
        le=1.0,
    )
    angular_damping: float = Field(
        default=0.0,
        description="Angular velocity damping (0.0-1.0) - like rotational friction",
        ge=0.0,
        le=1.0,
    )

    # Orientation-Dependent Drag (Advanced)
    drag_coefficient: Optional[float] = Field(
        None,
        description="Base drag coefficient (Cd) for aerodynamic drag force",
        ge=0.0,
    )
    drag_area: Optional[float] = Field(
        None,
        description="Reference cross-sectional area for drag calculation (m²)",
        gt=0.0,
    )
    drag_axis_ratios: Optional[list[float]] = Field(
        None,
        description="Drag coefficient ratios [x, y, z] relative to base Cd for orientation-dependent drag. "
        "E.g., [1.0, 0.3, 1.0] for streamlined in Y direction (like football)",
    )
    fluid_density: float = Field(
        default=1.225,
        description="Fluid density for drag calculation (kg/m³). Air=1.225, Water=1000",
        gt=0.0,
    )


class JointDefinition(BaseModel):
    """Definition for creating a joint between two bodies."""

    id: str = Field(..., description="Unique identifier for this joint")
    joint_type: JointType = Field(..., description="Type of joint")

    # Bodies to connect
    body_a: str = Field(..., description="First body ID")
    body_b: str = Field(..., description="Second body ID")

    # Attachment points (local coordinates)
    anchor_a: list[float] = Field(
        default=[0.0, 0.0, 0.0],
        description="Attachment point on body A (local) [x, y, z]",
    )
    anchor_b: list[float] = Field(
        default=[0.0, 0.0, 0.0],
        description="Attachment point on body B (local) [x, y, z]",
    )

    # Axis (for revolute and prismatic)
    axis: Optional[list[float]] = Field(
        None, description="Joint axis (local to each body) [x, y, z]"
    )

    # Limits (for revolute and prismatic)
    limits: Optional[list[float]] = Field(
        None,
        description="Joint limits: [min, max] (radians for revolute, meters for prismatic)",
    )


class ContactPoint(BaseModel):
    """Contact point information for a collision."""

    with_body: str = Field(..., description="ID of the other body in contact")
    point: list[float] = Field(..., description="Contact point in world space [x, y, z]")
    normal: list[float] = Field(..., description="Contact normal vector [x, y, z]")
    impulse: float = Field(..., description="Impulse magnitude at this contact")
    distance: float = Field(..., description="Penetration distance (negative if separated)")


class RigidBodyState(BaseModel):
    """Current state of a rigid body."""

    id: str = Field(..., description="Body identifier")
    position: list[float] = Field(..., description="Position [x, y, z]")
    orientation: list[float] = Field(..., description="Orientation quaternion [x, y, z, w]")
    velocity: list[float] = Field(..., description="Linear velocity [x, y, z]")
    angular_velocity: list[float] = Field(..., description="Angular velocity [x, y, z]")
    contacts: list[str] = Field(default=[], description="IDs of bodies in contact")


# ============================================================================
# Simulation Models
# ============================================================================


class SimulationConfig(BaseModel):
    """Configuration for a physics simulation."""

    gravity: list[float] = Field(default=[0.0, -9.81, 0.0], description="Gravity vector [x, y, z]")
    dimensions: int = Field(default=3, description="2D or 3D simulation", ge=2, le=3)
    dt: float = Field(default=0.016, description="Time step in seconds", gt=0.0)
    integrator: IntegratorType = Field(default=IntegratorType.VERLET, description="Integrator type")


class SimulationCreateResponse(BaseModel):
    """Response when creating a new simulation."""

    sim_id: str = Field(..., description="Unique simulation identifier")
    config: SimulationConfig = Field(..., description="Simulation configuration")


class SimulationStepResponse(BaseModel):
    """Response from stepping a simulation forward."""

    sim_id: str = Field(..., description="Simulation identifier")
    time: float = Field(..., description="Current simulation time in seconds")
    bodies: list[RigidBodyState] = Field(..., description="State of all bodies in the simulation")


class TrajectoryFrame(BaseModel):
    """Single frame in a trajectory recording."""

    time: float = Field(..., description="Absolute time in seconds", alias="t")
    position: list[float] = Field(..., description="Position [x, y, z] in meters")
    orientation: list[float] = Field(
        ..., description="Orientation quaternion [x, y, z, w]", alias="rotation"
    )
    velocity: Optional[list[float]] = Field(
        None, description="Linear velocity [x, y, z] in m/s (optional)"
    )
    angular_velocity: Optional[list[float]] = Field(
        None, description="Angular velocity [x, y, z] in rad/s (optional)"
    )

    class Config:
        populate_by_name = True  # Allow both 't' and 'time', 'rotation' and 'orientation'


class TrajectoryMeta(BaseModel):
    """Metadata for trajectory recordings."""

    body_id: str = Field(
        ..., description='Fully qualified body identifier (e.g., "rapier://sim-123/ball")'
    )
    total_time: float = Field(..., description="Total simulation time in seconds")
    num_frames: int = Field(..., description="Number of frames recorded")


class TrajectoryResponse(BaseModel):
    """Recorded trajectory for a body.

    This schema is the canonical contract between chuk-mcp-physics
    and visualization systems (R3F, Remotion, Three.js, etc.).
    """

    dt: float = Field(..., description="Time step between frames in seconds")
    frames: list[TrajectoryFrame] = Field(..., description="Trajectory frames")
    meta: TrajectoryMeta = Field(..., description="Trajectory metadata")
    contact_events: list[ContactEvent] = Field(
        default_factory=list, description="Contact events from physics engine (Phase 1.2)"
    )


class TrajectoryWithEventsResponse(BaseModel):
    """Recorded trajectory with detected collision and bounce events."""

    dt: float = Field(..., description="Time step between frames in seconds")
    frames: list[TrajectoryFrame] = Field(..., description="Trajectory frames")
    meta: TrajectoryMeta = Field(..., description="Trajectory metadata")
    contact_events: list[ContactEvent] = Field(
        default_factory=list, description="Detected contact/collision events"
    )
    bounces: list[BounceEvent] = Field(default_factory=list, description="Detected bounce events")


# ============================================================================
# Analytic Calculation Models
# ============================================================================


class ProjectileMotionRequest(BaseModel):
    """Request for projectile motion calculation."""

    initial_velocity: float = Field(..., description="Initial velocity in m/s", gt=0.0)
    angle_degrees: float = Field(..., description="Launch angle in degrees", ge=0.0, le=90.0)
    initial_height: float = Field(default=0.0, description="Initial height in meters", ge=0.0)
    gravity: float = Field(default=9.81, description="Gravitational acceleration (m/s²)", gt=0.0)


class ProjectileMotionResponse(BaseModel):
    """Response for projectile motion calculation."""

    max_height: float = Field(..., description="Maximum height reached in meters")
    range: float = Field(..., description="Horizontal range in meters")
    time_of_flight: float = Field(..., description="Total time in the air in seconds")
    trajectory_points: list[list[float]] = Field(
        ..., description="Sample trajectory points [[x, y], [x, y], ...]"
    )


class ProjectileWithDragRequest(BaseModel):
    """Request for projectile motion calculation with air resistance.

    Supports optional enhancements:
    - Spin effects (Magnus force) for curveballs, slices, hooks
    - Wind effects (constant wind vector)
    - Variable air density (altitude effects)
    """

    initial_velocity: float = Field(..., description="Initial velocity in m/s", gt=0.0)
    angle_degrees: float = Field(..., description="Launch angle in degrees", ge=0.0, le=90.0)
    initial_height: float = Field(default=0.0, description="Initial height in meters", ge=0.0)
    mass: float = Field(..., description="Object mass in kg", gt=0.0)
    drag_coefficient: float = Field(
        default=0.47, description="Drag coefficient (sphere=0.47, baseball=0.4)", ge=0.0
    )
    cross_sectional_area: float = Field(..., description="Cross-sectional area in m²", gt=0.0)
    fluid_density: float = Field(
        default=1.225, description="Fluid density in kg/m³ (air=1.225, water=1000)", gt=0.0
    )
    gravity: float = Field(default=9.81, description="Gravitational acceleration (m/s²)", gt=0.0)
    time_step: float = Field(
        default=0.01, description="Integration time step in seconds", gt=0.0, le=0.1
    )
    max_time: float = Field(default=30.0, description="Maximum simulation time in seconds", gt=0.0)

    # Optional enhancements
    spin_rate: float = Field(
        default=0.0, description="Spin rate in rad/s (for Magnus force)", ge=0.0
    )
    spin_axis: list[float] = Field(
        default=[0.0, 0.0, 1.0],
        description="Spin axis unit vector [x, y, z] (perpendicular to motion for max effect)",
    )
    wind_velocity: list[float] = Field(
        default=[0.0, 0.0], description="Wind velocity [vx, vy] in m/s (horizontal, vertical)"
    )
    altitude: float = Field(
        default=0.0, description="Altitude above sea level in meters (affects air density)", ge=0.0
    )
    temperature: float = Field(
        default=15.0,
        description="Air temperature in Celsius (affects air density)",
        ge=-50.0,
        le=60.0,
    )


class ProjectileWithDragResponse(BaseModel):
    """Response for projectile motion with drag calculation."""

    max_height: float = Field(..., description="Maximum height reached in meters")
    range: float = Field(..., description="Horizontal range in meters")
    time_of_flight: float = Field(..., description="Total time in the air in seconds")
    impact_velocity: float = Field(..., description="Speed at landing in m/s")
    impact_angle: float = Field(..., description="Angle at landing in degrees (below horizontal)")
    trajectory_points: list[list[float]] = Field(
        ..., description="Sample trajectory points [[x, y], [x, y], ...]"
    )
    energy_lost_to_drag: float = Field(..., description="Energy dissipated by drag in joules")
    initial_kinetic_energy: float = Field(..., description="Initial kinetic energy in joules")
    final_kinetic_energy: float = Field(..., description="Final kinetic energy in joules")

    # Enhancement tracking
    lateral_deflection: float = Field(
        default=0.0,
        description="Lateral deflection from spin/wind in meters (perpendicular to launch)",
    )
    magnus_force_max: float = Field(
        default=0.0, description="Maximum Magnus force magnitude in Newtons"
    )
    wind_drift: float = Field(default=0.0, description="Total wind drift in meters (horizontal)")
    effective_air_density: float = Field(
        default=1.225,
        description="Effective air density used (kg/m³) accounting for altitude/temperature",
    )


class CollisionCheckRequest(BaseModel):
    """Request to check if two objects will collide."""

    body1_position: list[float] = Field(..., description="Body 1 position [x, y, z]")
    body1_velocity: list[float] = Field(..., description="Body 1 velocity [x, y, z]")
    body1_radius: float = Field(..., description="Body 1 radius in meters", gt=0.0)

    body2_position: list[float] = Field(..., description="Body 2 position [x, y, z]")
    body2_velocity: list[float] = Field(..., description="Body 2 velocity [x, y, z]")
    body2_radius: float = Field(..., description="Body 2 radius in meters", gt=0.0)

    max_time: float = Field(default=10.0, description="Maximum time to check in seconds", gt=0.0)


class CollisionCheckResponse(BaseModel):
    """Response for collision check."""

    will_collide: bool = Field(..., description="Whether the objects will collide")
    collision_time: Optional[float] = Field(
        None, description="Time until collision in seconds (if collision occurs)"
    )
    collision_point: Optional[list[float]] = Field(
        None, description="Collision point [x, y, z] (if collision occurs)"
    )
    impact_speed: Optional[float] = Field(
        None, description="Relative speed at impact in m/s (if collision occurs)"
    )
    closest_approach_distance: float = Field(
        ..., description="Minimum distance between objects in meters"
    )
    closest_approach_time: float = Field(..., description="Time of closest approach in seconds")


class ForceCalculationRequest(BaseModel):
    """Request for force calculation (F = ma)."""

    mass: float = Field(..., description="Mass in kilograms", gt=0.0)
    acceleration: list[float] = Field(..., description="Acceleration vector [x, y, z] in m/s²")


class ForceCalculationResponse(BaseModel):
    """Response for force calculation."""

    force: list[float] = Field(..., description="Force vector [x, y, z] in Newtons")
    magnitude: float = Field(..., description="Force magnitude in Newtons")


class KineticEnergyRequest(BaseModel):
    """Request for kinetic energy calculation."""

    mass: float = Field(..., description="Mass in kilograms", gt=0.0)
    velocity: list[float] = Field(..., description="Velocity vector [x, y, z] in m/s")


class KineticEnergyResponse(BaseModel):
    """Response for kinetic energy calculation."""

    kinetic_energy: float = Field(..., description="Kinetic energy in Joules")
    speed: float = Field(..., description="Speed (velocity magnitude) in m/s")


class MomentumRequest(BaseModel):
    """Request for momentum calculation."""

    mass: float = Field(..., description="Mass in kilograms", gt=0.0)
    velocity: list[float] = Field(..., description="Velocity vector [x, y, z] in m/s")


class MomentumResponse(BaseModel):
    """Response for momentum calculation."""

    momentum: list[float] = Field(..., description="Momentum vector [x, y, z] in kg⋅m/s")
    magnitude: float = Field(..., description="Momentum magnitude in kg⋅m/s")


class PotentialEnergyRequest(BaseModel):
    """Request for potential energy calculation."""

    mass: float = Field(..., description="Mass in kilograms", gt=0.0)
    height: float = Field(..., description="Height in meters", ge=0.0)
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)


class PotentialEnergyResponse(BaseModel):
    """Response for potential energy calculation."""

    potential_energy: float = Field(..., description="Gravitational potential energy in Joules")
    equivalent_kinetic_velocity: float = Field(
        ..., description="Velocity if converted to kinetic energy (v = √(2gh)) in m/s"
    )


class WorkPowerRequest(BaseModel):
    """Request for work and power calculation."""

    force: list[float] = Field(..., description="Force vector [x, y, z] in Newtons")
    displacement: list[float] = Field(..., description="Displacement vector [x, y, z] in meters")
    time: Optional[float] = Field(
        None, description="Time taken in seconds (for power calculation)", gt=0.0
    )


class WorkPowerResponse(BaseModel):
    """Response for work and power calculation."""

    work: float = Field(..., description="Work done (W = F·d) in Joules")
    power: Optional[float] = Field(None, description="Power (P = W/t) in Watts (if time provided)")


class ElasticCollisionRequest(BaseModel):
    """Request for 1D elastic collision calculation."""

    mass1: float = Field(..., description="Mass of object 1 in kg", gt=0.0)
    velocity1: float = Field(..., description="Initial velocity of object 1 in m/s")
    mass2: float = Field(..., description="Mass of object 2 in kg", gt=0.0)
    velocity2: float = Field(..., description="Initial velocity of object 2 in m/s")


class ElasticCollisionResponse(BaseModel):
    """Response for elastic collision calculation."""

    final_velocity1: float = Field(..., description="Final velocity of object 1 in m/s")
    final_velocity2: float = Field(..., description="Final velocity of object 2 in m/s")
    initial_kinetic_energy: float = Field(..., description="Total KE before collision in Joules")
    final_kinetic_energy: float = Field(..., description="Total KE after collision in Joules")
    initial_momentum: float = Field(..., description="Total momentum before collision in kg⋅m/s")
    final_momentum: float = Field(..., description="Total momentum after collision in kg⋅m/s")


# ============================================================================
# Fluid Dynamics Models
# ============================================================================


class FluidEnvironment(BaseModel):
    """Fluid environment properties for drag and buoyancy calculations."""

    density: float = Field(..., description="Fluid density in kg/m³", gt=0.0)
    viscosity: float = Field(..., description="Dynamic viscosity in Pa·s (Pascal-seconds)", gt=0.0)
    name: Optional[str] = Field(None, description="Environment name (e.g., 'water', 'air')")

    @classmethod
    def water(cls, temperature_celsius: float = 20.0) -> "FluidEnvironment":
        """Create water environment at given temperature."""
        # Water density at 20°C: 998.2 kg/m³, viscosity: 1.002e-3 Pa·s
        return cls(density=998.2, viscosity=1.002e-3, name=f"water_{temperature_celsius}C")

    @classmethod
    def air(
        cls, temperature_celsius: float = 20.0, pressure_pa: float = 101325.0
    ) -> "FluidEnvironment":
        """Create air environment at given temperature and pressure."""
        # Air density at 20°C, 1 atm: 1.204 kg/m³, viscosity: 1.825e-5 Pa·s
        return cls(density=1.204, viscosity=1.825e-5, name=f"air_{temperature_celsius}C")

    @classmethod
    def oil(cls) -> "FluidEnvironment":
        """Create motor oil environment."""
        # Motor oil: density ~900 kg/m³, viscosity ~0.1 Pa·s
        return cls(density=900.0, viscosity=0.1, name="motor_oil")


class DragForceRequest(BaseModel):
    """Request for drag force calculation."""

    velocity: list[float] = Field(..., description="Velocity vector [x, y, z] in m/s")
    drag_coefficient: float = Field(
        default=0.47,
        description="Drag coefficient (sphere=0.47, streamlined=0.04, flat plate=1.28)",
        gt=0.0,
    )
    cross_sectional_area: float = Field(
        ..., description="Cross-sectional area in m² (perpendicular to flow)", gt=0.0
    )
    fluid_density: float = Field(
        ..., description="Fluid density in kg/m³ (water=1000, air=1.225)", gt=0.0
    )


class DragForceResponse(BaseModel):
    """Response for drag force calculation."""

    drag_force: list[float] = Field(..., description="Drag force vector [x, y, z] in Newtons")
    magnitude: float = Field(..., description="Drag force magnitude in Newtons")
    reynolds_number: float = Field(
        ..., description="Reynolds number (indicates flow regime: <2300=laminar, >4000=turbulent)"
    )


class BuoyancyRequest(BaseModel):
    """Request for buoyancy force calculation."""

    volume: float = Field(..., description="Object volume in m³", gt=0.0)
    fluid_density: float = Field(
        ..., description="Fluid density in kg/m³ (water=1000, air=1.225)", gt=0.0
    )
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)
    submerged_fraction: float = Field(
        default=1.0,
        description="Fraction of volume submerged (0.0-1.0)",
        ge=0.0,
        le=1.0,
    )


class BuoyancyResponse(BaseModel):
    """Response for buoyancy force calculation."""

    buoyant_force: float = Field(..., description="Upward buoyant force in Newtons")
    displaced_mass: float = Field(..., description="Mass of displaced fluid in kg")
    will_float: Optional[bool] = Field(
        None, description="Whether object floats (requires object_mass parameter)"
    )
    equilibrium_depth_fraction: Optional[float] = Field(
        None,
        description="Fraction submerged at equilibrium (requires object_mass and object_volume)",
    )


class TerminalVelocityRequest(BaseModel):
    """Request for terminal velocity calculation."""

    mass: float = Field(..., description="Object mass in kg", gt=0.0)
    drag_coefficient: float = Field(
        default=0.47,
        description="Drag coefficient (sphere=0.47, skydiver=1.0, streamlined=0.04)",
        gt=0.0,
    )
    cross_sectional_area: float = Field(..., description="Cross-sectional area in m²", gt=0.0)
    fluid_density: float = Field(
        ..., description="Fluid density in kg/m³ (air=1.225, water=1000)", gt=0.0
    )
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)


class TerminalVelocityResponse(BaseModel):
    """Response for terminal velocity calculation."""

    terminal_velocity: float = Field(..., description="Terminal velocity in m/s")
    time_to_95_percent: float = Field(
        ..., description="Time to reach 95% of terminal velocity in seconds"
    )
    drag_force_at_terminal: float = Field(
        ..., description="Drag force at terminal velocity in Newtons (equals weight)"
    )


class UnderwaterMotionRequest(BaseModel):
    """Request for underwater projectile motion calculation."""

    initial_position: list[float] = Field(
        default=[0.0, 0.0, 0.0], description="Initial position [x, y, z] in meters"
    )
    initial_velocity: list[float] = Field(..., description="Initial velocity [x, y, z] in m/s")
    mass: float = Field(..., description="Object mass in kg", gt=0.0)
    volume: float = Field(..., description="Object volume in m³", gt=0.0)
    drag_coefficient: float = Field(default=0.47, description="Drag coefficient", gt=0.0)
    cross_sectional_area: float = Field(..., description="Cross-sectional area in m²", gt=0.0)
    fluid: FluidEnvironment = Field(..., description="Fluid environment properties")
    gravity: float = Field(default=9.81, description="Gravitational acceleration in m/s²", gt=0.0)
    duration: float = Field(default=10.0, description="Simulation duration in seconds", gt=0.0)
    dt: float = Field(default=0.01, description="Time step in seconds", gt=0.0)


class UnderwaterMotionResponse(BaseModel):
    """Response for underwater motion calculation."""

    trajectory: list[list[float]] = Field(..., description="Trajectory points [[t, x, y, z], ...]")
    final_position: list[float] = Field(..., description="Final position [x, y, z] in meters")
    final_velocity: list[float] = Field(..., description="Final velocity [x, y, z] in m/s")
    max_depth: float = Field(..., description="Maximum depth reached (negative y) in meters")
    total_distance: float = Field(..., description="Total distance traveled in meters")
    settled: bool = Field(..., description="Whether object reached equilibrium/settled")
