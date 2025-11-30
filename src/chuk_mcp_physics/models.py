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
    radius: Optional[float] = Field(None, description="Radius for sphere/capsule/cylinder")
    half_height: Optional[float] = Field(None, description="Half-height for capsule/cylinder")
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
