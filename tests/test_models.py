"""Tests for Pydantic models."""

import pytest
from pydantic import ValidationError
from chuk_mcp_physics.models import (
    Vector3,
    Quaternion,
    RigidBodyDefinition,
    BodyType,
    ShapeType,
    ProjectileMotionRequest,
    CollisionCheckRequest,
)


class TestVector3:
    """Test Vector3 model."""

    def test_create_vector(self):
        """Create a vector with x, y, z."""
        v = Vector3(x=1.0, y=2.0, z=3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_to_list(self):
        """Convert vector to list."""
        v = Vector3(x=1.0, y=2.0, z=3.0)
        assert v.to_list() == [1.0, 2.0, 3.0]

    def test_from_list(self):
        """Create vector from list."""
        v = Vector3.from_list([4.0, 5.0, 6.0])
        assert v.x == 4.0
        assert v.y == 5.0
        assert v.z == 6.0


class TestQuaternion:
    """Test Quaternion model."""

    def test_create_quaternion(self):
        """Create a quaternion."""
        q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        assert q.x == 0.0
        assert q.w == 1.0

    def test_identity(self):
        """Identity quaternion."""
        q = Quaternion.identity()
        assert q.x == 0.0
        assert q.y == 0.0
        assert q.z == 0.0
        assert q.w == 1.0

    def test_to_list(self):
        """Convert to list."""
        q = Quaternion(x=0.1, y=0.2, z=0.3, w=0.4)
        assert q.to_list() == [0.1, 0.2, 0.3, 0.4]

    def test_from_list(self):
        """Create from list."""
        q = Quaternion.from_list([0.5, 0.6, 0.7, 0.8])
        assert q.x == 0.5
        assert q.w == 0.8


class TestRigidBodyDefinition:
    """Test RigidBodyDefinition model."""

    def test_valid_box(self):
        """Valid box definition."""
        body = RigidBodyDefinition(
            id="test_box",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.BOX,
            size=[2.0, 1.0, 3.0],
            mass=10.0,
        )
        assert body.id == "test_box"
        assert body.kind == BodyType.DYNAMIC
        assert body.shape == ShapeType.BOX
        assert body.size == [2.0, 1.0, 3.0]

    def test_valid_sphere(self):
        """Valid sphere definition."""
        body = RigidBodyDefinition(
            id="ball",
            kind=BodyType.DYNAMIC,
            shape=ShapeType.SPHERE,
            size=[0.5],
            mass=5.0,
        )
        assert body.size == [0.5]

    def test_defaults(self):
        """Check default values."""
        body = RigidBodyDefinition(
            id="simple",
            shape=ShapeType.BOX,
            size=[1, 1, 1],
        )
        assert body.kind == BodyType.DYNAMIC
        assert body.mass == 1.0
        assert body.position == [0.0, 0.0, 0.0]
        assert body.orientation == [0.0, 0.0, 0.0, 1.0]
        assert body.restitution == 0.5
        assert body.friction == 0.5

    def test_invalid_mass(self):
        """Mass must be positive."""
        with pytest.raises(ValidationError):
            RigidBodyDefinition(
                id="bad",
                shape=ShapeType.BOX,
                size=[1, 1, 1],
                mass=-5.0,  # Invalid
            )

    def test_invalid_restitution(self):
        """Restitution must be 0-1."""
        with pytest.raises(ValidationError):
            RigidBodyDefinition(
                id="bad",
                shape=ShapeType.BOX,
                size=[1, 1, 1],
                restitution=1.5,  # Invalid
            )


class TestProjectileMotionRequest:
    """Test ProjectileMotionRequest validation."""

    def test_valid_request(self):
        """Valid projectile motion request."""
        req = ProjectileMotionRequest(
            initial_velocity=20.0,
            angle_degrees=45.0,
        )
        assert req.initial_velocity == 20.0
        assert req.angle_degrees == 45.0
        assert req.gravity == 9.81  # Default

    def test_invalid_velocity(self):
        """Velocity must be positive."""
        with pytest.raises(ValidationError):
            ProjectileMotionRequest(
                initial_velocity=-10.0,  # Invalid
                angle_degrees=45.0,
            )

    def test_invalid_angle(self):
        """Angle must be 0-90."""
        with pytest.raises(ValidationError):
            ProjectileMotionRequest(
                initial_velocity=20.0,
                angle_degrees=120.0,  # Invalid
            )

    def test_negative_angle(self):
        """Negative angle is invalid."""
        with pytest.raises(ValidationError):
            ProjectileMotionRequest(
                initial_velocity=20.0,
                angle_degrees=-10.0,  # Invalid
            )


class TestCollisionCheckRequest:
    """Test CollisionCheckRequest validation."""

    def test_valid_request(self):
        """Valid collision check."""
        req = CollisionCheckRequest(
            body1_position=[0, 0, 0],
            body1_velocity=[1, 0, 0],
            body1_radius=1.0,
            body2_position=[10, 0, 0],
            body2_velocity=[-1, 0, 0],
            body2_radius=1.0,
        )
        assert req.body1_radius == 1.0

    def test_invalid_radius(self):
        """Radius must be positive."""
        with pytest.raises(ValidationError):
            CollisionCheckRequest(
                body1_position=[0, 0, 0],
                body1_velocity=[1, 0, 0],
                body1_radius=-1.0,  # Invalid
                body2_position=[10, 0, 0],
                body2_velocity=[-1, 0, 0],
                body2_radius=1.0,
            )

    def test_default_max_time(self):
        """Default max_time should be 10."""
        req = CollisionCheckRequest(
            body1_position=[0, 0, 0],
            body1_velocity=[1, 0, 0],
            body1_radius=1.0,
            body2_position=[10, 0, 0],
            body2_velocity=[-1, 0, 0],
            body2_radius=1.0,
        )
        assert req.max_time == 10.0
