"""Tests for orientation-dependent drag in Rapier simulations.

These tests verify that the orientation-dependent drag parameters are properly
handled in the RigidBodyDefinition model and passed to the Rapier provider.

Note: These tests validate the API/model layer. The actual physics calculation
of orientation-dependent drag forces is implemented in the Rapier service (Rust).
"""

import pytest
from chuk_mcp_physics.models import RigidBodyDefinition, ShapeType


def _make_body(**kwargs) -> RigidBodyDefinition:
    """Helper to create RigidBodyDefinition with sensible defaults."""
    defaults = {
        "id": "test",
        "shape": ShapeType.SPHERE,
        "size": [1.0],
        "position": [0.0, 0.0, 0.0],
        "mass": 1.0,
    }
    defaults.update(kwargs)
    return RigidBodyDefinition(**defaults)


class TestOrientationDragParameters:
    """Test orientation-dependent drag parameter validation and defaults."""

    def test_drag_parameters_optional(self):
        """Test that drag parameters are optional."""
        body = _make_body()

        assert body.drag_coefficient is None
        assert body.drag_area is None
        assert body.drag_axis_ratios is None
        assert body.fluid_density == 1.225  # Default air density

    def test_drag_parameters_set(self):
        """Test setting all drag parameters."""
        body = _make_body(
            drag_coefficient=0.47,
            drag_area=0.01,
            drag_axis_ratios=[1.0, 0.3, 1.0],
            fluid_density=1000.0,  # Water
        )

        assert body.drag_coefficient == 0.47
        assert body.drag_area == 0.01
        assert body.drag_axis_ratios == [1.0, 0.3, 1.0]
        assert body.fluid_density == 1000.0

    def test_drag_coefficient_non_negative(self):
        """Test that drag coefficient must be non-negative."""
        with pytest.raises(ValueError):
            _make_body(drag_coefficient=-0.5)

    def test_drag_area_positive(self):
        """Test that drag area must be positive."""
        with pytest.raises(ValueError):
            _make_body(drag_area=0.0)

        with pytest.raises(ValueError):
            _make_body(drag_area=-0.01)

    def test_fluid_density_positive(self):
        """Test that fluid density must be positive."""
        with pytest.raises(ValueError):
            _make_body(fluid_density=0.0)

    def test_drag_axis_ratios_three_values(self):
        """Test that drag_axis_ratios accepts three values."""
        body = _make_body(drag_axis_ratios=[0.5, 1.0, 0.8])

        assert len(body.drag_axis_ratios) == 3

    def test_typical_sports_ball_parameters(self):
        """Test realistic parameters for common sports objects."""
        # Baseball
        baseball = _make_body(
            id="baseball",
            position=[0, 2, 0],
            velocity=[40, 5, 0],
            mass=0.145,
            drag_coefficient=0.4,
            drag_area=0.0043,  # π * (0.037)²
            fluid_density=1.225,
        )

        assert baseball.drag_coefficient == 0.4
        assert baseball.drag_area == 0.0043

        # Football (streamlined)
        football = _make_body(
            id="football",
            position=[0, 2, 0],
            velocity=[25, 10, 0],
            mass=0.42,
            drag_coefficient=0.1,
            drag_area=0.023,
            drag_axis_ratios=[1.0, 0.2, 1.0],  # Streamlined along Y
            fluid_density=1.225,
        )

        assert football.drag_axis_ratios == [1.0, 0.2, 1.0]

        # Frisbee (flat)
        frisbee = _make_body(
            id="frisbee",
            position=[0, 1.5, 0],
            velocity=[20, 2, 0],
            mass=0.175,
            drag_coefficient=0.08,
            drag_area=0.057,  # π * (0.135)²
            drag_axis_ratios=[1.0, 0.1, 1.0],  # Very low drag when flat
            fluid_density=1.225,
        )

        assert frisbee.drag_coefficient == 0.08


class TestOrientationDragSerialization:
    """Test that drag parameters serialize correctly.

    These tests verify that the orientation-dependent drag parameters are
    properly handled in the model serialization. The actual Rapier physics
    simulation of orientation-dependent drag requires server-side implementation.
    """

    def test_serialize_body_with_all_drag_parameters(self):
        """Test serializing body with all drag parameters."""
        body = _make_body(
            id="test_body",
            position=[0.0, 10.0, 0.0],
            velocity=[10.0, 0.0, 0.0],
            drag_coefficient=0.47,
            drag_area=0.01,
            drag_axis_ratios=[1.0, 0.5, 1.0],
            fluid_density=1.225,
        )

        # Test serialization
        data = body.model_dump()

        assert data["drag_coefficient"] == 0.47
        assert data["drag_area"] == 0.01
        assert data["drag_axis_ratios"] == [1.0, 0.5, 1.0]
        assert data["fluid_density"] == 1.225

    def test_serialize_body_without_drag(self):
        """Test serializing body without drag parameters."""
        body = _make_body(
            id="test_body",
            position=[0.0, 10.0, 0.0],
            velocity=[10.0, 0.0, 0.0],
        )

        data = body.model_dump()

        assert data["drag_coefficient"] is None
        assert data["drag_area"] is None
        assert data["drag_axis_ratios"] is None
        assert data["fluid_density"] == 1.225  # Default

    def test_serialize_football_parameters(self):
        """Test serializing realistic football spiral parameters."""
        import math

        v0 = 25.0  # m/s
        angle = 30.0  # degrees
        vx = v0 * math.cos(math.radians(angle))
        vy = v0 * math.sin(math.radians(angle))

        body = _make_body(
            id="football",
            position=[0.0, 2.0, 0.0],
            velocity=[vx, vy, 0.0],
            mass=0.42,  # NFL football
            drag_coefficient=0.1,
            drag_area=0.023,
            drag_axis_ratios=[1.0, 0.2, 1.0],  # Streamlined
            fluid_density=1.225,
            angular_velocity=[0.0, 126.0, 0.0],  # 20 rev/s spiral
        )

        data = body.model_dump()

        assert data["drag_coefficient"] == 0.1
        assert data["drag_area"] == 0.023
        assert data["drag_axis_ratios"] == [1.0, 0.2, 1.0]
        assert data["angular_velocity"] == [0.0, 126.0, 0.0]

    def test_serialize_frisbee_parameters(self):
        """Test serializing realistic frisbee parameters."""
        import math

        v0 = 20.0  # m/s
        angle = 10.0  # degrees
        vx = v0 * math.cos(math.radians(angle))
        vy = v0 * math.sin(math.radians(angle))

        body = _make_body(
            id="frisbee",
            position=[0.0, 1.5, 0.0],
            velocity=[vx, vy, 0.0],
            mass=0.175,
            drag_coefficient=0.08,
            drag_area=0.057,
            drag_axis_ratios=[1.0, 0.1, 1.0],  # Very flat
            fluid_density=1.225,
            angular_velocity=[0.0, 62.8, 0.0],  # 600 rpm
        )

        data = body.model_dump()

        assert data["drag_coefficient"] == 0.08
        assert data["drag_area"] == 0.057
        assert data["drag_axis_ratios"] == [1.0, 0.1, 1.0]

    def test_serialize_water_drag(self):
        """Test serializing drag in water (high fluid density)."""
        body = _make_body(
            id="water_ball",
            position=[0.0, 5.0, 0.0],
            velocity=[10.0, 0.0, 0.0],
            drag_coefficient=0.47,
            drag_area=0.01,
            fluid_density=1000.0,  # Water
        )

        data = body.model_dump()

        assert data["fluid_density"] == 1000.0

    def test_serialize_different_axis_ratios(self):
        """Test serializing various drag axis ratio configurations."""
        # Streamlined along X
        body_x = _make_body(
            id="x_stream",
            drag_coefficient=0.5,
            drag_area=0.01,
            drag_axis_ratios=[0.2, 1.0, 1.0],  # Low drag along X
        )

        data_x = body_x.model_dump()
        assert data_x["drag_axis_ratios"] == [0.2, 1.0, 1.0]

        # Streamlined along Z
        body_z = _make_body(
            id="z_stream",
            drag_coefficient=0.5,
            drag_area=0.01,
            drag_axis_ratios=[1.0, 1.0, 0.2],  # Low drag along Z
        )

        data_z = body_z.model_dump()
        assert data_z["drag_axis_ratios"] == [1.0, 1.0, 0.2]

    def test_serialize_symmetric_drag(self):
        """Test serializing symmetric drag (equal in all directions)."""
        body = _make_body(
            id="sphere",
            drag_coefficient=0.47,
            drag_area=0.01,
            drag_axis_ratios=[1.0, 1.0, 1.0],  # Symmetric
        )

        data = body.model_dump()
        assert data["drag_axis_ratios"] == [1.0, 1.0, 1.0]


class TestDragCoefficientReferences:
    """Test that common drag coefficient values are documented and reasonable."""

    def test_sphere_drag(self):
        """Sphere Cd ~ 0.47."""
        sphere = _make_body(
            drag_coefficient=0.47,
            drag_area=0.01,
        )
        assert 0.4 <= sphere.drag_coefficient <= 0.5

    def test_streamlined_objects(self):
        """Streamlined objects have low Cd."""
        # Football spiral
        football = _make_body(
            mass=0.42,
            drag_coefficient=0.1,
            drag_area=0.023,
        )
        assert football.drag_coefficient < 0.2

        # Frisbee flat
        frisbee = _make_body(
            mass=0.175,
            drag_coefficient=0.08,
            drag_area=0.057,
        )
        assert frisbee.drag_coefficient < 0.15

    def test_bluff_objects(self):
        """Bluff objects have high Cd."""
        # Flat plate perpendicular
        plate = _make_body(
            drag_coefficient=1.2,
            drag_area=0.1,
        )
        assert plate.drag_coefficient > 1.0

    def test_zero_drag(self):
        """Test zero drag coefficient (frictionless case)."""
        frictionless = _make_body(
            drag_coefficient=0.0,
            drag_area=0.01,
        )
        assert frictionless.drag_coefficient == 0.0


class TestFluidDensities:
    """Test common fluid densities."""

    def test_air_sea_level(self):
        """Standard air density at sea level, 15°C."""
        body = _make_body(fluid_density=1.225)
        assert abs(body.fluid_density - 1.225) < 0.001

    def test_water(self):
        """Water density at 4°C."""
        body = _make_body(fluid_density=1000.0)
        assert body.fluid_density == 1000.0

    def test_high_altitude_air(self):
        """Air density at high altitude (Denver, 1600m)."""
        # Roughly 83% of sea level density
        body = _make_body(fluid_density=1.02)
        assert 1.0 <= body.fluid_density <= 1.1
