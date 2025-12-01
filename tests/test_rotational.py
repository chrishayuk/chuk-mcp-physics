"""Tests for rotational dynamics calculations."""

from chuk_mcp_physics.rotational import (
    TorqueRequest,
    MomentOfInertiaRequest,
    AngularMomentumRequest,
    RotationalKineticEnergyRequest,
    AngularAccelerationRequest,
    calculate_torque,
    calculate_moment_of_inertia,
    calculate_angular_momentum,
    calculate_rotational_kinetic_energy,
    calculate_angular_acceleration,
)


class TestTorque:
    def test_perpendicular_force(self):
        """Test torque with force perpendicular to position."""
        request = TorqueRequest(
            force=[50.0, 0.0, 0.0],
            position=[0.0, 0.0, 0.8],
        )
        result = calculate_torque(request)

        # τ = r × F, should be 40 N⋅m in y-direction
        assert abs(result.torque[1] - 40.0) < 0.01
        assert abs(result.magnitude - 40.0) < 0.01

    def test_parallel_force_zero_torque(self):
        """Test that parallel force produces zero torque."""
        request = TorqueRequest(
            force=[10.0, 0.0, 0.0],
            position=[1.0, 0.0, 0.0],
        )
        result = calculate_torque(request)

        assert result.magnitude < 0.01


class TestMomentOfInertia:
    def test_solid_sphere(self):
        """Test moment of inertia for solid sphere."""
        request = MomentOfInertiaRequest(
            shape="solid_sphere",
            mass=5.0,
            radius=0.3,
        )
        result = calculate_moment_of_inertia(request)

        # I = (2/5) * m * r²
        expected = (2.0 / 5.0) * 5.0 * 0.3 * 0.3
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_disk(self):
        """Test moment of inertia for disk."""
        request = MomentOfInertiaRequest(
            shape="disk",
            mass=5.0,
            radius=0.3,
        )
        result = calculate_moment_of_inertia(request)

        # I = (1/2) * m * r²
        expected = 0.5 * 5.0 * 0.3 * 0.3
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_rod_center(self):
        """Test moment of inertia for rod about center."""
        request = MomentOfInertiaRequest(
            shape="rod",
            mass=2.0,
            length=1.0,
            axis="center",
        )
        result = calculate_moment_of_inertia(request)

        # I = (1/12) * m * L²
        expected = (1.0 / 12.0) * 2.0 * 1.0 * 1.0
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_rod_end(self):
        """Test moment of inertia for rod about end."""
        request = MomentOfInertiaRequest(
            shape="rod",
            mass=2.0,
            length=1.0,
            axis="end",
        )
        result = calculate_moment_of_inertia(request)

        # I = (1/3) * m * L²
        expected = (1.0 / 3.0) * 2.0 * 1.0 * 1.0
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_box_x_axis(self):
        """Test moment of inertia for box about x-axis."""
        request = MomentOfInertiaRequest(
            shape="box",
            mass=10.0,
            width=2.0,
            height=3.0,
            depth=4.0,
            axis="x",
        )
        result = calculate_moment_of_inertia(request)

        # I = (1/12) * m * (h² + d²)
        expected = (1.0 / 12.0) * 10.0 * (3.0 * 3.0 + 4.0 * 4.0)
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_box_y_axis(self):
        """Test moment of inertia for box about y-axis."""
        request = MomentOfInertiaRequest(
            shape="box",
            mass=10.0,
            width=2.0,
            height=3.0,
            depth=4.0,
            axis="y",
        )
        result = calculate_moment_of_inertia(request)

        # I = (1/12) * m * (w² + d²)
        expected = (1.0 / 12.0) * 10.0 * (2.0 * 2.0 + 4.0 * 4.0)
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_box_z_axis(self):
        """Test moment of inertia for box about z-axis."""
        request = MomentOfInertiaRequest(
            shape="box",
            mass=10.0,
            width=2.0,
            height=3.0,
            depth=4.0,
            axis="z",
        )
        result = calculate_moment_of_inertia(request)

        # I = (1/12) * m * (w² + h²)
        expected = (1.0 / 12.0) * 10.0 * (2.0 * 2.0 + 3.0 * 3.0)
        assert abs(result.moment_of_inertia - expected) < 0.001

    def test_hollow_sphere(self):
        """Test moment of inertia for hollow sphere."""
        request = MomentOfInertiaRequest(
            shape="hollow_sphere",
            mass=5.0,
            radius=0.3,
        )
        result = calculate_moment_of_inertia(request)

        # I = (2/3) * m * r²
        expected = (2.0 / 3.0) * 5.0 * 0.3 * 0.3
        assert abs(result.moment_of_inertia - expected) < 0.001


class TestAngularMomentum:
    def test_simple_angular_momentum(self):
        """Test angular momentum calculation."""
        request = AngularMomentumRequest(
            moment_of_inertia=3.0,
            angular_velocity=[0.0, 5.0, 0.0],
        )
        result = calculate_angular_momentum(request)

        # L = I * ω
        assert abs(result.angular_momentum[1] - 15.0) < 0.01
        assert abs(result.magnitude - 15.0) < 0.01


class TestRotationalKineticEnergy:
    def test_rotational_ke(self):
        """Test rotational kinetic energy."""
        request = RotationalKineticEnergyRequest(
            moment_of_inertia=0.5,
            angular_velocity=100.0,
        )
        result = calculate_rotational_kinetic_energy(request)

        # KE = (1/2) * I * ω²
        expected = 0.5 * 0.5 * 100.0 * 100.0
        assert abs(result.rotational_ke - expected) < 0.1


class TestAngularAcceleration:
    def test_angular_acceleration(self):
        """Test angular acceleration."""
        request = AngularAccelerationRequest(
            torque=10.0,
            moment_of_inertia=0.5,
        )
        result = calculate_angular_acceleration(request)

        # α = τ / I
        expected = 10.0 / 0.5
        assert abs(result.angular_acceleration - expected) < 0.01
