"""Tests for 2D/3D kinematics calculations."""

from chuk_mcp_physics.kinematics import (
    AccelerationFromPositionRequest,
    AverageSpeedRequest,
    InstantaneousVelocityRequest,
    JerkCalculationRequest,
    MotionGraphRequest,
    TrajectoryFitRequest,
    calculate_acceleration_from_position,
    calculate_average_speed,
    calculate_instantaneous_velocity,
    calculate_jerk,
    fit_trajectory,
    generate_motion_graph,
)


class TestAccelerationFromPosition:
    def test_constant_velocity(self):
        """Test constant velocity (zero acceleration)."""
        request = AccelerationFromPositionRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [15.0, 0.0, 0.0]],
        )
        result = calculate_acceleration_from_position(request)

        # Velocity should be constant ~5 m/s in x
        assert len(result.velocities) == 4
        for vel in result.velocities:
            assert abs(vel[0] - 5.0) < 0.5  # vx ~5

        # Acceleration should be ~0
        assert len(result.accelerations) == 4
        for acc in result.accelerations:
            assert abs(acc[0]) < 0.5  # ax ~0

    def test_constant_acceleration(self):
        """Test constant acceleration (x = 0.5*a*t²)."""
        # x = 0.5 * 2 * t² = t²
        request = AccelerationFromPositionRequest(
            times=[0.0, 1.0, 2.0, 3.0, 4.0],
            positions=[
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [9.0, 0.0, 0.0],
                [16.0, 0.0, 0.0],
            ],
        )
        result = calculate_acceleration_from_position(request)

        # Acceleration should be constant ~2 m/s² (numerical derivative has some error)
        for acc in result.accelerations[1:-1]:  # Skip edges
            assert abs(acc[0] - 2.0) < 0.6  # Relaxed tolerance for numerical derivative

    def test_2d_motion(self):
        """Test 2D motion."""
        request = AccelerationFromPositionRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[
                [0.0, 0.0, 0.0],
                [3.0, 4.0, 0.0],
                [6.0, 8.0, 0.0],
                [9.0, 12.0, 0.0],
            ],
        )
        result = calculate_acceleration_from_position(request)

        assert len(result.velocities) == 4
        assert len(result.accelerations) == 4
        # Should have constant velocity [3, 4, 0]
        for vel in result.velocities:
            assert abs(vel[0] - 3.0) < 0.5
            assert abs(vel[1] - 4.0) < 0.5


class TestJerkCalculation:
    def test_zero_jerk(self):
        """Test zero jerk (constant acceleration)."""
        request = JerkCalculationRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            accelerations=[
                [5.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
            ],
        )
        result = calculate_jerk(request)

        # Jerk should be ~0
        for jerk in result.jerks:
            assert abs(jerk[0]) < 0.1

    def test_constant_jerk(self):
        """Test constant jerk (linear acceleration change)."""
        request = JerkCalculationRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            accelerations=[
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [6.0, 0.0, 0.0],
            ],
        )
        result = calculate_jerk(request)

        # Jerk should be constant ~2 m/s³
        for jerk in result.jerks[1:-1]:  # Skip edges
            assert abs(jerk[0] - 2.0) < 0.5

    def test_3d_jerk(self):
        """Test jerk in 3D."""
        request = JerkCalculationRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            accelerations=[
                [0.0, 0.0, 0.0],
                [3.0, 4.0, 0.0],
                [6.0, 8.0, 0.0],
                [9.0, 12.0, 0.0],
            ],
        )
        result = calculate_jerk(request)

        # Jerk magnitude should be sqrt(3² + 4²) = 5 m/s³
        assert result.max_jerk_magnitude > 4.5


class TestTrajectoryFit:
    def test_linear_fit(self):
        """Test linear trajectory fit."""
        request = TrajectoryFitRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[
                [0.0, 1.0, 0.0],
                [2.0, 3.0, 0.0],
                [4.0, 5.0, 0.0],
                [6.0, 7.0, 0.0],
            ],
            fit_type="linear",
        )
        result = fit_trajectory(request)

        # x(t) = 2t, so coefficients should be [0, 2]
        assert abs(result.coefficients_x[0]) < 0.1  # Intercept ~0
        assert abs(result.coefficients_x[1] - 2.0) < 0.1  # Slope ~2

        # y(t) = 1 + 2t, so coefficients should be [1, 2]
        assert abs(result.coefficients_y[0] - 1.0) < 0.1  # Intercept ~1
        assert abs(result.coefficients_y[1] - 2.0) < 0.1  # Slope ~2

        # R² should be perfect
        assert result.r_squared > 0.99

    def test_quadratic_fit(self):
        """Test quadratic trajectory fit (parabolic motion)."""
        request = TrajectoryFitRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [9.0, 0.0, 0.0],
            ],
            fit_type="quadratic",
        )
        result = fit_trajectory(request)

        # x(t) = t², so coefficients should be [0, 0, 1]
        assert abs(result.coefficients_x[0]) < 0.1  # Constant ~0
        assert abs(result.coefficients_x[1]) < 0.1  # Linear ~0
        assert abs(result.coefficients_x[2] - 1.0) < 0.1  # Quadratic ~1

        # R² should be perfect
        assert result.r_squared > 0.99

    def test_projectile_fit(self):
        """Test fitting projectile motion."""
        # Projectile: x = 10t, y = 20t - 5t²
        request = TrajectoryFitRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[
                [0.0, 0.0, 0.0],
                [10.0, 15.0, 0.0],
                [20.0, 20.0, 0.0],
                [30.0, 15.0, 0.0],
            ],
            fit_type="quadratic",
        )
        result = fit_trajectory(request)

        # x(t) = 10t: [0, 10, 0]
        assert abs(result.coefficients_x[1] - 10.0) < 0.5

        # y(t) = 20t - 5t²: [0, 20, -5]
        assert abs(result.coefficients_y[1] - 20.0) < 0.5
        assert abs(result.coefficients_y[2] - (-5.0)) < 0.5

        assert result.r_squared > 0.99


class TestMotionGraph:
    def test_x_component(self):
        """Test motion graph for X component."""
        request = MotionGraphRequest(
            times=[0.0, 1.0, 2.0, 3.0, 4.0],
            positions=[
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [9.0, 0.0, 0.0],
                [16.0, 0.0, 0.0],
            ],
            component="x",
        )
        result = generate_motion_graph(request)

        assert len(result.times) == 5
        assert len(result.positions) == 5
        assert len(result.velocities) == 5
        assert len(result.accelerations) == 5
        assert result.component == "x"

    def test_magnitude_component(self):
        """Test motion graph for magnitude."""
        request = MotionGraphRequest(
            times=[0.0, 1.0, 2.0],
            positions=[
                [0.0, 0.0, 0.0],
                [3.0, 4.0, 0.0],
                [6.0, 8.0, 0.0],
            ],
            component="magnitude",
        )
        result = generate_motion_graph(request)

        assert len(result.positions) == 3
        # Magnitude should be 5 m (3-4-5 triangle)
        assert abs(result.positions[1] - 5.0) < 0.1


class TestAverageSpeed:
    def test_straight_line(self):
        """Test average speed on straight line."""
        request = AverageSpeedRequest(
            positions=[
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [20.0, 0.0, 0.0],
                [30.0, 0.0, 0.0],
            ],
            times=[0.0, 1.0, 2.0, 3.0],
        )
        result = calculate_average_speed(request)

        # Distance = 30 m, time = 3 s, speed = 10 m/s
        assert abs(result.average_speed - 10.0) < 0.1
        assert abs(result.total_distance - 30.0) < 0.1
        assert abs(result.total_time - 3.0) < 0.01
        assert abs(result.displacement_magnitude - 30.0) < 0.1

    def test_back_and_forth(self):
        """Test path that goes back and forth."""
        request = AverageSpeedRequest(
            positions=[
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            times=[0.0, 1.0, 2.0],
        )
        result = calculate_average_speed(request)

        # Distance = 20 m (10 + 10), displacement = 0 m
        assert abs(result.total_distance - 20.0) < 0.1
        assert abs(result.displacement_magnitude) < 0.1
        assert abs(result.average_speed - 10.0) < 0.1

    def test_3d_path(self):
        """Test 3D path."""
        request = AverageSpeedRequest(
            positions=[
                [0.0, 0.0, 0.0],
                [3.0, 4.0, 0.0],  # Distance = 5
                [6.0, 8.0, 0.0],  # Distance = 5
            ],
            times=[0.0, 1.0, 2.0],
        )
        result = calculate_average_speed(request)

        # Total distance = 10 m
        assert abs(result.total_distance - 10.0) < 0.1
        assert abs(result.average_speed - 5.0) < 0.1


class TestInstantaneousVelocity:
    def test_exact_time_match(self):
        """Test velocity at exact time point."""
        request = InstantaneousVelocityRequest(
            positions=[
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
            ],
            times=[0.0, 1.0, 2.0],
            target_time=1.0,
        )
        result = calculate_instantaneous_velocity(request)

        # Velocity at t=1 should be ~5 m/s
        assert abs(result.velocity[0] - 5.0) < 0.5
        assert abs(result.speed - 5.0) < 0.5
        assert result.interpolated is False

    def test_interpolated_velocity(self):
        """Test velocity interpolation."""
        request = InstantaneousVelocityRequest(
            positions=[
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [20.0, 0.0, 0.0],
            ],
            times=[0.0, 1.0, 2.0],
            target_time=0.5,
        )
        result = calculate_instantaneous_velocity(request)

        # Velocity should be interpolated
        assert result.interpolated is True
        assert result.speed > 0

    def test_2d_velocity(self):
        """Test 2D velocity calculation."""
        request = InstantaneousVelocityRequest(
            positions=[
                [0.0, 0.0, 0.0],
                [3.0, 4.0, 0.0],
                [6.0, 8.0, 0.0],
            ],
            times=[0.0, 1.0, 2.0],
            target_time=1.0,
        )
        result = calculate_instantaneous_velocity(request)

        # Velocity should be [3, 4, 0], speed = 5
        assert abs(result.velocity[0] - 3.0) < 0.5
        assert abs(result.velocity[1] - 4.0) < 0.5
        assert abs(result.speed - 5.0) < 0.5
