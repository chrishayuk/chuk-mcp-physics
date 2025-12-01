"""Additional tests to improve coverage for kinematics edge cases."""

from chuk_mcp_physics.kinematics import (
    calculate_acceleration_from_position,
    calculate_jerk,
    fit_trajectory,
    generate_motion_graph,
    calculate_average_speed,
    calculate_instantaneous_velocity,
    AccelerationFromPositionRequest,
    JerkCalculationRequest,
    TrajectoryFitRequest,
    MotionGraphRequest,
    AverageSpeedRequest,
    InstantaneousVelocityRequest,
)


class TestEdgeCases:
    """Test edge cases for better coverage."""

    def test_minimum_three_points(self):
        """Test acceleration with minimum three position points."""
        request = AccelerationFromPositionRequest(
            times=[0.0, 1.0, 2.0],
            positions=[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [2.0, 2.0, 2.0]],
        )
        result = calculate_acceleration_from_position(request)
        assert len(result.velocities) == 3
        assert len(result.accelerations) == 3

    def test_jerk_two_points(self):
        """Test jerk with two acceleration points."""
        request = JerkCalculationRequest(
            times=[0.0, 1.0],
            accelerations=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        )
        result = calculate_jerk(request)
        assert len(result.jerks) == 2

    def test_trajectory_fit_cubic(self):
        """Test cubic trajectory fitting."""
        request = TrajectoryFitRequest(
            times=[0.0, 1.0, 2.0, 3.0, 4.0],
            positions=[[0, 0, 0], [1, 1, 0], [4, 4, 0], [9, 9, 0], [16, 16, 0]],
            fit_type="cubic",
        )
        result = fit_trajectory(request)
        assert result.r_squared >= 0.9  # Good fit
        assert len(result.coefficients_x) == 4  # Cubic has 4 coefficients

    def test_trajectory_fit_linear(self):
        """Test linear trajectory fitting."""
        request = TrajectoryFitRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[[0, 0, 0], [1, 2, 0], [2, 4, 0], [3, 6, 0]],
            fit_type="linear",
        )
        result = fit_trajectory(request)
        assert result.r_squared > 0.99  # Perfect linear fit
        assert len(result.coefficients_x) == 2  # Linear has 2 coefficients

    def test_motion_graph_y_component(self):
        """Test motion graph with y component."""
        request = MotionGraphRequest(
            times=[0.0, 1.0, 2.0, 3.0],
            positions=[[0, 0, 0], [1, 5, 0], [2, 10, 0], [3, 15, 0]],
            component="y",
        )
        result = generate_motion_graph(request)
        assert result.component == "y"
        assert len(result.positions) == 4

    def test_motion_graph_z_component(self):
        """Test motion graph with z component."""
        request = MotionGraphRequest(
            times=[0.0, 1.0, 2.0],
            positions=[[0, 0, 0], [0, 0, 5], [0, 0, 10]],
            component="z",
        )
        result = generate_motion_graph(request)
        assert result.component == "z"

    def test_average_speed_single_segment(self):
        """Test average speed with just two points."""
        request = AverageSpeedRequest(
            positions=[[0, 0, 0], [3, 4, 0]],
            times=[0.0, 1.0],
        )
        result = calculate_average_speed(request)
        # Distance = 5, time = 1, speed = 5
        assert abs(result.average_speed - 5.0) < 0.01
        assert result.total_distance == 5.0

    def test_instantaneous_velocity_first_point(self):
        """Test instantaneous velocity at first time point."""
        request = InstantaneousVelocityRequest(
            positions=[[0, 0, 0], [1, 1, 0], [2, 2, 0]],
            times=[0.0, 1.0, 2.0],
            target_time=0.0,
        )
        result = calculate_instantaneous_velocity(request)
        assert not result.interpolated
        assert result.speed > 0

    def test_instantaneous_velocity_last_point(self):
        """Test instantaneous velocity at last time point."""
        request = InstantaneousVelocityRequest(
            positions=[[0, 0, 0], [1, 1, 0], [2, 2, 0]],
            times=[0.0, 1.0, 2.0],
            target_time=2.0,
        )
        result = calculate_instantaneous_velocity(request)
        assert not result.interpolated
        assert result.speed > 0

    def test_instantaneous_velocity_between_points(self):
        """Test interpolated velocity between data points."""
        request = InstantaneousVelocityRequest(
            positions=[[0, 0, 0], [2, 2, 0], [4, 4, 0]],
            times=[0.0, 1.0, 2.0],
            target_time=0.5,  # Halfway between first and second
        )
        result = calculate_instantaneous_velocity(request)
        assert result.interpolated
        assert result.speed > 0

    def test_zero_displacement_average_speed(self):
        """Test average speed when returning to start."""
        request = AverageSpeedRequest(
            positions=[[0, 0, 0], [5, 0, 0], [0, 0, 0]],
            times=[0.0, 1.0, 2.0],
        )
        result = calculate_average_speed(request)
        # Total distance = 10 (out and back), displacement = 0
        assert result.total_distance == 10.0
        assert result.displacement_magnitude == 0.0
        assert result.average_speed == 5.0

    def test_3d_trajectory(self):
        """Test 3D motion path."""
        request = AverageSpeedRequest(
            positions=[[0, 0, 0], [1, 1, 1], [2, 2, 2]],
            times=[0.0, 1.0, 2.0],
        )
        result = calculate_average_speed(request)
        import math

        # Distance for each segment: sqrt(3)
        expected_distance = 2 * math.sqrt(3)
        assert abs(result.total_distance - expected_distance) < 0.01
