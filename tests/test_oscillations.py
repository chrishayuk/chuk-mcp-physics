"""Tests for oscillation calculations."""

import math

from chuk_mcp_physics.oscillations import (
    HookesLawRequest,
    SpringMassPeriodRequest,
    SimpleHarmonicMotionRequest,
    DampedOscillationRequest,
    PendulumPeriodRequest,
    calculate_hookes_law,
    calculate_spring_mass_period,
    calculate_simple_harmonic_motion,
    calculate_damped_oscillation,
    calculate_pendulum_period,
)


class TestHookesLaw:
    def test_basic_spring_force(self):
        """Test Hooke's Law calculation."""
        request = HookesLawRequest(
            spring_constant=10000.0,
            displacement=0.05,
        )
        result = calculate_hookes_law(request)

        assert abs(result.force - 500.0) < 0.1
        assert abs(result.potential_energy - 12.5) < 0.1


class TestSpringMassPeriod:
    def test_spring_period(self):
        """Test spring-mass period calculation."""
        request = SpringMassPeriodRequest(
            mass=0.5,
            spring_constant=20.0,
        )
        result = calculate_spring_mass_period(request)

        # T = 2π√(m/k)
        expected = 2.0 * math.pi * math.sqrt(0.5 / 20.0)
        assert abs(result.period - expected) < 0.01
        assert abs(result.frequency - (1.0 / expected)) < 0.01


class TestSimpleHarmonicMotion:
    def test_shm_at_equilibrium(self):
        """Test SHM at equilibrium position."""
        request = SimpleHarmonicMotionRequest(
            amplitude=0.1,
            angular_frequency=5.0,
            time=math.pi / (2.0 * 5.0),  # At quarter period
            phase=0.0,
        )
        result = calculate_simple_harmonic_motion(request)

        # At t = T/4, position should be near 0
        assert abs(result.position) < 0.01

    def test_shm_maximum_displacement(self):
        """Test SHM at maximum displacement."""
        request = SimpleHarmonicMotionRequest(
            amplitude=0.1,
            angular_frequency=5.0,
            time=0.0,
            phase=0.0,
        )
        result = calculate_simple_harmonic_motion(request)

        # At t = 0, position should be amplitude
        assert abs(result.position - 0.1) < 0.01
        assert abs(result.velocity) < 0.01  # Zero velocity at max displacement


class TestPendulumPeriod:
    def test_pendulum_small_angle(self):
        """Test pendulum period for small angles."""
        request = PendulumPeriodRequest(
            length=0.994,
            gravity=9.81,
        )
        result = calculate_pendulum_period(request)

        # Should be approximately 2 seconds
        assert abs(result.period - 2.0) < 0.01
        assert result.small_angle_approximation is True

    def test_pendulum_large_angle(self):
        """Test pendulum period with large angle correction."""
        request = PendulumPeriodRequest(
            length=1.0,
            gravity=9.81,
            amplitude_degrees=30.0,
        )
        result = calculate_pendulum_period(request)

        # Period should be slightly longer than small angle approximation
        small_angle_period = 2.0 * math.pi * math.sqrt(1.0 / 9.81)
        assert result.period > small_angle_period
        assert result.small_angle_approximation is False


class TestDampedOscillation:
    def test_underdamped(self):
        """Test underdamped oscillation."""
        request = DampedOscillationRequest(
            mass=1.0,
            spring_constant=100.0,
            damping_coefficient=2.0,
            time=0.0,
            initial_position=1.0,
            initial_velocity=0.0,
        )
        result = calculate_damped_oscillation(request)

        assert result.regime == "underdamped"
        assert result.damping_ratio < 1.0
        assert abs(result.position - 1.0) < 0.01

    def test_critically_damped(self):
        """Test critically damped oscillation."""
        m = 1.0
        k = 100.0
        # b_critical = 2√(mk)
        b = 2.0 * math.sqrt(m * k)

        request = DampedOscillationRequest(
            mass=m,
            spring_constant=k,
            damping_coefficient=b,
            time=0.1,
            initial_position=1.0,
            initial_velocity=0.0,
        )
        result = calculate_damped_oscillation(request)

        assert result.regime == "critically_damped"
        assert abs(result.damping_ratio - 1.0) < 0.01

    def test_overdamped(self):
        """Test overdamped oscillation."""
        request = DampedOscillationRequest(
            mass=1.0,
            spring_constant=100.0,
            damping_coefficient=50.0,  # Large damping
            time=0.1,
            initial_position=1.0,
            initial_velocity=0.0,
        )
        result = calculate_damped_oscillation(request)

        assert result.regime == "overdamped"
        assert result.damping_ratio > 1.0

    def test_damped_energy_decay(self):
        """Test that energy decays in damped oscillation."""
        request = DampedOscillationRequest(
            mass=1.0,
            spring_constant=100.0,
            damping_coefficient=2.0,
            time=0.0,
            initial_position=1.0,
            initial_velocity=0.0,
        )
        result_t0 = calculate_damped_oscillation(request)

        request.time = 5.0
        result_t5 = calculate_damped_oscillation(request)

        # Position amplitude should decrease over time
        assert abs(result_t5.position) < abs(result_t0.position)
