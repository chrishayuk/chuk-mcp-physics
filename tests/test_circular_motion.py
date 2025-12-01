"""Tests for circular motion calculations."""

import math

from chuk_mcp_physics.circular_motion import (
    CentripetalForceRequest,
    OrbitalPeriodRequest,
    BankingAngleRequest,
    EscapeVelocityRequest,
    CircularOrbitRequest,
    calculate_centripetal_force,
    calculate_orbital_period,
    calculate_banking_angle,
    calculate_escape_velocity,
    analyze_circular_orbit,
)


class TestCentripetalForce:
    def test_car_turning(self):
        """Test centripetal force for a car turning."""
        request = CentripetalForceRequest(
            mass=1500.0,
            velocity=20.0,
            radius=50.0,
        )
        result = calculate_centripetal_force(request)

        # F_c = m * v² / r
        expected = 1500.0 * 20.0 * 20.0 / 50.0
        assert abs(result.centripetal_force - expected) < 1.0
        assert abs(result.centripetal_acceleration - (20.0 * 20.0 / 50.0)) < 0.1


class TestOrbitalPeriod:
    def test_iss_orbit(self):
        """Test orbital period for ISS."""
        request = OrbitalPeriodRequest(
            orbital_radius=6.771e6,  # meters
            central_mass=5.972e24,  # Earth mass
        )
        result = calculate_orbital_period(request)

        # Period should be around 90-95 minutes
        assert 5000 < result.period < 6000  # seconds
        period_minutes = result.period / 60.0
        assert 80 < period_minutes < 100


class TestBankingAngle:
    def test_highway_ramp(self):
        """Test banking angle for highway ramp."""
        request = BankingAngleRequest(
            velocity=25.0,
            radius=100.0,
            gravity=9.81,
        )
        result = calculate_banking_angle(request)

        # θ = arctan(v² / (rg))
        expected = math.atan((25.0 * 25.0) / (100.0 * 9.81))
        assert abs(result.angle_radians - expected) < 0.01
        assert abs(result.angle_degrees - math.degrees(expected)) < 0.1


class TestEscapeVelocity:
    def test_earth_escape(self):
        """Test escape velocity from Earth."""
        request = EscapeVelocityRequest(
            mass=5.972e24,  # Earth mass
            radius=6.371e6,  # Earth radius
        )
        result = calculate_escape_velocity(request)

        # Should be around 11,200 m/s
        assert 11000 < result.escape_velocity < 11500
        assert 39600 < result.escape_velocity_kmh < 41400


class TestCircularOrbit:
    def test_leo_satellite(self):
        """Test circular orbit analysis for LEO satellite."""
        request = CircularOrbitRequest(
            altitude=400000.0,  # 400 km
            planet_mass=5.972e24,  # Earth
            planet_radius=6.371e6,  # Earth
        )
        result = analyze_circular_orbit(request)

        # Orbital radius should be planet radius + altitude
        expected_radius = 6.371e6 + 400000.0
        assert abs(result.orbital_radius - expected_radius) < 1000

        # Orbital velocity should be around 7,670 m/s
        assert 7500 < result.orbital_velocity < 7800

        # Period should be around 90-95 minutes
        assert 5000 < result.period_seconds < 6000

        # Centripetal acceleration should be positive
        assert result.centripetal_acceleration > 0
