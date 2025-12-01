"""Tests for advanced fluid dynamics calculations."""

import math


from chuk_mcp_physics.fluid_advanced import (
    BernoulliRequest,
    LiftForceRequest,
    MagnusForceRequest,
    PressureAtDepthRequest,
    ReynoldsNumberRequest,
    VenturiEffectRequest,
    calculate_bernoulli,
    calculate_lift_force,
    calculate_magnus_force,
    calculate_pressure_at_depth,
    calculate_reynolds_number,
    calculate_venturi_effect,
)


class TestLiftForce:
    def test_basic_lift(self):
        """Test basic lift force calculation."""
        request = LiftForceRequest(
            velocity=50.0,  # m/s
            wing_area=20.0,  # m²
            lift_coefficient=1.2,
            fluid_density=1.225,  # air
        )
        result = calculate_lift_force(request)

        # L = 0.5 * 1.225 * 50² * 1.2 * 20 = 36750 N
        assert result.lift_force > 36000
        assert result.lift_force < 37000
        assert result.dynamic_pressure > 0

    def test_low_velocity(self):
        """Test lift at low velocity."""
        request = LiftForceRequest(
            velocity=1.0,  # Very slow
            wing_area=20.0,
            lift_coefficient=1.2,
            fluid_density=1.225,
        )
        result = calculate_lift_force(request)

        # Low velocity = low lift
        assert result.lift_force < 50.0
        assert result.dynamic_pressure < 1.0

    def test_aircraft_wing(self):
        """Test realistic aircraft wing lift."""
        request = LiftForceRequest(
            velocity=70.0,  # ~250 km/h
            wing_area=15.0,
            lift_coefficient=1.5,
            fluid_density=1.225,
        )
        result = calculate_lift_force(request)

        assert result.lift_force > 50000  # Should generate significant lift
        assert result.dynamic_pressure > 3000

    def test_water_vs_air(self):
        """Test lift in water vs air."""
        # Water is ~800x denser than air
        air_request = LiftForceRequest(
            velocity=10.0,
            wing_area=1.0,
            lift_coefficient=1.0,
            fluid_density=1.225,
        )
        water_request = LiftForceRequest(
            velocity=10.0,
            wing_area=1.0,
            lift_coefficient=1.0,
            fluid_density=1000.0,
        )

        air_result = calculate_lift_force(air_request)
        water_result = calculate_lift_force(water_request)

        # Water lift should be much higher
        assert water_result.lift_force > 800 * air_result.lift_force


class TestMagnusForce:
    def test_spinning_ball(self):
        """Test Magnus force on spinning ball."""
        request = MagnusForceRequest(
            velocity=[20.0, 0.0, 0.0],  # Moving in +X
            angular_velocity=[0.0, 0.0, 50.0],  # Spinning around Z
            radius=0.11,  # Soccer ball
            fluid_density=1.225,
        )
        result = calculate_magnus_force(request)

        assert len(result.magnus_force) == 3
        assert result.magnus_force_magnitude > 0
        assert result.spin_rate == 50.0

    def test_no_spin(self):
        """Test no Magnus force without spin."""
        request = MagnusForceRequest(
            velocity=[20.0, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 0.0],  # No spin
            radius=0.11,
            fluid_density=1.225,
        )
        result = calculate_magnus_force(request)

        assert result.magnus_force_magnitude == 0.0
        assert result.spin_rate == 0.0

    def test_perpendicular_force(self):
        """Test Magnus force is perpendicular to velocity and spin."""
        request = MagnusForceRequest(
            velocity=[10.0, 0.0, 0.0],  # +X
            angular_velocity=[0.0, 0.0, 100.0],  # Spin around Z
            radius=0.1,
            fluid_density=1.225,
        )
        result = calculate_magnus_force(request)

        # Force should be in Y direction (perpendicular to both X and Z)
        assert abs(result.magnus_force[0]) < 0.1  # No X component
        assert abs(result.magnus_force[1]) > 0  # Has Y component
        assert abs(result.magnus_force[2]) < 0.1  # No Z component


class TestBernoulli:
    def test_energy_conservation(self):
        """Test Bernoulli energy conservation."""
        request = BernoulliRequest(
            pressure1=101325.0,  # 1 atm
            velocity1=0.0,  # Still water
            height1=10.0,  # 10m height
            velocity2=14.0,  # Exit velocity
            height2=0.0,  # Ground level
            fluid_density=1000.0,
            gravity=9.81,
        )
        result = calculate_bernoulli(request)

        # Total pressure should be conserved
        assert result.total_pressure_1 > 0
        assert result.pressure2 is not None

    def test_static_case(self):
        """Test static fluid (no velocity)."""
        request = BernoulliRequest(
            pressure1=101325.0,
            velocity1=0.0,
            height1=0.0,
            fluid_density=1000.0,
        )
        result = calculate_bernoulli(request)

        assert result.dynamic_pressure_1 == 0.0
        assert result.static_pressure_1 == 101325.0

    def test_flowing_water(self):
        """Test flowing water with velocity change."""
        request = BernoulliRequest(
            pressure1=200000.0,
            velocity1=5.0,
            height1=0.0,
            velocity2=10.0,
            height2=0.0,
            fluid_density=1000.0,
        )
        result = calculate_bernoulli(request)

        # When velocity increases, pressure should decrease
        assert result.pressure2 is not None
        assert result.pressure2 < 200000.0


class TestPressureAtDepth:
    def test_shallow_water(self):
        """Test pressure at shallow depth."""
        request = PressureAtDepthRequest(
            depth=10.0,  # 10 meters
            fluid_density=1000.0,
            atmospheric_pressure=101325.0,
        )
        result = calculate_pressure_at_depth(request)

        # P = P_atm + ρgh = 101325 + 1000*9.81*10 ≈ 199425 Pa
        assert abs(result.total_pressure - 199425.0) < 1000
        assert abs(result.gauge_pressure - 98100.0) < 1000
        assert abs(result.pressure_atmospheres - 1.97) < 0.1

    def test_scuba_diving(self):
        """Test pressure at scuba diving depth."""
        request = PressureAtDepthRequest(
            depth=30.0,  # 30 meters
            fluid_density=1025.0,  # Seawater
            atmospheric_pressure=101325.0,
        )
        result = calculate_pressure_at_depth(request)

        # Should be ~4 atmospheres
        assert result.pressure_atmospheres > 3.5
        assert result.pressure_atmospheres < 4.5

    def test_surface(self):
        """Test pressure at surface."""
        request = PressureAtDepthRequest(
            depth=0.0,
            fluid_density=1000.0,
            atmospheric_pressure=101325.0,
        )
        result = calculate_pressure_at_depth(request)

        assert result.total_pressure == 101325.0
        assert result.gauge_pressure == 0.0
        assert result.pressure_atmospheres == 1.0

    def test_deep_ocean(self):
        """Test pressure in deep ocean."""
        request = PressureAtDepthRequest(
            depth=1000.0,  # 1 km deep
            fluid_density=1025.0,
            atmospheric_pressure=101325.0,
        )
        result = calculate_pressure_at_depth(request)

        # Should be ~100 atmospheres
        assert result.pressure_atmospheres > 95
        assert result.pressure_atmospheres < 105


class TestReynoldsNumber:
    def test_laminar_flow(self):
        """Test laminar flow regime."""
        request = ReynoldsNumberRequest(
            velocity=0.1,
            characteristic_length=0.01,  # 1 cm
            fluid_density=1000.0,
            dynamic_viscosity=0.001,  # Water
        )
        result = calculate_reynolds_number(request)

        # Re = 1000 * 0.1 * 0.01 / 0.001 = 1000
        assert abs(result.reynolds_number - 1000.0) < 10
        assert result.flow_regime == "laminar"

    def test_transitional_flow(self):
        """Test transitional flow regime."""
        request = ReynoldsNumberRequest(
            velocity=0.3,  # Adjusted for transitional regime
            characteristic_length=0.01,
            fluid_density=1000.0,
            dynamic_viscosity=0.001,
        )
        result = calculate_reynolds_number(request)

        # Re = 3000 (transitional: 2300 < Re < 4000)
        assert result.reynolds_number > 2300
        assert result.reynolds_number < 4000
        assert result.flow_regime == "transitional"

    def test_turbulent_flow(self):
        """Test turbulent flow regime."""
        request = ReynoldsNumberRequest(
            velocity=2.0,
            characteristic_length=0.05,  # 5 cm pipe
            fluid_density=1000.0,
            dynamic_viscosity=0.001,
        )
        result = calculate_reynolds_number(request)

        # Re = 1000 * 2.0 * 0.05 / 0.001 = 100000
        assert abs(result.reynolds_number - 100000.0) < 1000
        assert result.flow_regime == "turbulent"

    def test_air_flow(self):
        """Test air flow Reynolds number."""
        request = ReynoldsNumberRequest(
            velocity=10.0,
            characteristic_length=0.1,
            fluid_density=1.225,  # Air
            dynamic_viscosity=1.8e-5,  # Air viscosity
        )
        result = calculate_reynolds_number(request)

        assert result.reynolds_number > 60000
        assert result.flow_regime == "turbulent"


class TestVenturiEffect:
    def test_basic_venturi(self):
        """Test basic Venturi effect."""
        request = VenturiEffectRequest(
            inlet_diameter=0.1,  # 10 cm
            throat_diameter=0.05,  # 5 cm
            inlet_velocity=2.0,
            fluid_density=1000.0,
        )
        result = calculate_venturi_effect(request)

        # Continuity: v2 = v1 * (A1/A2) = 2 * (10/5)² = 8 m/s
        assert abs(result.throat_velocity - 8.0) < 0.1
        assert result.pressure_drop > 0
        assert result.flow_rate > 0

    def test_flow_rate(self):
        """Test volumetric flow rate."""
        request = VenturiEffectRequest(
            inlet_diameter=0.1,
            throat_diameter=0.05,
            inlet_velocity=2.0,
            fluid_density=1000.0,
        )
        result = calculate_venturi_effect(request)

        # Q = A1 * v1 = π * 0.05² * 2 ≈ 0.0157 m³/s
        expected_flow = math.pi * (0.05**2) * 2.0
        assert abs(result.flow_rate - expected_flow) < 0.001

    def test_no_constriction(self):
        """Test with no constriction (same diameter)."""
        request = VenturiEffectRequest(
            inlet_diameter=0.1,
            throat_diameter=0.1,  # Same as inlet
            inlet_velocity=2.0,
            fluid_density=1000.0,
        )
        result = calculate_venturi_effect(request)

        # No constriction = no velocity change = no pressure drop
        assert abs(result.throat_velocity - 2.0) < 0.01
        assert abs(result.pressure_drop) < 1.0

    def test_large_constriction(self):
        """Test with large constriction."""
        request = VenturiEffectRequest(
            inlet_diameter=0.2,
            throat_diameter=0.05,  # 4x smaller diameter
            inlet_velocity=1.0,
            fluid_density=1000.0,
        )
        result = calculate_venturi_effect(request)

        # Area ratio = 16, so velocity should increase 16x
        assert result.throat_velocity > 15.0
        assert result.throat_velocity < 17.0
        # Large pressure drop
        assert result.pressure_drop > 100000
