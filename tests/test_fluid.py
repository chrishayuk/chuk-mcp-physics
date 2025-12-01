"""Tests for fluid dynamics calculations."""

import math
import pytest

from chuk_mcp_physics.fluid import (
    calculate_drag_force,
    calculate_buoyancy,
    calculate_terminal_velocity,
    simulate_underwater_motion,
)
from chuk_mcp_physics.models import (
    DragForceRequest,
    BuoyancyRequest,
    TerminalVelocityRequest,
    UnderwaterMotionRequest,
    FluidEnvironment,
)


class TestDragForce:
    """Test drag force calculations."""

    def test_drag_force_basic(self):
        """Test basic drag force calculation."""
        # 10cm sphere falling at 5 m/s through water
        radius = 0.05
        area = math.pi * radius * radius

        request = DragForceRequest(
            velocity=[0.0, -5.0, 0.0],
            cross_sectional_area=area,
            fluid_density=1000.0,  # water
            drag_coefficient=0.47,  # sphere
        )

        response = calculate_drag_force(request)

        # Drag should oppose motion (upward)
        assert response.drag_force[0] == pytest.approx(0.0, abs=0.01)
        assert response.drag_force[1] > 0  # Upward
        assert response.drag_force[2] == pytest.approx(0.0, abs=0.01)

        # Check magnitude: F = 0.5 * ρ * v² * Cd * A
        expected_magnitude = 0.5 * 1000.0 * 25.0 * 0.47 * area
        assert response.magnitude == pytest.approx(expected_magnitude, rel=0.01)

        # Reynolds number should be high (turbulent)
        assert response.reynolds_number > 1000

    def test_drag_force_zero_velocity(self):
        """Test drag force with zero velocity."""
        request = DragForceRequest(
            velocity=[0.0, 0.0, 0.0],
            cross_sectional_area=0.01,
            fluid_density=1000.0,
            drag_coefficient=0.47,
        )

        response = calculate_drag_force(request)

        assert response.drag_force == [0.0, 0.0, 0.0]
        assert response.magnitude == 0.0
        assert response.reynolds_number == 0.0

    def test_drag_force_horizontal_motion(self):
        """Test drag force for horizontal motion."""
        request = DragForceRequest(
            velocity=[10.0, 0.0, 0.0],  # Moving right
            cross_sectional_area=0.1,
            fluid_density=1.225,  # air
            drag_coefficient=0.47,
        )

        response = calculate_drag_force(request)

        # Drag should oppose motion (leftward)
        assert response.drag_force[0] < 0
        assert response.drag_force[1] == pytest.approx(0.0, abs=0.01)
        assert response.drag_force[2] == pytest.approx(0.0, abs=0.01)

    def test_drag_force_air_vs_water(self):
        """Test that drag is much higher in water than air."""
        # Same conditions, different fluids
        velocity = [0.0, -5.0, 0.0]
        area = 0.01
        cd = 0.47

        air_request = DragForceRequest(
            velocity=velocity,
            cross_sectional_area=area,
            fluid_density=1.225,  # air
            drag_coefficient=cd,
        )

        water_request = DragForceRequest(
            velocity=velocity,
            cross_sectional_area=area,
            fluid_density=1000.0,  # water
            drag_coefficient=cd,
        )

        air_response = calculate_drag_force(air_request)
        water_response = calculate_drag_force(water_request)

        # Water drag should be ~800x higher (density ratio)
        ratio = water_response.magnitude / air_response.magnitude
        assert ratio == pytest.approx(1000.0 / 1.225, rel=0.01)


class TestBuoyancy:
    """Test buoyancy calculations."""

    def test_buoyancy_fully_submerged(self):
        """Test buoyancy for fully submerged object."""
        # 10cm diameter sphere
        radius = 0.05
        volume = (4.0 / 3.0) * math.pi * radius**3

        request = BuoyancyRequest(
            volume=volume,
            fluid_density=1000.0,  # water
            gravity=9.81,
            submerged_fraction=1.0,
        )

        response = calculate_buoyancy(request)

        # F_b = ρ * V * g
        expected_force = 1000.0 * volume * 9.81
        assert response.buoyant_force == pytest.approx(expected_force, rel=0.01)

        # Displaced mass should equal fluid density * volume
        assert response.displaced_mass == pytest.approx(1000.0 * volume, rel=0.01)

    def test_buoyancy_partially_submerged(self):
        """Test buoyancy for partially submerged object."""
        volume = 1.0  # 1 m³

        request = BuoyancyRequest(
            volume=volume,
            fluid_density=1000.0,
            gravity=9.81,
            submerged_fraction=0.5,  # Half submerged
        )

        response = calculate_buoyancy(request)

        # Force should be half of fully submerged
        full_force = 1000.0 * volume * 9.81
        assert response.buoyant_force == pytest.approx(full_force * 0.5, rel=0.01)
        assert response.displaced_mass == pytest.approx(500.0, rel=0.01)

    def test_buoyancy_different_fluids(self):
        """Test buoyancy in different fluids."""
        volume = 0.001  # 1 liter

        # Water
        water_request = BuoyancyRequest(volume=volume, fluid_density=1000.0, gravity=9.81)
        water_response = calculate_buoyancy(water_request)

        # Air
        air_request = BuoyancyRequest(volume=volume, fluid_density=1.225, gravity=9.81)
        air_response = calculate_buoyancy(air_request)

        # Water buoyancy should be much higher
        ratio = water_response.buoyant_force / air_response.buoyant_force
        assert ratio == pytest.approx(1000.0 / 1.225, rel=0.01)


class TestTerminalVelocity:
    """Test terminal velocity calculations."""

    def test_terminal_velocity_basic(self):
        """Test basic terminal velocity calculation."""
        # Skydiver
        request = TerminalVelocityRequest(
            mass=70.0,
            cross_sectional_area=0.7,
            fluid_density=1.225,  # air
            drag_coefficient=1.0,
            gravity=9.81,
        )

        response = calculate_terminal_velocity(request)

        # v_t = sqrt(2mg / ρCdA)
        expected_vt = math.sqrt((2.0 * 70.0 * 9.81) / (1.225 * 1.0 * 0.7))
        assert response.terminal_velocity == pytest.approx(expected_vt, rel=0.01)

        # Drag force at terminal should equal weight
        assert response.drag_force_at_terminal == pytest.approx(70.0 * 9.81, rel=0.01)

        # Time to 95% should be positive
        assert response.time_to_95_percent > 0

    def test_terminal_velocity_water_vs_air(self):
        """Test that terminal velocity is lower in water."""
        mass = 1.0
        area = 0.01
        cd = 0.47

        air_request = TerminalVelocityRequest(
            mass=mass,
            cross_sectional_area=area,
            fluid_density=1.225,
            drag_coefficient=cd,
        )

        water_request = TerminalVelocityRequest(
            mass=mass,
            cross_sectional_area=area,
            fluid_density=1000.0,
            drag_coefficient=cd,
        )

        air_response = calculate_terminal_velocity(air_request)
        water_response = calculate_terminal_velocity(water_request)

        # Terminal velocity in water should be much lower
        assert water_response.terminal_velocity < air_response.terminal_velocity

        # Ratio should be sqrt(ρ_air / ρ_water)
        ratio = air_response.terminal_velocity / water_response.terminal_velocity
        expected_ratio = math.sqrt(1000.0 / 1.225)
        assert ratio == pytest.approx(expected_ratio, rel=0.01)

    def test_terminal_velocity_streamlined_vs_bluff(self):
        """Test that streamlined shapes fall faster."""
        mass = 1.0
        area = 0.01
        fluid_density = 1.225

        # Streamlined (Cd = 0.04)
        streamlined = TerminalVelocityRequest(
            mass=mass,
            cross_sectional_area=area,
            fluid_density=fluid_density,
            drag_coefficient=0.04,
        )

        # Bluff body (Cd = 1.0)
        bluff = TerminalVelocityRequest(
            mass=mass,
            cross_sectional_area=area,
            fluid_density=fluid_density,
            drag_coefficient=1.0,
        )

        streamlined_response = calculate_terminal_velocity(streamlined)
        bluff_response = calculate_terminal_velocity(bluff)

        # Streamlined should fall faster
        assert streamlined_response.terminal_velocity > bluff_response.terminal_velocity

        # Ratio should be sqrt(Cd_bluff / Cd_streamlined)
        ratio = streamlined_response.terminal_velocity / bluff_response.terminal_velocity
        expected_ratio = math.sqrt(1.0 / 0.04)
        assert ratio == pytest.approx(expected_ratio, rel=0.01)


class TestUnderwaterMotion:
    """Test underwater motion simulation."""

    def test_underwater_motion_basic(self):
        """Test basic underwater motion simulation."""
        fluid = FluidEnvironment(density=1000.0, viscosity=1.002e-3, name="water")

        request = UnderwaterMotionRequest(
            initial_position=[0.0, 0.0, 0.0],
            initial_velocity=[10.0, 0.0, 0.0],  # 10 m/s forward
            mass=10.0,
            volume=0.01,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            fluid=fluid,
            gravity=9.81,
            duration=5.0,
            dt=0.01,
        )

        response = simulate_underwater_motion(request)

        # Check basic properties
        assert len(response.trajectory) > 0
        assert len(response.final_position) == 3
        assert len(response.final_velocity) == 3

        # Object should have moved forward
        assert response.final_position[0] > 0

        # Velocity should have decreased due to drag
        final_speed = math.sqrt(sum(v**2 for v in response.final_velocity))
        assert final_speed < 10.0

        # Total distance should be positive
        assert response.total_distance > 0

    def test_underwater_motion_sinking(self):
        """Test that dense object sinks."""
        fluid = FluidEnvironment(density=1000.0, viscosity=1.002e-3)

        # Heavy object (denser than water)
        request = UnderwaterMotionRequest(
            initial_position=[0.0, 0.0, 0.0],
            initial_velocity=[0.0, 0.0, 0.0],
            mass=100.0,  # 100 kg
            volume=0.01,  # 0.01 m³ -> density = 10,000 kg/m³ >> water
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            fluid=fluid,
            gravity=9.81,
            duration=2.0,
            dt=0.01,
        )

        response = simulate_underwater_motion(request)

        # Object should sink (negative y)
        assert response.final_position[1] < 0
        assert response.max_depth < 0

    def test_underwater_motion_rising(self):
        """Test that buoyant object rises."""
        fluid = FluidEnvironment(density=1000.0, viscosity=1.002e-3)

        # Light object (less dense than water)
        request = UnderwaterMotionRequest(
            initial_position=[0.0, -5.0, 0.0],  # Start underwater
            initial_velocity=[0.0, 0.0, 0.0],
            mass=1.0,  # 1 kg
            volume=0.01,  # 0.01 m³ -> density = 100 kg/m³ << water
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            fluid=fluid,
            gravity=9.81,
            duration=3.0,
            dt=0.01,
        )

        response = simulate_underwater_motion(request)

        # Object should rise (positive y velocity)
        assert response.final_position[1] > -5.0  # Higher than start

    def test_underwater_motion_settling(self):
        """Test that object eventually settles."""
        fluid = FluidEnvironment(density=1000.0, viscosity=1.002e-3)

        # Neutrally buoyant with high drag
        request = UnderwaterMotionRequest(
            initial_position=[0.0, 0.0, 0.0],
            initial_velocity=[1.0, 0.0, 0.0],  # Small initial velocity
            mass=10.0,
            volume=0.01,  # Same density as water
            cross_sectional_area=0.1,  # Large area = high drag
            drag_coefficient=2.0,  # High drag coefficient
            fluid=fluid,
            gravity=9.81,
            duration=20.0,  # Long duration
            dt=0.01,
        )

        response = simulate_underwater_motion(request)

        # Check if settled (very low velocity)
        final_speed = math.sqrt(sum(v**2 for v in response.final_velocity))
        # With high drag and long duration, should settle
        assert final_speed < 0.1  # Less than 10 cm/s

    def test_underwater_motion_trajectory_format(self):
        """Test trajectory output format."""
        fluid = FluidEnvironment(density=1000.0, viscosity=1.002e-3)

        request = UnderwaterMotionRequest(
            initial_position=[0.0, 0.0, 0.0],
            initial_velocity=[5.0, 0.0, 0.0],
            mass=10.0,
            volume=0.01,
            cross_sectional_area=0.01,
            drag_coefficient=0.47,
            fluid=fluid,
            duration=1.0,
            dt=0.1,  # Larger dt for fewer points
        )

        response = simulate_underwater_motion(request)

        # Each trajectory point should be [t, x, y, z]
        for point in response.trajectory:
            assert len(point) == 4
            assert point[0] >= 0  # Time should be non-negative


class TestFluidEnvironment:
    """Test fluid environment presets."""

    def test_water_preset(self):
        """Test water environment preset."""
        water = FluidEnvironment.water()

        assert water.density == pytest.approx(998.2, rel=0.01)
        assert water.viscosity == pytest.approx(1.002e-3, rel=0.01)
        assert "water" in water.name

    def test_air_preset(self):
        """Test air environment preset."""
        air = FluidEnvironment.air()

        assert air.density == pytest.approx(1.204, rel=0.01)
        assert air.viscosity == pytest.approx(1.825e-5, rel=0.01)
        assert "air" in air.name

    def test_oil_preset(self):
        """Test oil environment preset."""
        oil = FluidEnvironment.oil()

        assert oil.density == pytest.approx(900.0, rel=0.01)
        assert oil.viscosity == pytest.approx(0.1, rel=0.01)
        assert oil.name == "motor_oil"
