"""Tests for Phase 2 server tools."""

import pytest

from chuk_mcp_physics.tools import (
    rotational,
    oscillations,
    circular_motion,
    conservation,
    statics,
    collisions,
)


# For backwards compatibility, create a namespace object
class _ToolNamespace:
    def __getattr__(self, name):
        # Try each tool module
        for module in [
            rotational,
            oscillations,
            circular_motion,
            conservation,
            statics,
            collisions,
        ]:
            if hasattr(module, name):
                return getattr(module, name)
        raise AttributeError(f"Tool '{name}' not found")


server = _ToolNamespace()


class TestRotationalTools:
    @pytest.mark.asyncio
    async def test_calculate_torque(self):
        """Test torque calculation tool."""
        result = await server.calculate_torque(
            force_x=50.0,
            force_y=0.0,
            force_z=0.0,
            position_x=0.0,
            position_y=0.0,
            position_z=0.8,
        )

        assert "torque" in result
        assert "magnitude" in result
        assert abs(result["magnitude"] - 40.0) < 0.1

    @pytest.mark.asyncio
    async def test_calculate_moment_of_inertia(self):
        """Test moment of inertia calculation tool."""
        result = await server.calculate_moment_of_inertia(
            shape="disk",
            mass=5.0,
            radius=0.3,
        )

        assert "moment_of_inertia" in result
        expected = 0.5 * 5.0 * 0.3 * 0.3
        assert abs(result["moment_of_inertia"] - expected) < 0.001

    @pytest.mark.asyncio
    async def test_calculate_angular_momentum(self):
        """Test angular momentum calculation tool."""
        result = await server.calculate_angular_momentum(
            moment_of_inertia=3.0,
            angular_velocity_x=0.0,
            angular_velocity_y=5.0,
            angular_velocity_z=0.0,
        )

        assert "angular_momentum" in result
        assert "magnitude" in result
        assert abs(result["magnitude"] - 15.0) < 0.1

    @pytest.mark.asyncio
    async def test_calculate_rotational_kinetic_energy(self):
        """Test rotational KE calculation tool."""
        result = await server.calculate_rotational_kinetic_energy(
            moment_of_inertia=0.5,
            angular_velocity=100.0,
        )

        assert "rotational_ke" in result
        expected = 0.5 * 0.5 * 100.0 * 100.0
        assert abs(result["rotational_ke"] - expected) < 1.0

    @pytest.mark.asyncio
    async def test_calculate_angular_acceleration(self):
        """Test angular acceleration calculation tool."""
        result = await server.calculate_angular_acceleration(
            torque=10.0,
            moment_of_inertia=0.5,
        )

        assert "angular_acceleration" in result
        assert abs(result["angular_acceleration"] - 20.0) < 0.1


class TestOscillationTools:
    @pytest.mark.asyncio
    async def test_calculate_hookes_law(self):
        """Test Hooke's law calculation tool."""
        result = await server.calculate_hookes_law(
            spring_constant=10000.0,
            displacement=0.05,
        )

        assert "force" in result
        assert "potential_energy" in result
        assert abs(result["force"] - 500.0) < 0.1

    @pytest.mark.asyncio
    async def test_calculate_spring_mass_period(self):
        """Test spring-mass period calculation tool."""
        result = await server.calculate_spring_mass_period(
            mass=0.5,
            spring_constant=20.0,
        )

        assert "period" in result
        assert "frequency" in result
        assert result["period"] > 0

    @pytest.mark.asyncio
    async def test_calculate_simple_harmonic_motion(self):
        """Test SHM calculation tool."""
        result = await server.calculate_simple_harmonic_motion(
            amplitude=0.1,
            angular_frequency=5.0,
            time=0.0,
            phase=0.0,
        )

        assert "position" in result
        assert "velocity" in result
        assert "acceleration" in result
        assert abs(result["position"] - 0.1) < 0.01

    @pytest.mark.asyncio
    async def test_calculate_damped_oscillation(self):
        """Test damped oscillation calculation tool."""
        result = await server.calculate_damped_oscillation(
            mass=1.0,
            spring_constant=100.0,
            damping_coefficient=2.0,
            time=0.0,
        )

        assert "position" in result
        assert "velocity" in result
        assert "damping_ratio" in result
        assert "regime" in result
        assert result["regime"] == "underdamped"

    @pytest.mark.asyncio
    async def test_calculate_pendulum_period(self):
        """Test pendulum period calculation tool."""
        result = await server.calculate_pendulum_period(
            length=0.994,
        )

        assert "period" in result
        assert abs(result["period"] - 2.0) < 0.01


class TestCircularMotionTools:
    @pytest.mark.asyncio
    async def test_calculate_centripetal_force(self):
        """Test centripetal force calculation tool."""
        result = await server.calculate_centripetal_force(
            mass=1500.0,
            velocity=20.0,
            radius=50.0,
        )

        assert "centripetal_force" in result
        assert "centripetal_acceleration" in result
        expected = 1500.0 * 20.0 * 20.0 / 50.0
        assert abs(result["centripetal_force"] - expected) < 1.0

    @pytest.mark.asyncio
    async def test_calculate_orbital_period(self):
        """Test orbital period calculation tool."""
        result = await server.calculate_orbital_period(
            orbital_radius=6.771e6,
            central_mass=5.972e24,
        )

        assert "period" in result
        assert "orbital_velocity" in result
        assert 5000 < result["period"] < 6000

    @pytest.mark.asyncio
    async def test_calculate_banking_angle(self):
        """Test banking angle calculation tool."""
        result = await server.calculate_banking_angle(
            velocity=25.0,
            radius=100.0,
        )

        assert "angle_radians" in result
        assert "angle_degrees" in result
        assert result["angle_degrees"] > 0

    @pytest.mark.asyncio
    async def test_calculate_escape_velocity(self):
        """Test escape velocity calculation tool."""
        result = await server.calculate_escape_velocity(
            mass=5.972e24,
            radius=6.371e6,
        )

        assert "escape_velocity" in result
        assert 11000 < result["escape_velocity"] < 11500

    @pytest.mark.asyncio
    async def test_analyze_circular_orbit(self):
        """Test circular orbit analysis tool."""
        result = await server.analyze_circular_orbit(
            altitude=400000.0,
            planet_mass=5.972e24,
            planet_radius=6.371e6,
        )

        assert "orbital_radius" in result
        assert "orbital_velocity" in result
        assert "period_seconds" in result
        assert result["period_seconds"] > 0


class TestCollisionTools:
    @pytest.mark.asyncio
    async def test_calculate_inelastic_collision_3d(self):
        """Test 3D inelastic collision calculation tool."""
        result = await server.calculate_inelastic_collision_3d(
            mass1=1500.0,
            velocity1=[20.0, 0.0, 0.0],
            mass2=1200.0,
            velocity2=[-15.0, 0.0, 0.0],
            coefficient_of_restitution=0.0,
        )

        assert "final_velocity1" in result
        assert "final_velocity2" in result
        assert "energy_loss" in result
        assert result["energy_loss"] > 0

    @pytest.mark.asyncio
    async def test_calculate_elastic_collision_3d(self):
        """Test 3D elastic collision calculation tool."""
        result = await server.calculate_elastic_collision_3d(
            mass1=0.17,
            velocity1=[2.0, 0.0, 0.0],
            mass2=0.17,
            velocity2=[0.0, 0.0, 0.0],
        )

        assert "final_velocity1" in result
        assert "final_velocity2" in result
        assert "initial_kinetic_energy" in result
        assert "final_kinetic_energy" in result

    @pytest.mark.asyncio
    async def test_collision_with_string_velocities(self):
        """Test collision with JSON string velocities."""
        result = await server.calculate_elastic_collision_3d(
            mass1=0.17,
            velocity1="[2.0, 0.0, 0.0]",
            mass2=0.17,
            velocity2="[0.0, 0.0, 0.0]",
        )

        assert "final_velocity1" in result
        assert "final_velocity2" in result


class TestConservationTools:
    @pytest.mark.asyncio
    async def test_check_energy_conservation(self):
        """Test energy conservation check tool."""
        result = await server.check_energy_conservation(
            initial_kinetic_energy=100.0,
            final_kinetic_energy=50.0,
            initial_potential_energy=0.0,
            final_potential_energy=50.0,
        )

        assert "is_conserved" in result
        assert "energy_difference" in result
        assert result["is_conserved"] is True

    @pytest.mark.asyncio
    async def test_check_momentum_conservation(self):
        """Test momentum conservation check tool."""
        result = await server.check_momentum_conservation(
            initial_momentum=[3000.0, 0.0, 0.0],
            final_momentum=[3000.0, 0.0, 0.0],
        )

        assert "is_conserved" in result
        assert "momentum_difference_magnitude" in result
        assert result["is_conserved"] is True

    @pytest.mark.asyncio
    async def test_check_angular_momentum_conservation(self):
        """Test angular momentum conservation check tool."""
        result = await server.check_angular_momentum_conservation(
            initial_angular_momentum=[0.0, 15.0, 0.0],
            final_angular_momentum=[0.0, 15.0, 0.0],
        )

        assert "is_conserved" in result
        assert result["is_conserved"] is True

    @pytest.mark.asyncio
    async def test_conservation_with_string_input(self):
        """Test conservation check with JSON string input."""
        result = await server.check_momentum_conservation(
            initial_momentum="[3000.0, 0.0, 0.0]",
            final_momentum="[3000.0, 0.0, 0.0]",
        )

        assert "is_conserved" in result
        assert result["is_conserved"] is True

    @pytest.mark.asyncio
    async def test_track_energy_dissipation(self):
        """Test energy dissipation tracking tool."""
        trajectory_data = {
            "frames": [
                {
                    "time": 0.0,
                    "position": [0.0, 10.0, 0.0],
                    "rotation": [0.0, 0.0, 0.0, 1.0],
                    "velocity": [0.0, 0.0, 0.0],
                },
                {
                    "time": 1.0,
                    "position": [0.0, 0.0, 0.0],
                    "rotation": [0.0, 0.0, 0.0, 1.0],
                    "velocity": [0.0, 0.0, 0.0],
                },
            ]
        }

        result = await server.track_energy_dissipation(
            trajectory_data=trajectory_data,
            mass=1.0,
        )

        assert "frames" in result
        assert "total_energy_loss" in result
        assert "average_power_dissipated" in result
        assert len(result["frames"]) == 2
