"""Tests for collision calculations."""

from chuk_mcp_physics.collisions import (
    InelasticCollision3DRequest,
    ElasticCollision3DRequest,
    calculate_inelastic_collision_3d,
    calculate_elastic_collision_3d,
)


class TestInelasticCollision3D:
    def test_perfectly_inelastic(self):
        """Test perfectly inelastic collision (e=0)."""
        request = InelasticCollision3DRequest(
            mass1=1500.0,
            velocity1=[20.0, 0.0, 0.0],
            mass2=1200.0,
            velocity2=[-15.0, 0.0, 0.0],
            coefficient_of_restitution=0.0,
        )
        result = calculate_inelastic_collision_3d(request)

        # Momentum should be conserved
        p_initial = 1500.0 * 20.0 + 1200.0 * (-15.0)
        p_final = 1500.0 * result.final_velocity1[0] + 1200.0 * result.final_velocity2[0]
        assert abs(p_initial - p_final) < 1.0

        # Significant energy loss
        assert result.energy_loss > 0
        assert result.energy_loss_percent > 50

    def test_elastic_collision_via_cor(self):
        """Test elastic collision using e=1.0."""
        request = InelasticCollision3DRequest(
            mass1=0.17,
            velocity1=[2.0, 0.0, 0.0],
            mass2=0.17,
            velocity2=[0.0, 0.0, 0.0],
            coefficient_of_restitution=1.0,
        )
        result = calculate_inelastic_collision_3d(request)

        # Energy should be conserved
        assert abs(result.initial_kinetic_energy - result.final_kinetic_energy) < 0.01
        assert result.energy_loss_percent < 1.0


class TestElasticCollision3D:
    def test_equal_mass_head_on(self):
        """Test elastic collision with equal masses, head-on."""
        request = ElasticCollision3DRequest(
            mass1=0.17,
            velocity1=[2.0, 0.0, 0.0],
            mass2=0.17,
            velocity2=[0.0, 0.0, 0.0],
        )
        result = calculate_elastic_collision_3d(request)

        # With equal masses, velocities should exchange
        assert abs(result.final_velocity1[0]) < 0.1  # Ball 1 stops
        assert abs(result.final_velocity2[0] - 2.0) < 0.1  # Ball 2 moves at 2 m/s

        # Energy and momentum conserved
        assert abs(result.initial_kinetic_energy - result.final_kinetic_energy) < 0.01

    def test_momentum_conservation(self):
        """Test momentum conservation in elastic collision."""
        request = ElasticCollision3DRequest(
            mass1=2.0,
            velocity1=[5.0, 0.0, 0.0],
            mass2=3.0,
            velocity2=[-2.0, 0.0, 0.0],
        )
        result = calculate_elastic_collision_3d(request)

        # Check momentum conservation
        p_initial_x = 2.0 * 5.0 + 3.0 * (-2.0)
        p_final_x = 2.0 * result.final_velocity1[0] + 3.0 * result.final_velocity2[0]
        assert abs(p_initial_x - p_final_x) < 0.01
