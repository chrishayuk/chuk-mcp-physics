"""Tests for conservation law verification."""

from chuk_mcp_physics.conservation import (
    EnergyConservationCheckRequest,
    MomentumConservationCheckRequest,
    AngularMomentumConservationCheckRequest,
    EnergyDissipationTrackingRequest,
    check_energy_conservation,
    check_momentum_conservation,
    check_angular_momentum_conservation,
    track_energy_dissipation,
)
from chuk_mcp_physics.models import TrajectoryFrame


class TestEnergyConservation:
    def test_perfect_conservation(self):
        """Test perfect energy conservation."""
        request = EnergyConservationCheckRequest(
            initial_kinetic_energy=100.0,
            final_kinetic_energy=50.0,
            initial_potential_energy=0.0,
            final_potential_energy=50.0,
            expected_energy_loss=0.0,
            tolerance=0.01,
        )
        result = check_energy_conservation(request)

        assert result.is_conserved is True
        assert abs(result.energy_difference) < 0.1
        assert result.initial_total_energy == 100.0
        assert result.final_total_energy == 100.0

    def test_energy_loss(self):
        """Test energy conservation with expected loss."""
        request = EnergyConservationCheckRequest(
            initial_kinetic_energy=0.0,
            final_kinetic_energy=0.0,
            initial_potential_energy=10.0,
            final_potential_energy=6.4,
            expected_energy_loss=3.6,
            tolerance=0.01,
        )
        result = check_energy_conservation(request)

        assert result.is_conserved is True
        assert abs(result.actual_loss - 3.6) < 0.1
        assert result.energy_difference_percent > 30.0

    def test_unexpected_energy_loss(self):
        """Test detection of unexpected energy loss."""
        request = EnergyConservationCheckRequest(
            initial_kinetic_energy=100.0,
            final_kinetic_energy=50.0,
            initial_potential_energy=0.0,
            final_potential_energy=0.0,
            expected_energy_loss=0.0,
            tolerance=0.01,
        )
        result = check_energy_conservation(request)

        assert result.is_conserved is False
        assert result.actual_loss == 50.0
        assert result.energy_difference_percent == 50.0


class TestMomentumConservation:
    def test_perfect_momentum_conservation(self):
        """Test perfect momentum conservation."""
        request = MomentumConservationCheckRequest(
            initial_momentum=[3000.0, 0.0, 0.0],
            final_momentum=[3000.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_momentum_conservation(request)

        assert result.is_conserved is True
        assert result.momentum_difference_magnitude < 0.1

    def test_slight_momentum_difference(self):
        """Test slight momentum difference within tolerance."""
        request = MomentumConservationCheckRequest(
            initial_momentum=[3000.0, 0.0, 0.0],
            final_momentum=[2995.0, 5.0, 0.0],
            tolerance=0.01,
        )
        result = check_momentum_conservation(request)

        assert result.is_conserved is True
        assert result.momentum_difference_percent < 1.0

    def test_momentum_violation(self):
        """Test detection of momentum conservation violation."""
        request = MomentumConservationCheckRequest(
            initial_momentum=[3000.0, 0.0, 0.0],
            final_momentum=[2500.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_momentum_conservation(request)

        assert result.is_conserved is False
        assert result.momentum_difference_magnitude > 100.0


class TestAngularMomentumConservation:
    def test_perfect_angular_momentum_conservation(self):
        """Test perfect angular momentum conservation."""
        request = AngularMomentumConservationCheckRequest(
            initial_angular_momentum=[0.0, 15.0, 0.0],
            final_angular_momentum=[0.0, 15.0, 0.0],
            tolerance=0.01,
        )
        result = check_angular_momentum_conservation(request)

        assert result.is_conserved is True
        assert result.L_difference_magnitude < 0.1

    def test_slight_L_difference(self):
        """Test slight angular momentum difference within tolerance."""
        request = AngularMomentumConservationCheckRequest(
            initial_angular_momentum=[0.0, 15.0, 0.0],
            final_angular_momentum=[0.0, 15.05, 0.0],
            tolerance=0.01,
        )
        result = check_angular_momentum_conservation(request)

        assert result.is_conserved is True

    def test_L_violation(self):
        """Test detection of angular momentum violation."""
        request = AngularMomentumConservationCheckRequest(
            initial_angular_momentum=[0.0, 15.0, 0.0],
            final_angular_momentum=[0.0, 10.0, 0.0],
            tolerance=0.01,
        )
        result = check_angular_momentum_conservation(request)

        assert result.is_conserved is False
        assert result.L_difference_magnitude > 1.0


class TestEnergyDissipationTracking:
    def test_basic_tracking(self):
        """Test energy dissipation tracking."""
        frames = [
            TrajectoryFrame(
                time=0.0,
                position=[0.0, 10.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            ),
            TrajectoryFrame(
                time=0.5,
                position=[0.0, 5.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, -10.0, 0.0],
            ),
            TrajectoryFrame(
                time=1.0,
                position=[0.0, 0.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            ),
        ]

        request = EnergyDissipationTrackingRequest(
            frames=frames,
            mass=1.0,
            gravity=9.81,
            reference_height=0.0,
        )
        result = track_energy_dissipation(request)

        assert len(result.frames) == 3
        assert result.initial_total_energy > 0
        assert result.frames[0].potential_energy > 0
        assert result.frames[0].kinetic_energy == 0.0

    def test_energy_loss_calculation(self):
        """Test energy loss calculation over trajectory."""
        frames = [
            TrajectoryFrame(
                time=0.0,
                position=[0.0, 10.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            ),
            TrajectoryFrame(
                time=1.0,
                position=[0.0, 8.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            ),
        ]

        request = EnergyDissipationTrackingRequest(
            frames=frames,
            mass=1.0,
            gravity=9.81,
            reference_height=0.0,
        )
        result = track_energy_dissipation(request)

        # Should have lost 2m worth of potential energy
        expected_loss = 1.0 * 9.81 * 2.0
        assert abs(result.total_energy_loss - expected_loss) < 0.1
        assert result.total_energy_loss_percent > 0

    def test_power_dissipation(self):
        """Test average power dissipation calculation."""
        frames = [
            TrajectoryFrame(
                time=0.0,
                position=[0.0, 10.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            ),
            TrajectoryFrame(
                time=2.0,
                position=[0.0, 0.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            ),
        ]

        request = EnergyDissipationTrackingRequest(
            frames=frames,
            mass=1.0,
            gravity=9.81,
            reference_height=0.0,
        )
        result = track_energy_dissipation(request)

        # Power = Energy / Time
        expected_power = (1.0 * 9.81 * 10.0) / 2.0
        assert abs(result.average_power_dissipated - expected_power) < 0.1
