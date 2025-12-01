"""Tests for statics and equilibrium calculations."""

import math


from chuk_mcp_physics.statics import (
    BeamReactionRequest,
    CenterOfMassRequest,
    EquilibriumCheckRequest,
    ForceBalanceRequest,
    NormalForceRequest,
    StaticFrictionRequest,
    TorqueBalanceRequest,
    calculate_beam_reactions,
    calculate_center_of_mass,
    calculate_normal_force,
    calculate_static_friction,
    check_equilibrium,
    check_force_balance,
    check_torque_balance,
)


class TestForceBalance:
    def test_balanced_forces(self):
        """Test forces in equilibrium."""
        request = ForceBalanceRequest(
            forces=[
                [100.0, 0.0, 0.0],
                [-100.0, 0.0, 0.0],
                [0.0, 50.0, 0.0],
                [0.0, -50.0, 0.0],
            ],
            tolerance=0.01,
        )
        result = check_force_balance(request)

        assert result.is_balanced is True
        assert result.net_force_magnitude < 0.1

    def test_unbalanced_forces(self):
        """Test forces not in equilibrium."""
        request = ForceBalanceRequest(
            forces=[
                [100.0, 0.0, 0.0],
                [-50.0, 0.0, 0.0],
            ],
            tolerance=0.01,
        )
        result = check_force_balance(request)

        assert result.is_balanced is False
        assert abs(result.net_force_magnitude - 50.0) < 0.1

    def test_3d_force_balance(self):
        """Test 3D force equilibrium."""
        request = ForceBalanceRequest(
            forces=[
                [10.0, 0.0, 0.0],
                [0.0, 20.0, 0.0],
                [0.0, 0.0, 30.0],
                [-10.0, -20.0, -30.0],
            ],
            tolerance=0.01,
        )
        result = check_force_balance(request)

        assert result.is_balanced is True
        assert len(result.individual_magnitudes) == 4

    def test_single_force(self):
        """Test single force (unbalanced)."""
        request = ForceBalanceRequest(
            forces=[[100.0, 0.0, 0.0]],
            tolerance=0.01,
        )
        result = check_force_balance(request)

        assert result.is_balanced is False
        assert abs(result.net_force_magnitude - 100.0) < 0.1


class TestTorqueBalance:
    def test_balanced_torques(self):
        """Test torques in equilibrium."""
        request = TorqueBalanceRequest(
            torques=[
                [0.0, 0.0, 50.0],
                [0.0, 0.0, -50.0],
            ],
            tolerance=0.01,
        )
        result = check_torque_balance(request)

        assert result.is_balanced is True
        assert result.net_torque_magnitude < 0.1

    def test_unbalanced_torques(self):
        """Test torques not in equilibrium."""
        request = TorqueBalanceRequest(
            torques=[
                [0.0, 0.0, 100.0],
                [0.0, 0.0, -30.0],
            ],
            tolerance=0.01,
        )
        result = check_torque_balance(request)

        assert result.is_balanced is False
        assert abs(result.net_torque_magnitude - 70.0) < 0.1

    def test_3d_torque_balance(self):
        """Test 3D torque equilibrium."""
        request = TorqueBalanceRequest(
            torques=[
                [10.0, 0.0, 0.0],
                [-10.0, 0.0, 0.0],
                [0.0, 5.0, 0.0],
                [0.0, -5.0, 0.0],
            ],
            tolerance=0.01,
        )
        result = check_torque_balance(request)

        assert result.is_balanced is True
        assert len(result.individual_magnitudes) == 4


class TestCenterOfMass:
    def test_two_equal_masses(self):
        """Test center of mass of two equal masses."""
        request = CenterOfMassRequest(
            masses=[1.0, 1.0],
            positions=[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        )
        result = calculate_center_of_mass(request)

        assert abs(result.center_of_mass[0] - 1.0) < 0.01
        assert abs(result.center_of_mass[1]) < 0.01
        assert abs(result.center_of_mass[2]) < 0.01
        assert abs(result.total_mass - 2.0) < 0.01

    def test_three_masses_1d(self):
        """Test center of mass of three masses on x-axis."""
        request = CenterOfMassRequest(
            masses=[1.0, 2.0, 3.0],
            positions=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        )
        result = calculate_center_of_mass(request)

        # COM = (1*0 + 2*1 + 3*2) / 6 = 8/6 ≈ 1.333
        assert abs(result.center_of_mass[0] - (8.0 / 6.0)) < 0.01
        assert abs(result.total_mass - 6.0) < 0.01

    def test_3d_center_of_mass(self):
        """Test center of mass in 3D."""
        request = CenterOfMassRequest(
            masses=[2.0, 3.0, 5.0],
            positions=[
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [7.0, 8.0, 9.0],
            ],
        )
        result = calculate_center_of_mass(request)

        # COM_x = (2*1 + 3*4 + 5*7) / 10 = 49/10 = 4.9
        # COM_y = (2*2 + 3*5 + 5*8) / 10 = 59/10 = 5.9
        # COM_z = (2*3 + 3*6 + 5*9) / 10 = 69/10 = 6.9
        assert abs(result.center_of_mass[0] - 4.9) < 0.01
        assert abs(result.center_of_mass[1] - 5.9) < 0.01
        assert abs(result.center_of_mass[2] - 6.9) < 0.01
        assert abs(result.total_mass - 10.0) < 0.01

    def test_single_mass(self):
        """Test center of mass with single mass."""
        request = CenterOfMassRequest(
            masses=[5.0],
            positions=[[3.0, 4.0, 5.0]],
        )
        result = calculate_center_of_mass(request)

        assert abs(result.center_of_mass[0] - 3.0) < 0.01
        assert abs(result.center_of_mass[1] - 4.0) < 0.01
        assert abs(result.center_of_mass[2] - 5.0) < 0.01


class TestStaticFriction:
    def test_max_friction_only(self):
        """Test maximum static friction calculation."""
        request = StaticFrictionRequest(
            normal_force=100.0,
            coefficient_static_friction=0.5,
        )
        result = calculate_static_friction(request)

        assert abs(result.max_static_friction - 50.0) < 0.1
        assert result.will_slip is None
        assert result.friction_force is None

    def test_no_slip(self):
        """Test object does not slip."""
        request = StaticFrictionRequest(
            normal_force=100.0,
            coefficient_static_friction=0.5,
            applied_force=40.0,
        )
        result = calculate_static_friction(request)

        assert abs(result.max_static_friction - 50.0) < 0.1
        assert result.will_slip is False
        assert abs(result.friction_force - 40.0) < 0.1

    def test_slip(self):
        """Test object will slip."""
        request = StaticFrictionRequest(
            normal_force=100.0,
            coefficient_static_friction=0.5,
            applied_force=60.0,
        )
        result = calculate_static_friction(request)

        assert abs(result.max_static_friction - 50.0) < 0.1
        assert result.will_slip is True
        assert abs(result.friction_force - 50.0) < 0.1

    def test_exactly_at_limit(self):
        """Test applied force exactly at friction limit."""
        request = StaticFrictionRequest(
            normal_force=100.0,
            coefficient_static_friction=0.5,
            applied_force=50.0,
        )
        result = calculate_static_friction(request)

        assert result.will_slip is False
        assert abs(result.friction_force - 50.0) < 0.1


class TestNormalForce:
    def test_horizontal_surface(self):
        """Test normal force on horizontal surface."""
        request = NormalForceRequest(
            mass=10.0,
            angle_degrees=0.0,
        )
        result = calculate_normal_force(request)

        expected_normal = 10.0 * 9.81
        assert abs(result.normal_force - expected_normal) < 0.1
        assert abs(result.weight_component_perpendicular - expected_normal) < 0.1
        assert abs(result.weight_component_parallel) < 0.1

    def test_30_degree_incline(self):
        """Test normal force on 30° incline."""
        request = NormalForceRequest(
            mass=10.0,
            angle_degrees=30.0,
        )
        result = calculate_normal_force(request)

        weight = 10.0 * 9.81
        expected_normal = weight * math.cos(math.radians(30.0))
        expected_parallel = weight * math.sin(math.radians(30.0))

        assert abs(result.normal_force - expected_normal) < 0.1
        assert abs(result.weight_component_perpendicular - expected_normal) < 0.1
        assert abs(result.weight_component_parallel - expected_parallel) < 0.1

    def test_45_degree_incline(self):
        """Test normal force on 45° incline."""
        request = NormalForceRequest(
            mass=5.0,
            angle_degrees=45.0,
        )
        result = calculate_normal_force(request)

        weight = 5.0 * 9.81
        # At 45°, perpendicular and parallel components are equal
        expected_component = weight / math.sqrt(2)

        assert abs(result.normal_force - expected_component) < 0.1
        assert abs(result.weight_component_perpendicular - expected_component) < 0.1
        assert abs(result.weight_component_parallel - expected_component) < 0.1

    def test_with_additional_force(self):
        """Test normal force with additional perpendicular force."""
        request = NormalForceRequest(
            mass=10.0,
            angle_degrees=0.0,
            additional_force=50.0,
        )
        result = calculate_normal_force(request)

        expected_normal = 10.0 * 9.81 + 50.0
        assert abs(result.normal_force - expected_normal) < 0.1


class TestEquilibrium:
    def test_complete_equilibrium(self):
        """Test system in complete static equilibrium."""
        # Two equal and opposite forces at symmetric positions create balanced torques
        request = EquilibriumCheckRequest(
            forces=[
                [0.0, 100.0, 0.0],  # Upward force at x=1
                [0.0, -100.0, 0.0],  # Downward force at x=-1
            ],
            force_positions=[
                [1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0],
            ],
            pivot_point=[0.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_equilibrium(request)

        # Forces balance: 100 - 100 = 0 ✓
        assert result.force_balanced is True
        # Torques: r₁ × F₁ = [1,0,0] × [0,100,0] = [0,0,100]
        #          r₂ × F₂ = [-1,0,0] × [0,-100,0] = [0,0,100]
        # Net torque = [0,0,200] ✗ NOT balanced
        # This is actually NOT in equilibrium - need different setup
        assert result.torque_balanced is False
        assert result.in_equilibrium is False

    def test_complete_equilibrium_correct(self):
        """Test system in complete static equilibrium (corrected)."""
        # Use forces that create opposing torques
        request = EquilibriumCheckRequest(
            forces=[
                [0.0, 100.0, 0.0],  # Upward force at x=1
                [0.0, -100.0, 0.0],  # Downward force at x=1 (same position)
            ],
            force_positions=[
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            pivot_point=[0.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_equilibrium(request)

        # Forces balance ✓
        # Torques cancel (same position, opposite forces) ✓
        assert result.force_balanced is True
        assert result.torque_balanced is True
        assert result.in_equilibrium is True

    def test_force_balanced_torque_unbalanced(self):
        """Test force balanced but torque not balanced."""
        request = EquilibriumCheckRequest(
            forces=[
                [0.0, 100.0, 0.0],
                [0.0, -100.0, 0.0],
            ],
            force_positions=[
                [2.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            pivot_point=[0.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_equilibrium(request)

        assert result.force_balanced is True
        assert result.torque_balanced is False
        assert result.in_equilibrium is False

    def test_torque_balanced_force_unbalanced(self):
        """Test torque balanced but force not balanced."""
        request = EquilibriumCheckRequest(
            forces=[
                [0.0, 100.0, 0.0],
                [0.0, -50.0, 0.0],
            ],
            force_positions=[
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ],
            pivot_point=[0.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_equilibrium(request)

        assert result.force_balanced is False
        # Torque balanced: 100*1 - 50*2 = 0
        assert result.torque_balanced is True
        assert result.in_equilibrium is False

    def test_neither_balanced(self):
        """Test neither force nor torque balanced."""
        request = EquilibriumCheckRequest(
            forces=[
                [0.0, 100.0, 0.0],
                [0.0, -30.0, 0.0],
            ],
            force_positions=[
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            pivot_point=[0.0, 0.0, 0.0],
            tolerance=0.01,
        )
        result = check_equilibrium(request)

        assert result.force_balanced is False
        assert result.torque_balanced is False
        assert result.in_equilibrium is False


class TestBeamReactions:
    def test_centered_load(self):
        """Test beam with single centered load."""
        request = BeamReactionRequest(
            beam_length=10.0,
            loads=[1000.0],
            load_positions=[5.0],
        )
        result = calculate_beam_reactions(request)

        # Centered load: reactions should be equal
        assert abs(result.reaction_left - 500.0) < 0.1
        assert abs(result.reaction_right - 500.0) < 0.1
        assert abs(result.total_load - 1000.0) < 0.1
        assert result.is_balanced is True

    def test_off_center_load(self):
        """Test beam with off-center load."""
        request = BeamReactionRequest(
            beam_length=10.0,
            loads=[1000.0],
            load_positions=[3.0],
        )
        result = calculate_beam_reactions(request)

        # Moment about left: R_right * 10 = 1000 * 3
        # R_right = 300, R_left = 700
        assert abs(result.reaction_right - 300.0) < 0.1
        assert abs(result.reaction_left - 700.0) < 0.1
        assert abs(result.total_load - 1000.0) < 0.1
        assert result.is_balanced is True

    def test_multiple_loads(self):
        """Test beam with multiple loads."""
        request = BeamReactionRequest(
            beam_length=10.0,
            loads=[1000.0, 500.0],
            load_positions=[3.0, 7.0],
        )
        result = calculate_beam_reactions(request)

        # Total load = 1500
        # Moment about left: R_right * 10 = 1000*3 + 500*7 = 6500
        # R_right = 650, R_left = 850
        assert abs(result.reaction_right - 650.0) < 0.1
        assert abs(result.reaction_left - 850.0) < 0.1
        assert abs(result.total_load - 1500.0) < 0.1
        assert result.is_balanced is True

    def test_three_loads(self):
        """Test beam with three loads."""
        request = BeamReactionRequest(
            beam_length=12.0,
            loads=[600.0, 800.0, 400.0],
            load_positions=[2.0, 6.0, 10.0],
        )
        result = calculate_beam_reactions(request)

        # Total load = 1800
        # Moment about left: R_right * 12 = 600*2 + 800*6 + 400*10 = 10000
        # R_right = 10000/12 ≈ 833.33, R_left ≈ 966.67
        assert abs(result.reaction_right - (10000.0 / 12.0)) < 0.1
        assert abs(result.reaction_left - (1800.0 - 10000.0 / 12.0)) < 0.1
        assert abs(result.total_load - 1800.0) < 0.1
        assert result.is_balanced is True

    def test_load_at_left_support(self):
        """Test beam with load at left support."""
        request = BeamReactionRequest(
            beam_length=10.0,
            loads=[1000.0],
            load_positions=[0.0],
        )
        result = calculate_beam_reactions(request)

        # Load at left: all reaction at left
        assert abs(result.reaction_left - 1000.0) < 0.1
        assert abs(result.reaction_right) < 0.1
        assert result.is_balanced is True

    def test_load_at_right_support(self):
        """Test beam with load at right support."""
        request = BeamReactionRequest(
            beam_length=10.0,
            loads=[1000.0],
            load_positions=[10.0],
        )
        result = calculate_beam_reactions(request)

        # Load at right: all reaction at right
        assert abs(result.reaction_right - 1000.0) < 0.1
        assert abs(result.reaction_left) < 0.1
        assert result.is_balanced is True
