"""Comprehensive tests for tools/statics.py module."""

import json
import pytest


class TestStaticsToolsComplete:
    """Complete test coverage for statics tools."""

    @pytest.mark.asyncio
    async def test_check_force_balance_balanced(self):
        """Test force balance check with balanced forces."""
        from chuk_mcp_physics.tools.statics import check_force_balance

        result = await check_force_balance(
            forces=[[0, 1000, 0], [0, 500, 0], [0, -1500, 0]],
            tolerance=0.01,
        )

        assert "net_force" in result
        assert "net_force_magnitude" in result
        assert "is_balanced" in result
        assert "individual_magnitudes" in result
        assert result["is_balanced"] is True

    @pytest.mark.asyncio
    async def test_check_force_balance_unbalanced(self):
        """Test force balance check with unbalanced forces."""
        from chuk_mcp_physics.tools.statics import check_force_balance

        result = await check_force_balance(
            forces=[[100, 0, 0], [50, 0, 0]],
            tolerance=0.01,
        )

        assert "is_balanced" in result
        assert result["is_balanced"] is False

    @pytest.mark.asyncio
    async def test_check_force_balance_with_json_string(self):
        """Test force balance with JSON string input."""
        from chuk_mcp_physics.tools.statics import check_force_balance

        result = await check_force_balance(
            forces=json.dumps([[0, 100, 0], [0, -100, 0]]),
            tolerance=0.01,
        )

        assert "is_balanced" in result

    @pytest.mark.asyncio
    async def test_check_torque_balance_balanced(self):
        """Test torque balance check with balanced torques."""
        from chuk_mcp_physics.tools.statics import check_torque_balance

        result = await check_torque_balance(
            torques=[[0, 0, 100], [0, 0, -100]],
            tolerance=0.01,
        )

        assert "net_torque" in result
        assert "net_torque_magnitude" in result
        assert "is_balanced" in result
        assert "individual_magnitudes" in result
        assert result["is_balanced"] is True

    @pytest.mark.asyncio
    async def test_check_torque_balance_unbalanced(self):
        """Test torque balance with unbalanced torques."""
        from chuk_mcp_physics.tools.statics import check_torque_balance

        result = await check_torque_balance(
            torques=[[0, 0, 100], [0, 0, 50]],
            tolerance=0.01,
        )

        assert "is_balanced" in result
        assert result["is_balanced"] is False

    @pytest.mark.asyncio
    async def test_check_torque_balance_with_json_string(self):
        """Test torque balance with JSON string input."""
        from chuk_mcp_physics.tools.statics import check_torque_balance

        result = await check_torque_balance(
            torques=json.dumps([[0, 0, 100], [0, 0, -100]]),
            tolerance=0.01,
        )

        assert "is_balanced" in result

    @pytest.mark.asyncio
    async def test_calculate_center_of_mass_with_lists(self):
        """Test center of mass calculation with list inputs."""
        from chuk_mcp_physics.tools.statics import calculate_center_of_mass

        result = await calculate_center_of_mass(
            masses=[1.0, 2.0, 3.0],
            positions=[[0, 0, 0], [1, 0, 0], [2, 0, 0]],
        )

        assert "center_of_mass" in result
        assert "total_mass" in result
        assert result["total_mass"] == 6.0
        # Center of mass = (1*0 + 2*1 + 3*2) / (1+2+3) = 8/6 = 1.333...
        assert result["center_of_mass"][0] == pytest.approx(1.333, rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_center_of_mass_with_json_strings(self):
        """Test center of mass with JSON string inputs."""
        from chuk_mcp_physics.tools.statics import calculate_center_of_mass

        result = await calculate_center_of_mass(
            masses=json.dumps([1.0, 1.0]),
            positions=json.dumps([[0, 0, 0], [2, 0, 0]]),
        )

        assert "center_of_mass" in result
        assert result["center_of_mass"][0] == pytest.approx(1.0, rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_static_friction_no_slip(self):
        """Test static friction when object doesn't slip."""
        from chuk_mcp_physics.tools.statics import calculate_static_friction

        result = await calculate_static_friction(
            normal_force=100,
            coefficient_static_friction=0.5,
            applied_force=40,
        )

        assert "max_static_friction" in result
        assert "will_slip" in result
        assert "friction_force" in result
        assert result["will_slip"] is False

    @pytest.mark.asyncio
    async def test_calculate_static_friction_will_slip(self):
        """Test static friction when object will slip."""
        from chuk_mcp_physics.tools.statics import calculate_static_friction

        result = await calculate_static_friction(
            normal_force=100,
            coefficient_static_friction=0.5,
            applied_force=60,
        )

        assert "will_slip" in result
        assert result["will_slip"] is True

    @pytest.mark.asyncio
    async def test_calculate_static_friction_no_applied_force(self):
        """Test static friction without applied force."""
        from chuk_mcp_physics.tools.statics import calculate_static_friction

        result = await calculate_static_friction(
            normal_force=100,
            coefficient_static_friction=0.5,
        )

        assert "max_static_friction" in result
        assert result["max_static_friction"] == 50.0

    @pytest.mark.asyncio
    async def test_calculate_normal_force_horizontal(self):
        """Test normal force on horizontal surface."""
        from chuk_mcp_physics.tools.statics import calculate_normal_force

        result = await calculate_normal_force(
            mass=10.0,
            gravity=9.81,
            angle_degrees=0.0,
        )

        assert "normal_force" in result
        assert "weight_component_perpendicular" in result
        assert "weight_component_parallel" in result
        assert result["normal_force"] == pytest.approx(98.1, rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_normal_force_inclined(self):
        """Test normal force on inclined plane."""
        from chuk_mcp_physics.tools.statics import calculate_normal_force

        result = await calculate_normal_force(
            mass=10.0,
            gravity=9.81,
            angle_degrees=30.0,
        )

        assert "normal_force" in result
        assert result["normal_force"] < 98.1  # Less than horizontal

    @pytest.mark.asyncio
    async def test_calculate_normal_force_with_additional_force(self):
        """Test normal force with additional perpendicular force."""
        from chuk_mcp_physics.tools.statics import calculate_normal_force

        result = await calculate_normal_force(
            mass=10.0,
            gravity=9.81,
            angle_degrees=0.0,
            additional_force=50.0,
        )

        assert "normal_force" in result
        assert result["normal_force"] == pytest.approx(148.1, rel=0.01)

    @pytest.mark.asyncio
    async def test_check_equilibrium_balanced(self):
        """Test equilibrium check with balanced system."""
        from chuk_mcp_physics.tools.statics import check_equilibrium

        result = await check_equilibrium(
            forces=[[0, 100, 0], [0, -100, 0]],
            force_positions=[[1, 0, 0], [1, 0, 0]],
            tolerance=0.01,
        )

        assert "force_balanced" in result
        assert "torque_balanced" in result
        assert "in_equilibrium" in result
        assert "net_force" in result
        assert "net_torque" in result

    @pytest.mark.asyncio
    async def test_check_equilibrium_with_json_strings(self):
        """Test equilibrium check with JSON string inputs."""
        from chuk_mcp_physics.tools.statics import check_equilibrium

        result = await check_equilibrium(
            forces=json.dumps([[0, 100, 0], [0, -100, 0]]),
            force_positions=json.dumps([[1, 0, 0], [1, 0, 0]]),
            tolerance=0.01,
        )

        assert "in_equilibrium" in result

    @pytest.mark.asyncio
    async def test_check_equilibrium_with_pivot_point(self):
        """Test equilibrium check with custom pivot point."""
        from chuk_mcp_physics.tools.statics import check_equilibrium

        result = await check_equilibrium(
            forces=[[0, 100, 0], [0, -100, 0]],
            force_positions=[[2, 0, 0], [3, 0, 0]],
            pivot_point=[2.5, 0, 0],
            tolerance=0.01,
        )

        assert "torque_balanced" in result

    @pytest.mark.asyncio
    async def test_check_equilibrium_pivot_as_json_string(self):
        """Test equilibrium with pivot point as JSON string."""
        from chuk_mcp_physics.tools.statics import check_equilibrium

        result = await check_equilibrium(
            forces=[[0, 100, 0], [0, -100, 0]],
            force_positions=[[1, 0, 0], [1, 0, 0]],
            pivot_point=json.dumps([0, 0, 0]),
            tolerance=0.01,
        )

        assert "in_equilibrium" in result

    @pytest.mark.asyncio
    async def test_calculate_beam_reactions_with_lists(self):
        """Test beam reaction calculation with list inputs."""
        from chuk_mcp_physics.tools.statics import calculate_beam_reactions

        result = await calculate_beam_reactions(
            beam_length=10.0,
            loads=[1000, 500],
            load_positions=[3.0, 7.0],
        )

        assert "reaction_left" in result
        assert "reaction_right" in result
        assert "total_load" in result
        assert "is_balanced" in result
        assert result["total_load"] == 1500

    @pytest.mark.asyncio
    async def test_calculate_beam_reactions_with_json_strings(self):
        """Test beam reactions with JSON string inputs."""
        from chuk_mcp_physics.tools.statics import calculate_beam_reactions

        result = await calculate_beam_reactions(
            beam_length=10.0,
            loads=json.dumps([1000]),
            load_positions=json.dumps([5.0]),
        )

        assert "reaction_left" in result
        assert "reaction_right" in result
        # Centered load should give equal reactions
        assert result["reaction_left"] == pytest.approx(result["reaction_right"], rel=0.01)

    @pytest.mark.asyncio
    async def test_calculate_beam_reactions_asymmetric(self):
        """Test beam reactions with asymmetric loading."""
        from chuk_mcp_physics.tools.statics import calculate_beam_reactions

        result = await calculate_beam_reactions(
            beam_length=10.0,
            loads=[1000],
            load_positions=[2.0],
        )

        assert "reaction_left" in result
        assert "reaction_right" in result
        # Load closer to left should give higher left reaction
        assert result["reaction_left"] > result["reaction_right"]
