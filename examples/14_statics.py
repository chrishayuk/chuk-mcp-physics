"""
Example 14: Statics & Equilibrium (Phase 2.6)

Demonstrates static equilibrium calculations including:
- Force balance
- Torque balance
- Center of mass
- Static friction
- Normal forces
- Complete equilibrium analysis
- Beam reactions

No external services required - uses analytic provider.
"""

import asyncio
from chuk_mcp_physics.tools.statics import (
    check_force_balance,
    check_torque_balance,
    calculate_center_of_mass,
    calculate_static_friction,
    calculate_normal_force,
    check_equilibrium,
    calculate_beam_reactions,
)


async def main():
    print("\n" + "=" * 70)
    print("STATICS & EQUILIBRIUM EXAMPLES (Phase 2.6)")
    print("=" * 70)

    # Example 1: Force Balance - Bridge supports
    print("\n1. Force Balance - Bridge Support Analysis")
    print("-" * 70)

    forces = [[0, 1000, 0], [0, 500, 0], [0, -1500, 0]]  # Two upward, one downward

    result = await check_force_balance(forces=forces, tolerance=0.01)

    print("Forces on bridge:")
    print("  Support 1: [0, 1000, 0] N (upward)")
    print("  Support 2: [0, 500, 0] N (upward)")
    print("  Load: [0, -1500, 0] N (downward)")

    print("\nRESULTS:")
    print(f"  Net force: {result['net_force']}")
    print(f"  Net force magnitude: {result['net_force_magnitude']:.2f} N")
    print(
        f"  Is balanced: {result['is_balanced']} ✓"
        if result["is_balanced"]
        else f"  Is balanced: {result['is_balanced']} ✗"
    )

    # Example 2: Seesaw Balance
    print("\n\n2. Torque Balance - Seesaw")
    print("-" * 70)

    torques = [[0, 0, 100], [0, 0, -100]]  # Equal and opposite torques

    result = await check_torque_balance(torques=torques, tolerance=0.01)

    print("Torques on seesaw:")
    print("  Child 1: 100 N⋅m (clockwise)")
    print("  Child 2: -100 N⋅m (counter-clockwise)")

    print("\nRESULTS:")
    print(f"  Net torque magnitude: {result['net_torque_magnitude']:.2f} N⋅m")
    print(
        f"  Is balanced: {result['is_balanced']} ✓"
        if result["is_balanced"]
        else f"  Is balanced: {result['is_balanced']} ✗"
    )

    # Example 3: Center of Mass
    print("\n\n3. Center of Mass - Three-Body System")
    print("-" * 70)

    masses = [1.0, 2.0, 3.0]
    positions = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]

    result = await calculate_center_of_mass(masses=masses, positions=positions)

    print("System:")
    for i, (m, p) in enumerate(zip(masses, positions)):
        print(f"  Mass {i + 1}: {m:.1f} kg at {p}")

    print("\nRESULTS:")
    print(f"  Center of mass: {result['center_of_mass']}")
    print(f"  Total mass: {result['total_mass']:.1f} kg")
    print(f"  Balance point is at x = {result['center_of_mass'][0]:.2f} m")

    # Example 4: Static Friction - Box on floor
    print("\n\n4. Static Friction - Will the Box Slide?")
    print("-" * 70)

    normal = 100  # N (10 kg box)
    mu_s = 0.5  # Coefficient of static friction

    test_forces = [30, 45, 55]

    print("Box on floor:")
    print(f"  Normal force: {normal} N")
    print(f"  Coefficient of friction: {mu_s}")

    print(f"\n{'Applied Force (N)':<20} {'Will Slip?':<15} {'Friction Force (N)':<20}")
    print("-" * 70)

    for force in test_forces:
        result = await calculate_static_friction(
            normal_force=normal, coefficient_static_friction=mu_s, applied_force=force
        )
        will_slip = "Yes ✗" if result["will_slip"] else "No ✓"
        print(f"{force:<20} {will_slip:<15} {result['friction_force']:<20.1f}")

    # Example 5: Normal Force on Incline
    print("\n\n5. Normal Force - Box on Inclined Plane")
    print("-" * 70)

    mass = 10.0  # kg
    angles = [0, 15, 30, 45]

    print("10 kg box on ramp at various angles:")
    print(f"\n{'Angle (°)':<15} {'Normal Force (N)':<20} {'Parallel Component (N)':<25}")
    print("-" * 70)

    for angle in angles:
        result = await calculate_normal_force(mass=mass, angle_degrees=angle)
        print(
            f"{angle:<15} {result['normal_force']:<20.1f} {result['weight_component_parallel']:<25.1f}"
        )

    # Example 6: Complete Equilibrium Check
    print("\n\n6. Complete Equilibrium - Beam with Multiple Forces")
    print("-" * 70)

    forces = [[0, 100, 0], [0, -100, 0]]
    positions = [[1, 0, 0], [1, 0, 0]]  # Both at same point

    result = await check_equilibrium(forces=forces, force_positions=positions, tolerance=0.01)

    print("System with forces at positions:")
    for i, (f, p) in enumerate(zip(forces, positions)):
        print(f"  Force {i + 1}: {f} N at {p} m")

    print("\nRESULTS:")
    print(
        f"  Force balanced: {result['force_balanced']} ✓"
        if result["force_balanced"]
        else f"  Force balanced: {result['force_balanced']} ✗"
    )
    print(
        f"  Torque balanced: {result['torque_balanced']} ✓"
        if result["torque_balanced"]
        else f"  Torque balanced: {result['torque_balanced']} ✗"
    )
    print(
        f"  In equilibrium: {result['in_equilibrium']} ✓"
        if result["in_equilibrium"]
        else f"  In equilibrium: {result['in_equilibrium']} ✗"
    )

    # Example 7: Beam Reactions
    print("\n\n7. Simply Supported Beam - Reaction Forces")
    print("-" * 70)

    result = await calculate_beam_reactions(
        beam_length=10.0,  # 10 m beam
        loads=[1000, 500],  # Point loads in N
        load_positions=[3.0, 7.0],  # Positions along beam
    )

    print("10 m beam with two point loads:")
    print("  Load 1: 1000 N at 3 m from left")
    print("  Load 2: 500 N at 7 m from left")

    print("\nSupport Reactions:")
    print(f"  Left support: {result['reaction_left']:.1f} N")
    print(f"  Right support: {result['reaction_right']:.1f} N")
    print(f"  Total load: {result['total_load']:.1f} N")
    print(
        f"  Is balanced: {result['is_balanced']} ✓"
        if result["is_balanced"]
        else f"  Is balanced: {result['is_balanced']} ✗"
    )
    print(
        f"  Check: {result['reaction_left'] + result['reaction_right']:.1f} N = {result['total_load']:.1f} N"
    )

    print("\n" + "=" * 70)
    print("Phase 2.6 Complete! ✓")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
