#!/usr/bin/env python3
"""Viscosity and Reynolds Number Demo.

This example demonstrates the viscosity parameter for accurate Reynolds number
calculations across different fluids. The Reynolds number determines flow regime:
    - Re < 2,300: Laminar flow (smooth, predictable)
    - 2,300 < Re < 4,000: Transitional flow
    - Re > 4,000: Turbulent flow (chaotic, eddies)

The viscosity parameter allows accurate Re calculations for any fluid, not just
air and water. This is critical for understanding flow behavior in:
    - Motor oil and lubricants (very viscous)
    - Honey and syrups (extremely viscous)
    - Temperature-sensitive fluids
    - Industrial process fluids
"""

import asyncio
import math

from chuk_mcp_physics.tools.fluid import calculate_drag_force


def classify_flow(reynolds_number: float) -> str:
    """Classify flow regime based on Reynolds number."""
    if reynolds_number < 2300:
        return "Laminar (smooth)"
    elif reynolds_number < 4000:
        return "Transitional"
    else:
        return "Turbulent (chaotic)"


async def demo_viscosity_comparison():
    """Compare drag force across fluids with different viscosities."""
    print("\n" + "=" * 80)
    print("DEMO 1: Same Ball, Different Fluids - The Importance of Viscosity")
    print("=" * 80)
    print("\nScenario: 5cm radius ball falling at 2 m/s through various fluids")
    print("  - Ball radius: 0.05 m (basketball-like)")
    print("  - Velocity: 2 m/s downward")
    print("  - Shape: Sphere (Cd = 0.47)")
    print()

    radius = 0.05
    area = math.pi * radius * radius
    velocity = [0.0, -2.0, 0.0]
    cd = 0.47

    fluids = [
        {
            "name": "Air (20°C)",
            "density": 1.225,
            "viscosity": 1.825e-5,
            "color": "🌫️ ",
        },
        {
            "name": "Water (20°C)",
            "density": 1000.0,
            "viscosity": 1.002e-3,
            "color": "💧",
        },
        {
            "name": "Motor Oil",
            "density": 900.0,
            "viscosity": 0.1,
            "color": "🛢️ ",
        },
        {
            "name": "Honey (approximate)",
            "density": 1420.0,
            "viscosity": 10.0,
            "color": "🍯",
        },
    ]

    print(
        f"{'Fluid':<25} {'Density':<12} {'Viscosity':<15} {'Drag':<10} {'Re':<12} {'Flow Regime':<20}"
    )
    print("-" * 105)

    for fluid in fluids:
        result = await calculate_drag_force(
            velocity=velocity,
            cross_sectional_area=area,
            fluid_density=fluid["density"],
            drag_coefficient=cd,
            viscosity=fluid["viscosity"],  # ← KEY: Explicit viscosity
        )

        flow_regime = classify_flow(result["reynolds_number"])

        print(
            f"{fluid['color']} {fluid['name']:<22} "
            f"{fluid['density']:<12.1f} "
            f"{fluid['viscosity']:<15.2e} "
            f"{result['magnitude']:<10.2f} "
            f"{result['reynolds_number']:<12.0f} "
            f"{flow_regime}"
        )

    print("\n💡 Key Insight: Viscosity ranges over 6 orders of magnitude!")
    print("   Air → Honey: viscosity increases by ~500,000x")
    print("   This completely changes the flow regime and behavior.")


async def demo_motor_oil_accuracy():
    """Show the importance of explicit viscosity for motor oil."""
    print("\n" + "=" * 80)
    print("DEMO 2: Motor Oil - Why Explicit Viscosity Matters")
    print("=" * 80)
    print("\nProblem: Motor oil has water-like density (~900 kg/m³) but 100x higher viscosity!")
    print("Without explicit viscosity, Reynolds number is completely wrong.\n")

    radius = 0.05
    area = math.pi * radius * radius
    velocity = [0.0, -2.0, 0.0]

    # Test 1: WITH explicit viscosity (CORRECT)
    print("✅ WITH explicit viscosity parameter:")
    result_correct = await calculate_drag_force(
        velocity=velocity,
        cross_sectional_area=area,
        fluid_density=900.0,
        drag_coefficient=0.47,
        viscosity=0.1,  # Motor oil actual viscosity
    )

    print("   Viscosity used: 0.1 Pa·s (actual motor oil)")
    print(f"   Reynolds number: {result_correct['reynolds_number']:.0f}")
    print(f"   Flow regime: {classify_flow(result_correct['reynolds_number'])}")
    print(f"   Drag force: {result_correct['magnitude']:.2f} N")

    # Test 2: WITHOUT explicit viscosity (WRONG)
    print("\n❌ WITHOUT explicit viscosity parameter:")
    result_wrong = await calculate_drag_force(
        velocity=velocity,
        cross_sectional_area=area,
        fluid_density=900.0,
        drag_coefficient=0.47,
        # No viscosity - will estimate as water-like (1e-3) because density > 100
    )

    print("   Viscosity used: 1.0e-3 Pa·s (estimated as water-like - WRONG!)")
    print(f"   Reynolds number: {result_wrong['reynolds_number']:.0f}")
    print(f"   Flow regime: {classify_flow(result_wrong['reynolds_number'])} (INCORRECT!)")
    print(f"   Drag force: {result_wrong['magnitude']:.2f} N (same, drag is still correct)")

    # Show the error
    error_factor = result_wrong["reynolds_number"] / result_correct["reynolds_number"]
    print(f"\n⚠️  Error: Reynolds number is {error_factor:.0f}x too high!")
    print("   This leads to completely wrong flow regime classification.")
    print(f"   Actual: {classify_flow(result_correct['reynolds_number'])}")
    print(f"   Estimated: {classify_flow(result_wrong['reynolds_number'])}")


async def demo_temperature_effects():
    """Show how temperature affects viscosity."""
    print("\n" + "=" * 80)
    print("DEMO 3: Temperature Effects on Viscosity")
    print("=" * 80)
    print("\nWater viscosity changes significantly with temperature:")
    print("  - Cold water is more viscous → higher drag, lower Re")
    print("  - Hot water is less viscous → lower drag, higher Re\n")

    radius = 0.05
    area = math.pi * radius * radius
    velocity = [0.0, -2.0, 0.0]

    temperatures = [
        {"temp": 0, "viscosity": 1.787e-3, "emoji": "🧊"},
        {"temp": 20, "viscosity": 1.002e-3, "emoji": "🌡️"},
        {"temp": 60, "viscosity": 0.467e-3, "emoji": "♨️ "},
        {"temp": 100, "viscosity": 0.282e-3, "emoji": "💨"},
    ]

    print(f"{'Temp':<15} {'Viscosity':<20} {'Re':<15} {'Flow Regime'}")
    print("-" * 65)

    for t in temperatures:
        result = await calculate_drag_force(
            velocity=velocity,
            cross_sectional_area=area,
            fluid_density=1000.0,
            drag_coefficient=0.47,
            viscosity=t["viscosity"],
        )

        flow_regime = classify_flow(result["reynolds_number"])
        print(
            f"{t['emoji']} {t['temp']}°C{'':<9} "
            f"{t['viscosity']:<20.3e} "
            f"{result['reynolds_number']:<15.0f} "
            f"{flow_regime}"
        )

    print("\n💡 Key Insight: Viscosity decreases ~6x from ice-cold to boiling!")
    print("   This affects flow behavior significantly.")


async def demo_industrial_fluids():
    """Show Reynolds number for various industrial fluids."""
    print("\n" + "=" * 80)
    print("DEMO 4: Industrial Fluids - Why One Size Doesn't Fit All")
    print("=" * 80)
    print("\nIndustrial processes use fluids with vastly different properties:")
    print("  - Hydraulic oil, glycerin, liquid sugar, etc.")
    print("  - Each requires accurate viscosity for proper flow analysis\n")

    radius = 0.025  # 5cm diameter pipe
    area = math.pi * radius * radius
    velocity = [5.0, 0.0, 0.0]  # Flowing through horizontal pipe

    fluids = [
        {"name": "Hydraulic Oil", "density": 870, "viscosity": 0.05, "emoji": "⚙️ "},
        {"name": "Glycerin", "density": 1260, "viscosity": 1.5, "emoji": "🧴"},
        {"name": "Liquid Sugar", "density": 1400, "viscosity": 3.0, "emoji": "🍬"},
        {"name": "Molasses", "density": 1400, "viscosity": 5.0, "emoji": "🥞"},
    ]

    print(f"{'Fluid':<20} {'Velocity':<12} {'Re':<15} {'Flow Regime':<20} {'Notes'}")
    print("-" * 90)

    for fluid in fluids:
        result = await calculate_drag_force(
            velocity=velocity,
            cross_sectional_area=area,
            fluid_density=fluid["density"],
            drag_coefficient=0.47,
            viscosity=fluid["viscosity"],
        )

        flow_regime = classify_flow(result["reynolds_number"])

        # Add engineering notes
        if result["reynolds_number"] < 2300:
            notes = "Easy to pump"
        elif result["reynolds_number"] < 4000:
            notes = "Careful monitoring"
        else:
            notes = "High turbulence"

        print(
            f"{fluid['emoji']} {fluid['name']:<17} "
            f"{5.0:<12.1f} "
            f"{result['reynolds_number']:<15.0f} "
            f"{flow_regime:<20} "
            f"{notes}"
        )

    print("\n💡 Engineering Insight: Flow regime determines:")
    print("   - Pump requirements (laminar = easier)")
    print("   - Mixing behavior (turbulent = better mixing)")
    print("   - Heat transfer efficiency (turbulent = better)")
    print("   - Pipe friction losses (laminar = lower losses)")


async def demo_backwards_compatibility():
    """Show that old code still works without viscosity parameter."""
    print("\n" + "=" * 80)
    print("DEMO 5: Backwards Compatibility")
    print("=" * 80)
    print("\nOld code without viscosity parameter still works!")
    print("It uses density-based estimation:\n")

    radius = 0.05
    area = math.pi * radius * radius

    # Air-like density (< 100 kg/m³)
    print("Air-like density (1.225 kg/m³):")
    result_air = await calculate_drag_force(
        velocity=[10.0, 0.0, 0.0],
        cross_sectional_area=area,
        fluid_density=1.225,
        drag_coefficient=0.47,
        # No viscosity - will estimate as air-like (1.8e-5)
    )
    print("  ✅ Estimates viscosity: 1.8e-5 Pa·s (air-like)")
    print(f"  Reynolds number: {result_air['reynolds_number']:.0f}")

    # Water-like density (> 100 kg/m³)
    print("\nWater-like density (1000 kg/m³):")
    result_water = await calculate_drag_force(
        velocity=[0.0, -5.0, 0.0],
        cross_sectional_area=area,
        fluid_density=1000,
        drag_coefficient=0.47,
        # No viscosity - will estimate as water-like (1e-3)
    )
    print("  ✅ Estimates viscosity: 1.0e-3 Pa·s (water-like)")
    print(f"  Reynolds number: {result_water['reynolds_number']:.0f}")

    print("\n💡 Backwards Compatibility: Existing code works unchanged!")
    print("   But for accurate Re with other fluids, use the viscosity parameter.")


async def main():
    """Run all demonstrations."""
    print("\n" + "█" * 80)
    print("█" + " " * 78 + "█")
    print("█" + " " * 15 + "VISCOSITY AND REYNOLDS NUMBER DEMONSTRATIONS" + " " * 19 + "█")
    print("█" + " " * 78 + "█")
    print("█" * 80)

    await demo_viscosity_comparison()
    await demo_motor_oil_accuracy()
    await demo_temperature_effects()
    await demo_industrial_fluids()
    await demo_backwards_compatibility()

    print("\n" + "=" * 80)
    print("SUMMARY: Viscosity Parameter Benefits")
    print("=" * 80)
    print("""
✅ Accurate Reynolds number for ANY fluid
✅ Correct flow regime classification
✅ Temperature effects modeling
✅ Industrial fluid analysis
✅ 100% backwards compatible

🎯 When to use viscosity parameter:
   - Motor oil, hydraulic fluids (high viscosity)
   - Honey, syrups, molasses (very high viscosity)
   - Temperature-sensitive applications
   - Industrial process design
   - Any non-standard fluid

📚 Common viscosity values:
   - Air (20°C):      1.825e-5 Pa·s
   - Water (20°C):    1.002e-3 Pa·s
   - Motor oil:       0.1 Pa·s
   - Honey:           ~10 Pa·s
   - Glycerin:        ~1.5 Pa·s

For air and water, viscosity parameter is optional (estimated correctly).
For other fluids, provide explicit viscosity for accurate Reynolds number!
""")
    print("=" * 80)


if __name__ == "__main__":
    asyncio.run(main())
