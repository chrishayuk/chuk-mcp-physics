"""MCP tool for unit conversions."""

from chuk_mcp_server import tool
from ..units import convert_units, get_supported_units


@tool()
def convert_unit(value: float, from_unit: str, to_unit: str) -> dict:
    """
    Convert a value from one unit to another.

    Supports 62 unit types across 16 categories:
    - Velocity: m/s, km/h, mph, ft/s, knots
    - Distance: m, km, mi, ft, yd, in
    - Mass: kg, g, lb, oz
    - Force: N, kN, lbf
    - Energy: J, kJ, cal, BTU, kWh
    - Power: W, kW, hp
    - Temperature: K, C, F
    - Angle: rad, deg
    - Pressure: Pa, kPa, bar, psi, atm
    - Area: m², km², ft², acre
    - Volume: m³, L, gal, ft³
    - Time: s, min, hr, day
    - Acceleration: m/s², g, ft/s²
    - Torque: N·m, lb·ft, lb·in
    - Frequency: Hz, kHz, MHz, GHz
    - Data Size: B, KB, MB, GB

    Enables natural language queries like:
    - "Convert 60 mph to m/s"
    - "How fast is 100 km/h in mph?"
    - "Convert 10 kg to pounds"

    Args:
        value: The numeric value to convert
        from_unit: Source unit (e.g., 'mph', 'kg', 'J')
        to_unit: Target unit (e.g., 'm/s', 'lb', 'kWh')

    Returns:
        Dictionary with:
        - original_value: Input value
        - original_unit: Input unit
        - converted_value: Result value
        - converted_unit: Result unit
        - formatted: Human-readable string

    Examples:
        >>> convert_unit(100, 'm/s', 'mph')
        {
            "original_value": 100,
            "original_unit": "m/s",
            "converted_value": 223.694,
            "converted_unit": "mph",
            "formatted": "100 m/s = 223.694 mph"
        }

        >>> convert_unit(60, 'mph', 'km/h')
        {
            "original_value": 60,
            "original_unit": "mph",
            "converted_value": 96.56064,
            "converted_unit": "km/h",
            "formatted": "60 mph = 96.56 km/h"
        }
    """
    try:
        result = convert_units(value, from_unit, to_unit)

        return {
            "original_value": value,
            "original_unit": from_unit,
            "converted_value": result,
            "converted_unit": to_unit,
            "formatted": f"{value} {from_unit} = {result:.4f} {to_unit}",
        }
    except ValueError as e:
        return {
            "error": str(e),
            "supported_units": get_supported_units(),
        }


@tool()
def list_unit_conversions() -> dict:
    """
    List all supported unit conversions.

    Returns a dictionary mapping category names to lists of supported units.

    Returns:
        Dictionary with supported unit categories:
        - velocity: Speed units
        - distance: Length units
        - mass: Weight units
        - force: Force units
        - energy: Energy units
        - power: Power units
        - temperature: Temperature scales
        - angle: Angular units
        - pressure: Pressure units
        - area: Area units
        - volume: Volume units

    Example:
        >>> list_unit_conversions()
        {
            "velocity": ["m/s", "km/h", "mph", "ft/s", "knots"],
            "distance": ["m", "km", "mi", "ft", "yd", "in"],
            "mass": ["kg", "g", "lb", "oz"],
            ...
        }
    """
    units = get_supported_units()

    # Add examples for each category
    examples = {
        "velocity": "100 m/s = 223.69 mph",
        "distance": "1 km = 0.621 mi",
        "mass": "1 kg = 2.205 lb",
        "force": "100 N = 22.48 lbf",
        "energy": "1000 J = 1 kJ = 0.239 kcal",
        "power": "1000 W = 1 kW = 1.341 hp",
        "temperature": "0 C = 32 F = 273.15 K",
        "angle": "180 deg = π rad",
        "pressure": "101325 Pa = 1 atm = 14.7 psi",
        "area": "1 acre = 4046.86 m²",
        "volume": "1 m³ = 1000 L = 264.17 gal",
    }

    return {
        "categories": units,
        "examples": examples,
        "total_conversions": sum(len(v) for v in units.values()),
    }
