"""
Unit conversion utilities for physics calculations.

Supports common conversions for natural language queries:
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
"""


# ============================================================================
# VELOCITY CONVERSIONS
# ============================================================================


def ms_to_mph(ms: float) -> float:
    """Convert meters per second to miles per hour."""
    return ms * 2.23694


def mph_to_ms(mph: float) -> float:
    """Convert miles per hour to meters per second."""
    return mph / 2.23694


def ms_to_kmh(ms: float) -> float:
    """Convert meters per second to kilometers per hour."""
    return ms * 3.6


def kmh_to_ms(kmh: float) -> float:
    """Convert kilometers per hour to meters per second."""
    return kmh / 3.6


def ms_to_fts(ms: float) -> float:
    """Convert meters per second to feet per second."""
    return ms * 3.28084


def fts_to_ms(fts: float) -> float:
    """Convert feet per second to meters per second."""
    return fts / 3.28084


def ms_to_knots(ms: float) -> float:
    """Convert meters per second to knots."""
    return ms * 1.94384


def knots_to_ms(knots: float) -> float:
    """Convert knots to meters per second."""
    return knots / 1.94384


# ============================================================================
# DISTANCE CONVERSIONS
# ============================================================================


def m_to_km(m: float) -> float:
    """Convert meters to kilometers."""
    return m / 1000


def km_to_m(km: float) -> float:
    """Convert kilometers to meters."""
    return km * 1000


def m_to_mi(m: float) -> float:
    """Convert meters to miles."""
    return m / 1609.34


def mi_to_m(mi: float) -> float:
    """Convert miles to meters."""
    return mi * 1609.34


def m_to_ft(m: float) -> float:
    """Convert meters to feet."""
    return m * 3.28084


def ft_to_m(ft: float) -> float:
    """Convert feet to meters."""
    return ft / 3.28084


def m_to_yd(m: float) -> float:
    """Convert meters to yards."""
    return m * 1.09361


def yd_to_m(yd: float) -> float:
    """Convert yards to meters."""
    return yd / 1.09361


def m_to_in(m: float) -> float:
    """Convert meters to inches."""
    return m * 39.3701


def in_to_m(inches: float) -> float:
    """Convert inches to meters."""
    return inches / 39.3701


# ============================================================================
# MASS CONVERSIONS
# ============================================================================


def kg_to_g(kg: float) -> float:
    """Convert kilograms to grams."""
    return kg * 1000


def g_to_kg(g: float) -> float:
    """Convert grams to kilograms."""
    return g / 1000


def kg_to_lb(kg: float) -> float:
    """Convert kilograms to pounds."""
    return kg * 2.20462


def lb_to_kg(lb: float) -> float:
    """Convert pounds to kilograms."""
    return lb / 2.20462


def kg_to_oz(kg: float) -> float:
    """Convert kilograms to ounces."""
    return kg * 35.274


def oz_to_kg(oz: float) -> float:
    """Convert ounces to kilograms."""
    return oz / 35.274


# ============================================================================
# FORCE CONVERSIONS
# ============================================================================


def n_to_kn(n: float) -> float:
    """Convert newtons to kilonewtons."""
    return n / 1000


def kn_to_n(kn: float) -> float:
    """Convert kilonewtons to newtons."""
    return kn * 1000


def n_to_lbf(n: float) -> float:
    """Convert newtons to pound-force."""
    return n * 0.224809


def lbf_to_n(lbf: float) -> float:
    """Convert pound-force to newtons."""
    return lbf / 0.224809


# ============================================================================
# ENERGY CONVERSIONS
# ============================================================================


def j_to_kj(j: float) -> float:
    """Convert joules to kilojoules."""
    return j / 1000


def kj_to_j(kj: float) -> float:
    """Convert kilojoules to joules."""
    return kj * 1000


def j_to_cal(j: float) -> float:
    """Convert joules to calories."""
    return j * 0.239006


def cal_to_j(cal: float) -> float:
    """Convert calories to joules."""
    return cal / 0.239006


def j_to_kwh(j: float) -> float:
    """Convert joules to kilowatt-hours."""
    return j / 3_600_000


def kwh_to_j(kwh: float) -> float:
    """Convert kilowatt-hours to joules."""
    return kwh * 3_600_000


def j_to_btu(j: float) -> float:
    """Convert joules to BTU (British Thermal Units)."""
    return j * 0.000947817


def btu_to_j(btu: float) -> float:
    """Convert BTU to joules."""
    return btu / 0.000947817


# ============================================================================
# POWER CONVERSIONS
# ============================================================================


def w_to_kw(w: float) -> float:
    """Convert watts to kilowatts."""
    return w / 1000


def kw_to_w(kw: float) -> float:
    """Convert kilowatts to watts."""
    return kw * 1000


def w_to_hp(w: float) -> float:
    """Convert watts to horsepower."""
    return w / 745.7


def hp_to_w(hp: float) -> float:
    """Convert horsepower to watts."""
    return hp * 745.7


# ============================================================================
# TEMPERATURE CONVERSIONS
# ============================================================================


def c_to_k(c: float) -> float:
    """Convert Celsius to Kelvin."""
    return c + 273.15


def k_to_c(k: float) -> float:
    """Convert Kelvin to Celsius."""
    return k - 273.15


def c_to_f(c: float) -> float:
    """Convert Celsius to Fahrenheit."""
    return (c * 9 / 5) + 32


def f_to_c(f: float) -> float:
    """Convert Fahrenheit to Celsius."""
    return (f - 32) * 5 / 9


def f_to_k(f: float) -> float:
    """Convert Fahrenheit to Kelvin."""
    return c_to_k(f_to_c(f))


def k_to_f(k: float) -> float:
    """Convert Kelvin to Fahrenheit."""
    return c_to_f(k_to_c(k))


# ============================================================================
# ANGLE CONVERSIONS
# ============================================================================


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    import math

    return deg * (math.pi / 180)


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees."""
    import math

    return rad * (180 / math.pi)


# ============================================================================
# PRESSURE CONVERSIONS
# ============================================================================


def pa_to_kpa(pa: float) -> float:
    """Convert pascals to kilopascals."""
    return pa / 1000


def kpa_to_pa(kpa: float) -> float:
    """Convert kilopascals to pascals."""
    return kpa * 1000


def pa_to_bar(pa: float) -> float:
    """Convert pascals to bar."""
    return pa / 100_000


def bar_to_pa(bar: float) -> float:
    """Convert bar to pascals."""
    return bar * 100_000


def pa_to_psi(pa: float) -> float:
    """Convert pascals to PSI (pounds per square inch)."""
    return pa * 0.000145038


def psi_to_pa(psi: float) -> float:
    """Convert PSI to pascals."""
    return psi / 0.000145038


def pa_to_atm(pa: float) -> float:
    """Convert pascals to atmospheres."""
    return pa / 101_325


def atm_to_pa(atm: float) -> float:
    """Convert atmospheres to pascals."""
    return atm * 101_325


# ============================================================================
# AREA CONVERSIONS
# ============================================================================


def m2_to_km2(m2: float) -> float:
    """Convert square meters to square kilometers."""
    return m2 / 1_000_000


def km2_to_m2(km2: float) -> float:
    """Convert square kilometers to square meters."""
    return km2 * 1_000_000


def m2_to_ft2(m2: float) -> float:
    """Convert square meters to square feet."""
    return m2 * 10.7639


def ft2_to_m2(ft2: float) -> float:
    """Convert square feet to square meters."""
    return ft2 / 10.7639


def m2_to_acre(m2: float) -> float:
    """Convert square meters to acres."""
    return m2 / 4046.86


def acre_to_m2(acre: float) -> float:
    """Convert acres to square meters."""
    return acre * 4046.86


# ============================================================================
# VOLUME CONVERSIONS
# ============================================================================


def m3_to_l(m3: float) -> float:
    """Convert cubic meters to liters."""
    return m3 * 1000


def l_to_m3(liters: float) -> float:
    """Convert liters to cubic meters."""
    return liters / 1000


def m3_to_gal(m3: float) -> float:
    """Convert cubic meters to US gallons."""
    return m3 * 264.172


def gal_to_m3(gal: float) -> float:
    """Convert US gallons to cubic meters."""
    return gal / 264.172


def m3_to_ft3(m3: float) -> float:
    """Convert cubic meters to cubic feet."""
    return m3 * 35.3147


def ft3_to_m3(ft3: float) -> float:
    """Convert cubic feet to cubic meters."""
    return ft3 / 35.3147


# ============================================================================
# TIME CONVERSIONS
# ============================================================================


def s_to_min(s: float) -> float:
    """Convert seconds to minutes."""
    return s / 60


def min_to_s(minutes: float) -> float:
    """Convert minutes to seconds."""
    return minutes * 60


def s_to_hr(s: float) -> float:
    """Convert seconds to hours."""
    return s / 3600


def hr_to_s(hr: float) -> float:
    """Convert hours to seconds."""
    return hr * 3600


def s_to_day(s: float) -> float:
    """Convert seconds to days."""
    return s / 86400


def day_to_s(days: float) -> float:
    """Convert days to seconds."""
    return days * 86400


def min_to_hr(minutes: float) -> float:
    """Convert minutes to hours."""
    return minutes / 60


def hr_to_min(hr: float) -> float:
    """Convert hours to minutes."""
    return hr * 60


def hr_to_day(hr: float) -> float:
    """Convert hours to days."""
    return hr / 24


def day_to_hr(days: float) -> float:
    """Convert days to hours."""
    return days * 24


# ============================================================================
# ACCELERATION CONVERSIONS
# ============================================================================


def ms2_to_g(ms2: float) -> float:
    """Convert m/s² to g-force (standard gravity = 9.80665 m/s²)."""
    return ms2 / 9.80665


def g_to_ms2(g: float) -> float:
    """Convert g-force to m/s²."""
    return g * 9.80665


def ms2_to_fts2(ms2: float) -> float:
    """Convert m/s² to ft/s²."""
    return ms2 * 3.28084


def fts2_to_ms2(fts2: float) -> float:
    """Convert ft/s² to m/s²."""
    return fts2 / 3.28084


# ============================================================================
# TORQUE CONVERSIONS
# ============================================================================


def nm_to_lbft(nm: float) -> float:
    """Convert newton-meters to pound-feet."""
    return nm * 0.737562


def lbft_to_nm(lbft: float) -> float:
    """Convert pound-feet to newton-meters."""
    return lbft / 0.737562


def nm_to_lbin(nm: float) -> float:
    """Convert newton-meters to pound-inches."""
    return nm * 8.85075


def lbin_to_nm(lbin: float) -> float:
    """Convert pound-inches to newton-meters."""
    return lbin / 8.85075


# ============================================================================
# FREQUENCY CONVERSIONS
# ============================================================================


def hz_to_khz(hz: float) -> float:
    """Convert hertz to kilohertz."""
    return hz / 1000


def khz_to_hz(khz: float) -> float:
    """Convert kilohertz to hertz."""
    return khz * 1000


def hz_to_mhz(hz: float) -> float:
    """Convert hertz to megahertz."""
    return hz / 1_000_000


def mhz_to_hz(mhz: float) -> float:
    """Convert megahertz to hertz."""
    return mhz * 1_000_000


def hz_to_ghz(hz: float) -> float:
    """Convert hertz to gigahertz."""
    return hz / 1_000_000_000


def ghz_to_hz(ghz: float) -> float:
    """Convert gigahertz to hertz."""
    return ghz * 1_000_000_000


def khz_to_mhz(khz: float) -> float:
    """Convert kilohertz to megahertz."""
    return khz / 1000


def mhz_to_khz(mhz: float) -> float:
    """Convert megahertz to kilohertz."""
    return mhz * 1000


def mhz_to_ghz(mhz: float) -> float:
    """Convert megahertz to gigahertz."""
    return mhz / 1000


def ghz_to_mhz(ghz: float) -> float:
    """Convert gigahertz to megahertz."""
    return ghz * 1000


# ============================================================================
# DATA SIZE CONVERSIONS
# ============================================================================


def b_to_kb(b: float) -> float:
    """Convert bytes to kilobytes (1 KB = 1024 bytes)."""
    return b / 1024


def kb_to_b(kb: float) -> float:
    """Convert kilobytes to bytes."""
    return kb * 1024


def b_to_mb(b: float) -> float:
    """Convert bytes to megabytes."""
    return b / (1024 * 1024)


def mb_to_b(mb: float) -> float:
    """Convert megabytes to bytes."""
    return mb * 1024 * 1024


def b_to_gb(b: float) -> float:
    """Convert bytes to gigabytes."""
    return b / (1024 * 1024 * 1024)


def gb_to_b(gb: float) -> float:
    """Convert gigabytes to bytes."""
    return gb * 1024 * 1024 * 1024


def kb_to_mb(kb: float) -> float:
    """Convert kilobytes to megabytes."""
    return kb / 1024


def mb_to_kb(mb: float) -> float:
    """Convert megabytes to kilobytes."""
    return mb * 1024


def mb_to_gb(mb: float) -> float:
    """Convert megabytes to gigabytes."""
    return mb / 1024


def gb_to_mb(gb: float) -> float:
    """Convert gigabytes to megabytes."""
    return gb * 1024


# ============================================================================
# CONVENIENCE CONVERSION FUNCTION
# ============================================================================

CONVERSION_MAP = {
    # Velocity
    ("m/s", "mph"): ms_to_mph,
    ("mph", "m/s"): mph_to_ms,
    ("m/s", "km/h"): ms_to_kmh,
    ("km/h", "m/s"): kmh_to_ms,
    ("m/s", "ft/s"): ms_to_fts,
    ("ft/s", "m/s"): fts_to_ms,
    ("m/s", "knots"): ms_to_knots,
    ("knots", "m/s"): knots_to_ms,
    # Distance
    ("m", "km"): m_to_km,
    ("km", "m"): km_to_m,
    ("m", "mi"): m_to_mi,
    ("mi", "m"): mi_to_m,
    ("m", "ft"): m_to_ft,
    ("ft", "m"): ft_to_m,
    ("m", "yd"): m_to_yd,
    ("yd", "m"): yd_to_m,
    ("m", "in"): m_to_in,
    ("in", "m"): in_to_m,
    # Mass
    ("kg", "g"): kg_to_g,
    ("g", "kg"): g_to_kg,
    ("kg", "lb"): kg_to_lb,
    ("lb", "kg"): lb_to_kg,
    ("kg", "oz"): kg_to_oz,
    ("oz", "kg"): oz_to_kg,
    # Force
    ("N", "kN"): n_to_kn,
    ("kN", "N"): kn_to_n,
    ("N", "lbf"): n_to_lbf,
    ("lbf", "N"): lbf_to_n,
    # Energy
    ("J", "kJ"): j_to_kj,
    ("kJ", "J"): kj_to_j,
    ("J", "cal"): j_to_cal,
    ("cal", "J"): cal_to_j,
    ("J", "kWh"): j_to_kwh,
    ("kWh", "J"): kwh_to_j,
    ("J", "BTU"): j_to_btu,
    ("BTU", "J"): btu_to_j,
    # Power
    ("W", "kW"): w_to_kw,
    ("kW", "W"): kw_to_w,
    ("W", "hp"): w_to_hp,
    ("hp", "W"): hp_to_w,
    # Temperature
    ("C", "K"): c_to_k,
    ("K", "C"): k_to_c,
    ("C", "F"): c_to_f,
    ("F", "C"): f_to_c,
    ("F", "K"): f_to_k,
    ("K", "F"): k_to_f,
    # Angle
    ("deg", "rad"): deg_to_rad,
    ("rad", "deg"): rad_to_deg,
    # Pressure
    ("Pa", "kPa"): pa_to_kpa,
    ("kPa", "Pa"): kpa_to_pa,
    ("Pa", "bar"): pa_to_bar,
    ("bar", "Pa"): bar_to_pa,
    ("Pa", "psi"): pa_to_psi,
    ("psi", "Pa"): psi_to_pa,
    ("Pa", "atm"): pa_to_atm,
    ("atm", "Pa"): atm_to_pa,
    # Area
    ("m²", "km²"): m2_to_km2,
    ("km²", "m²"): km2_to_m2,
    ("m²", "ft²"): m2_to_ft2,
    ("ft²", "m²"): ft2_to_m2,
    ("m²", "acre"): m2_to_acre,
    ("acre", "m²"): acre_to_m2,
    # Volume
    ("m³", "L"): m3_to_l,
    ("L", "m³"): l_to_m3,
    ("m³", "gal"): m3_to_gal,
    ("gal", "m³"): gal_to_m3,
    ("m³", "ft³"): m3_to_ft3,
    ("ft³", "m³"): ft3_to_m3,
    # Time
    ("s", "min"): s_to_min,
    ("min", "s"): min_to_s,
    ("s", "hr"): s_to_hr,
    ("hr", "s"): hr_to_s,
    ("s", "day"): s_to_day,
    ("day", "s"): day_to_s,
    ("min", "hr"): min_to_hr,
    ("hr", "min"): hr_to_min,
    ("hr", "day"): hr_to_day,
    ("day", "hr"): day_to_hr,
    # Acceleration
    ("m/s²", "g"): ms2_to_g,
    ("g", "m/s²"): g_to_ms2,
    ("m/s²", "ft/s²"): ms2_to_fts2,
    ("ft/s²", "m/s²"): fts2_to_ms2,
    # Torque
    ("N·m", "lb·ft"): nm_to_lbft,
    ("lb·ft", "N·m"): lbft_to_nm,
    ("N·m", "lb·in"): nm_to_lbin,
    ("lb·in", "N·m"): lbin_to_nm,
    # Frequency
    ("Hz", "kHz"): hz_to_khz,
    ("kHz", "Hz"): khz_to_hz,
    ("Hz", "MHz"): hz_to_mhz,
    ("MHz", "Hz"): mhz_to_hz,
    ("Hz", "GHz"): hz_to_ghz,
    ("GHz", "Hz"): ghz_to_hz,
    ("kHz", "MHz"): khz_to_mhz,
    ("MHz", "kHz"): mhz_to_khz,
    ("MHz", "GHz"): mhz_to_ghz,
    ("GHz", "MHz"): ghz_to_mhz,
    # Data Size
    ("B", "KB"): b_to_kb,
    ("KB", "B"): kb_to_b,
    ("B", "MB"): b_to_mb,
    ("MB", "B"): mb_to_b,
    ("B", "GB"): b_to_gb,
    ("GB", "B"): gb_to_b,
    ("KB", "MB"): kb_to_mb,
    ("MB", "KB"): mb_to_kb,
    ("MB", "GB"): mb_to_gb,
    ("GB", "MB"): gb_to_mb,
}


def convert_units(value: float, from_unit: str, to_unit: str) -> float:
    """
    Convert a value from one unit to another.

    Args:
        value: The numeric value to convert
        from_unit: The source unit (e.g., 'm/s', 'mph', 'kg')
        to_unit: The target unit (e.g., 'mph', 'm/s', 'lb')

    Returns:
        The converted value

    Raises:
        ValueError: If the conversion is not supported

    Examples:
        >>> convert_units(100, 'm/s', 'mph')
        223.694
        >>> convert_units(60, 'mph', 'km/h')
        96.56064
        >>> convert_units(10, 'kg', 'lb')
        22.0462
    """
    # Handle identity conversions
    if from_unit == to_unit:
        return value

    # Try direct conversion
    key = (from_unit, to_unit)
    if key in CONVERSION_MAP:
        return CONVERSION_MAP[key](value)

    # Try indirect conversion through SI base unit
    # For example: mph -> km/h = mph -> m/s -> km/h
    if from_unit in ["mph", "km/h", "ft/s", "knots"] and to_unit in [
        "mph",
        "km/h",
        "ft/s",
        "knots",
    ]:
        # Convert to m/s, then to target
        ms_value = convert_units(value, from_unit, "m/s")
        return convert_units(ms_value, "m/s", to_unit)

    # For distance units: convert through meters
    if from_unit in ["km", "mi", "ft", "yd", "in"] and to_unit in ["km", "mi", "ft", "yd", "in"]:
        # Convert to m, then to target
        m_value = convert_units(value, from_unit, "m")
        return convert_units(m_value, "m", to_unit)

    # For mass units: convert through kg
    if from_unit in ["g", "lb", "oz"] and to_unit in ["g", "lb", "oz"]:
        # Convert to kg, then to target
        kg_value = convert_units(value, from_unit, "kg")
        return convert_units(kg_value, "kg", to_unit)

    # For temperature units: convert through K
    if from_unit in ["C", "F"] and to_unit in ["C", "F"]:
        # Convert to K, then to target
        k_value = convert_units(value, from_unit, "K")
        return convert_units(k_value, "K", to_unit)

    # For time units: convert through seconds
    if from_unit in ["min", "hr", "day"] and to_unit in ["min", "hr", "day"]:
        # Convert to s, then to target
        s_value = convert_units(value, from_unit, "s")
        return convert_units(s_value, "s", to_unit)

    # For acceleration units: convert through m/s²
    if from_unit in ["g", "ft/s²"] and to_unit in ["g", "ft/s²"]:
        # Convert to m/s², then to target
        ms2_value = convert_units(value, from_unit, "m/s²")
        return convert_units(ms2_value, "m/s²", to_unit)

    # For torque units: convert through N·m
    if from_unit in ["lb·ft", "lb·in"] and to_unit in ["lb·ft", "lb·in"]:
        # Convert to N·m, then to target
        nm_value = convert_units(value, from_unit, "N·m")
        return convert_units(nm_value, "N·m", to_unit)

    # For frequency units: convert through Hz
    if from_unit in ["kHz", "MHz", "GHz"] and to_unit in ["kHz", "MHz", "GHz"]:
        # Convert to Hz, then to target
        hz_value = convert_units(value, from_unit, "Hz")
        return convert_units(hz_value, "Hz", to_unit)

    # For data size units: convert through bytes
    if from_unit in ["KB", "MB", "GB"] and to_unit in ["KB", "MB", "GB"]:
        # Convert to B, then to target
        b_value = convert_units(value, from_unit, "B")
        return convert_units(b_value, "B", to_unit)

    raise ValueError(f"Conversion from '{from_unit}' to '{to_unit}' not supported")


def get_supported_units() -> dict[str, list[str]]:
    """
    Get a dictionary of supported unit categories and their units.

    Returns:
        Dictionary mapping category names to lists of supported units
    """
    return {
        "velocity": ["m/s", "km/h", "mph", "ft/s", "knots"],
        "distance": ["m", "km", "mi", "ft", "yd", "in"],
        "mass": ["kg", "g", "lb", "oz"],
        "force": ["N", "kN", "lbf"],
        "energy": ["J", "kJ", "cal", "BTU", "kWh"],
        "power": ["W", "kW", "hp"],
        "temperature": ["K", "C", "F"],
        "angle": ["rad", "deg"],
        "pressure": ["Pa", "kPa", "bar", "psi", "atm"],
        "area": ["m²", "km²", "ft²", "acre"],
        "volume": ["m³", "L", "gal", "ft³"],
        "time": ["s", "min", "hr", "day"],
        "acceleration": ["m/s²", "g", "ft/s²"],
        "torque": ["N·m", "lb·ft", "lb·in"],
        "frequency": ["Hz", "kHz", "MHz", "GHz"],
        "data_size": ["B", "KB", "MB", "GB"],
    }
