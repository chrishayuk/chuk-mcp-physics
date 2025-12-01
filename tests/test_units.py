"""Tests for unit conversion module."""

import pytest
import math
from chuk_mcp_physics.units import (
    # Velocity
    ms_to_mph,
    mph_to_ms,
    ms_to_kmh,
    kmh_to_ms,
    # Distance
    m_to_km,
    km_to_m,
    m_to_mi,
    mi_to_m,
    m_to_ft,
    ft_to_m,
    # Mass
    kg_to_lb,
    lb_to_kg,
    kg_to_g,
    g_to_kg,
    # Temperature
    c_to_f,
    f_to_c,
    c_to_k,
    k_to_c,
    # Angle
    deg_to_rad,
    rad_to_deg,
    # Main conversion function
    convert_units,
    get_supported_units,
)


class TestVelocityConversions:
    """Test velocity unit conversions."""

    def test_ms_to_mph(self):
        assert abs(ms_to_mph(10) - 22.3694) < 0.001
        assert abs(ms_to_mph(100) - 223.694) < 0.001

    def test_mph_to_ms(self):
        assert abs(mph_to_ms(60) - 26.8224) < 0.001
        assert abs(mph_to_ms(100) - 44.704) < 0.001

    def test_roundtrip_ms_mph(self):
        original = 50.0
        converted = ms_to_mph(original)
        back = mph_to_ms(converted)
        assert abs(original - back) < 0.001

    def test_ms_to_kmh(self):
        assert abs(ms_to_kmh(10) - 36.0) < 0.001
        assert abs(ms_to_kmh(27.778) - 100.0) < 0.1

    def test_kmh_to_ms(self):
        assert abs(kmh_to_ms(100) - 27.778) < 0.001
        assert abs(kmh_to_ms(36) - 10.0) < 0.001

    def test_ms_to_fts(self):
        from chuk_mcp_physics.units import ms_to_fts, fts_to_ms

        assert abs(ms_to_fts(10) - 32.8084) < 0.001
        assert abs(fts_to_ms(32.8084) - 10) < 0.001

    def test_ms_to_knots(self):
        from chuk_mcp_physics.units import ms_to_knots, knots_to_ms

        assert abs(ms_to_knots(10) - 19.4384) < 0.001
        assert abs(knots_to_ms(19.4384) - 10) < 0.001


class TestDistanceConversions:
    """Test distance unit conversions."""

    def test_m_to_km(self):
        assert m_to_km(1000) == 1.0
        assert m_to_km(5000) == 5.0

    def test_km_to_m(self):
        assert km_to_m(1) == 1000
        assert km_to_m(2.5) == 2500

    def test_m_to_mi(self):
        assert abs(m_to_mi(1609.34) - 1.0) < 0.001
        assert abs(m_to_mi(5000) - 3.107) < 0.001

    def test_mi_to_m(self):
        assert abs(mi_to_m(1) - 1609.34) < 0.1
        assert abs(mi_to_m(10) - 16093.4) < 0.1

    def test_m_to_ft(self):
        assert abs(m_to_ft(1) - 3.28084) < 0.001
        assert abs(m_to_ft(10) - 32.8084) < 0.001

    def test_ft_to_m(self):
        assert abs(ft_to_m(3.28084) - 1.0) < 0.001
        assert abs(ft_to_m(100) - 30.48) < 0.01

    def test_m_to_yd(self):
        from chuk_mcp_physics.units import m_to_yd, yd_to_m

        assert abs(m_to_yd(1) - 1.09361) < 0.001
        assert abs(yd_to_m(1.09361) - 1.0) < 0.001

    def test_m_to_in(self):
        from chuk_mcp_physics.units import m_to_in, in_to_m

        assert abs(m_to_in(1) - 39.3701) < 0.001
        assert abs(in_to_m(39.3701) - 1.0) < 0.001


class TestMassConversions:
    """Test mass unit conversions."""

    def test_kg_to_lb(self):
        assert abs(kg_to_lb(1) - 2.20462) < 0.001
        assert abs(kg_to_lb(10) - 22.0462) < 0.001

    def test_lb_to_kg(self):
        assert abs(lb_to_kg(2.20462) - 1.0) < 0.001
        assert abs(lb_to_kg(100) - 45.359) < 0.001

    def test_kg_to_g(self):
        assert kg_to_g(1) == 1000
        assert kg_to_g(0.5) == 500

    def test_g_to_kg(self):
        assert g_to_kg(1000) == 1.0
        assert g_to_kg(500) == 0.5

    def test_kg_to_oz(self):
        from chuk_mcp_physics.units import kg_to_oz, oz_to_kg

        assert abs(kg_to_oz(1) - 35.274) < 0.001
        assert abs(oz_to_kg(35.274) - 1.0) < 0.001

    def test_indirect_mass_conversion(self):
        # g → lb via kg
        result = convert_units(1000, "g", "lb")
        assert abs(result - 2.20462) < 0.001
        # oz → lb via kg
        result2 = convert_units(16, "oz", "lb")
        assert abs(result2 - 1.0) < 0.01


class TestTemperatureConversions:
    """Test temperature unit conversions."""

    def test_c_to_f(self):
        assert c_to_f(0) == 32
        assert c_to_f(100) == 212
        assert abs(c_to_f(37) - 98.6) < 0.1

    def test_f_to_c(self):
        assert f_to_c(32) == 0
        assert f_to_c(212) == 100
        assert abs(f_to_c(98.6) - 37) < 0.1

    def test_c_to_k(self):
        assert c_to_k(0) == 273.15
        assert c_to_k(-273.15) == 0
        assert c_to_k(100) == 373.15

    def test_k_to_c(self):
        assert k_to_c(273.15) == 0
        assert k_to_c(0) == -273.15
        assert k_to_c(373.15) == 100

    def test_f_to_k(self):
        from chuk_mcp_physics.units import f_to_k

        assert abs(f_to_k(32) - 273.15) < 0.01
        assert abs(f_to_k(212) - 373.15) < 0.01

    def test_k_to_f(self):
        from chuk_mcp_physics.units import k_to_f

        assert abs(k_to_f(273.15) - 32) < 0.01
        assert abs(k_to_f(373.15) - 212) < 0.01

    def test_indirect_temperature_conversion(self):
        # C → F via K (indirect path)
        result = convert_units(0, "C", "F")
        assert abs(result - 32) < 0.01
        # F → C via K (indirect path)
        result2 = convert_units(212, "F", "C")
        assert abs(result2 - 100) < 0.01


class TestAngleConversions:
    """Test angle unit conversions."""

    def test_deg_to_rad(self):
        assert abs(deg_to_rad(0) - 0) < 0.001
        assert abs(deg_to_rad(180) - math.pi) < 0.001
        assert abs(deg_to_rad(90) - math.pi / 2) < 0.001
        assert abs(deg_to_rad(360) - 2 * math.pi) < 0.001

    def test_rad_to_deg(self):
        assert abs(rad_to_deg(0) - 0) < 0.001
        assert abs(rad_to_deg(math.pi) - 180) < 0.001
        assert abs(rad_to_deg(math.pi / 2) - 90) < 0.001
        assert abs(rad_to_deg(2 * math.pi) - 360) < 0.001

    def test_roundtrip_angle(self):
        original = 45.0
        rad = deg_to_rad(original)
        back = rad_to_deg(rad)
        assert abs(original - back) < 0.001


class TestConvertUnits:
    """Test the main convert_units function."""

    def test_identity_conversion(self):
        assert convert_units(100, "m/s", "m/s") == 100
        assert convert_units(50, "kg", "kg") == 50

    def test_direct_conversion(self):
        result = convert_units(100, "m/s", "mph")
        assert abs(result - 223.694) < 0.001

    def test_indirect_conversion(self):
        # mph -> km/h via m/s
        result = convert_units(60, "mph", "km/h")
        expected = ms_to_kmh(mph_to_ms(60))
        assert abs(result - expected) < 0.001

    def test_unsupported_conversion(self):
        with pytest.raises(ValueError, match="not supported"):
            convert_units(100, "kg", "m/s")  # Can't convert mass to velocity

    def test_common_conversions(self):
        # Test common real-world conversions

        # Speed limit: 60 mph
        assert abs(convert_units(60, "mph", "km/h") - 96.56) < 0.1

        # Marathon: 42.195 km
        assert abs(convert_units(42.195, "km", "mi") - 26.219) < 0.001

        # Body weight: 70 kg
        assert abs(convert_units(70, "kg", "lb") - 154.32) < 0.1

        # Room temperature: 20 C
        assert abs(convert_units(20, "C", "F") - 68) < 0.1


class TestGetSupportedUnits:
    """Test getting supported units list."""

    def test_returns_dict(self):
        units = get_supported_units()
        assert isinstance(units, dict)

    def test_has_expected_categories(self):
        units = get_supported_units()
        expected_categories = [
            "velocity",
            "distance",
            "mass",
            "force",
            "energy",
            "power",
            "temperature",
            "angle",
            "pressure",
            "area",
            "volume",
        ]
        for category in expected_categories:
            assert category in units

    def test_velocity_units(self):
        units = get_supported_units()
        assert "m/s" in units["velocity"]
        assert "mph" in units["velocity"]
        assert "km/h" in units["velocity"]

    def test_distance_units(self):
        units = get_supported_units()
        assert "m" in units["distance"]
        assert "km" in units["distance"]
        assert "mi" in units["distance"]
        assert "ft" in units["distance"]


class TestTimeConversions:
    """Test time unit conversions."""

    def test_seconds_to_minutes(self):
        from chuk_mcp_physics.units import s_to_min, min_to_s

        assert s_to_min(60) == 1.0
        assert s_to_min(120) == 2.0
        assert min_to_s(1) == 60

    def test_seconds_to_hours(self):
        from chuk_mcp_physics.units import s_to_hr, hr_to_s

        assert s_to_hr(3600) == 1.0
        assert hr_to_s(1) == 3600
        assert hr_to_s(2) == 7200

    def test_seconds_to_days(self):
        from chuk_mcp_physics.units import s_to_day, day_to_s

        assert s_to_day(86400) == 1.0
        assert day_to_s(1) == 86400

    def test_minutes_to_hours(self):
        from chuk_mcp_physics.units import min_to_hr, hr_to_min

        assert min_to_hr(60) == 1.0
        assert hr_to_min(1) == 60

    def test_hours_to_days(self):
        from chuk_mcp_physics.units import hr_to_day, day_to_hr

        assert hr_to_day(24) == 1.0
        assert day_to_hr(1) == 24
        assert day_to_hr(2) == 48

    def test_indirect_time_conversion(self):
        # min → hr via seconds
        result = convert_units(120, "min", "hr")
        assert abs(result - 2.0) < 0.001
        # hr → day via seconds
        result2 = convert_units(48, "hr", "day")
        assert abs(result2 - 2.0) < 0.001
        # min → day via seconds
        result3 = convert_units(1440, "min", "day")
        assert abs(result3 - 1.0) < 0.001


class TestAccelerationConversions:
    """Test acceleration unit conversions."""

    def test_ms2_to_g(self):
        from chuk_mcp_physics.units import ms2_to_g, g_to_ms2

        assert abs(ms2_to_g(9.80665) - 1.0) < 0.001
        assert abs(ms2_to_g(19.6133) - 2.0) < 0.001
        assert abs(g_to_ms2(1) - 9.80665) < 0.001
        assert abs(g_to_ms2(2) - 19.6133) < 0.001

    def test_ms2_to_fts2(self):
        from chuk_mcp_physics.units import ms2_to_fts2, fts2_to_ms2

        assert abs(ms2_to_fts2(10) - 32.8084) < 0.001
        assert abs(fts2_to_ms2(32.8084) - 10) < 0.001

    def test_roundtrip_acceleration(self):
        original = 5.0  # g-force
        from chuk_mcp_physics.units import g_to_ms2, ms2_to_g

        ms2 = g_to_ms2(original)
        back = ms2_to_g(ms2)
        assert abs(original - back) < 0.001

    def test_indirect_acceleration_conversion(self):
        # g → ft/s² via m/s²
        result = convert_units(1, "g", "ft/s²")
        assert abs(result - 32.174) < 0.01
        # ft/s² → g via m/s²
        result2 = convert_units(32.174, "ft/s²", "g")
        assert abs(result2 - 1.0) < 0.01


class TestTorqueConversions:
    """Test torque unit conversions."""

    def test_nm_to_lbft(self):
        from chuk_mcp_physics.units import nm_to_lbft

        assert abs(nm_to_lbft(100) - 73.7562) < 0.001

    def test_lbft_to_nm(self):
        from chuk_mcp_physics.units import lbft_to_nm

        assert abs(lbft_to_nm(73.7562) - 100) < 0.001

    def test_nm_to_lbin(self):
        from chuk_mcp_physics.units import nm_to_lbin, lbin_to_nm

        assert abs(nm_to_lbin(10) - 88.507) < 0.01
        assert abs(lbin_to_nm(88.507) - 10) < 0.01

    def test_indirect_torque_conversion(self):
        # lb·ft → lb·in via N·m
        result = convert_units(1, "lb·ft", "lb·in")
        assert abs(result - 12.0) < 0.1  # 1 ft = 12 in


class TestFrequencyConversions:
    """Test frequency unit conversions."""

    def test_hz_to_khz(self):
        from chuk_mcp_physics.units import hz_to_khz, khz_to_hz

        assert hz_to_khz(1000) == 1.0
        assert hz_to_khz(5000) == 5.0
        assert khz_to_hz(1) == 1000

    def test_hz_to_mhz(self):
        from chuk_mcp_physics.units import hz_to_mhz, mhz_to_hz

        assert hz_to_mhz(1000000) == 1.0
        assert mhz_to_hz(1) == 1000000

    def test_hz_to_ghz(self):
        from chuk_mcp_physics.units import hz_to_ghz, ghz_to_hz

        assert hz_to_ghz(1000000000) == 1.0
        assert ghz_to_hz(1) == 1000000000

    def test_khz_to_mhz(self):
        from chuk_mcp_physics.units import khz_to_mhz, mhz_to_khz

        assert khz_to_mhz(1000) == 1.0
        assert mhz_to_khz(1) == 1000

    def test_mhz_to_ghz(self):
        from chuk_mcp_physics.units import mhz_to_ghz, ghz_to_mhz

        assert mhz_to_ghz(1000) == 1.0
        assert ghz_to_mhz(1) == 1000

    def test_indirect_frequency_conversion(self):
        # kHz → MHz via Hz
        result = convert_units(1000, "kHz", "MHz")
        assert abs(result - 1.0) < 0.001
        # MHz → GHz via Hz
        result2 = convert_units(1000, "MHz", "GHz")
        assert abs(result2 - 1.0) < 0.001
        # kHz → GHz via Hz
        result3 = convert_units(1000000, "kHz", "GHz")
        assert abs(result3 - 1.0) < 0.001


class TestDataSizeConversions:
    """Test data size unit conversions."""

    def test_b_to_kb(self):
        from chuk_mcp_physics.units import b_to_kb, kb_to_b

        assert b_to_kb(1024) == 1.0
        assert b_to_kb(2048) == 2.0
        assert kb_to_b(1) == 1024

    def test_b_to_mb(self):
        from chuk_mcp_physics.units import b_to_mb, mb_to_b

        assert b_to_mb(1024 * 1024) == 1.0
        assert mb_to_b(1) == 1024 * 1024

    def test_b_to_gb(self):
        from chuk_mcp_physics.units import b_to_gb, gb_to_b

        assert b_to_gb(1024 * 1024 * 1024) == 1.0
        assert gb_to_b(1) == 1024 * 1024 * 1024

    def test_kb_to_mb(self):
        from chuk_mcp_physics.units import kb_to_mb, mb_to_kb

        assert kb_to_mb(1024) == 1.0
        assert mb_to_kb(1) == 1024

    def test_mb_to_gb(self):
        from chuk_mcp_physics.units import mb_to_gb, gb_to_mb

        assert mb_to_gb(1024) == 1.0
        assert gb_to_mb(1) == 1024

    def test_indirect_data_conversion(self):
        # KB → GB via bytes
        result = convert_units(1024 * 1024, "KB", "GB")
        assert abs(result - 1.0) < 0.001


class TestRealWorldScenarios:
    """Test real-world physics scenarios with unit conversions."""

    def test_car_speed_conversion(self):
        # Car traveling at 100 km/h
        ms = convert_units(100, "km/h", "m/s")
        assert abs(ms - 27.778) < 0.001

        mph = convert_units(100, "km/h", "mph")
        assert abs(mph - 62.137) < 0.001

    def test_projectile_height_conversion(self):
        # Ball thrown to 10 meters
        feet = convert_units(10, "m", "ft")
        assert abs(feet - 32.8084) < 0.001

    def test_force_conversion(self):
        # 100 N force
        from chuk_mcp_physics.units import n_to_lbf

        lbf = n_to_lbf(100)
        assert abs(lbf - 22.48) < 0.01

    def test_energy_conversion(self):
        # 1000 J of kinetic energy
        from chuk_mcp_physics.units import j_to_kj, j_to_cal

        kj = j_to_kj(1000)
        assert kj == 1.0

        cal = j_to_cal(1000)
        assert abs(cal - 239.006) < 0.001

    def test_pressure_conversion(self):
        # Atmospheric pressure
        from chuk_mcp_physics.units import pa_to_psi, pa_to_atm

        psi = pa_to_psi(101325)
        assert abs(psi - 14.696) < 0.01

        atm = pa_to_atm(101325)
        assert abs(atm - 1.0) < 0.001

    def test_acceleration_g_force(self):
        # Rocket acceleration
        from chuk_mcp_physics.units import ms2_to_g

        g_force = ms2_to_g(29.42)  # ~3g
        assert abs(g_force - 3.0) < 0.1

    def test_engine_torque(self):
        # Car engine torque
        from chuk_mcp_physics.units import nm_to_lbft

        lbft = nm_to_lbft(300)  # 300 N·m
        assert abs(lbft - 221.27) < 0.1

    def test_computation_time(self):
        # Algorithm runtime
        result = convert_units(3600, "s", "hr")
        assert abs(result - 1.0) < 0.001

    def test_cpu_frequency(self):
        # Processor speed
        from chuk_mcp_physics.units import mhz_to_ghz

        ghz = mhz_to_ghz(3500)  # 3.5 GHz
        assert abs(ghz - 3.5) < 0.001


class TestForceConversions:
    """Test force unit conversions."""

    def test_n_to_kn(self):
        from chuk_mcp_physics.units import n_to_kn, kn_to_n

        assert n_to_kn(1000) == 1.0
        assert kn_to_n(1) == 1000

    def test_n_to_lbf(self):
        from chuk_mcp_physics.units import n_to_lbf, lbf_to_n

        assert abs(n_to_lbf(100) - 22.48) < 0.01
        assert abs(lbf_to_n(22.48) - 100) < 0.1


class TestEnergyConversions:
    """Test energy unit conversions."""

    def test_j_to_kj(self):
        from chuk_mcp_physics.units import j_to_kj, kj_to_j

        assert j_to_kj(1000) == 1.0
        assert kj_to_j(1) == 1000

    def test_j_to_cal(self):
        from chuk_mcp_physics.units import j_to_cal, cal_to_j

        assert abs(j_to_cal(1000) - 239.006) < 0.001
        assert abs(cal_to_j(239.006) - 1000) < 0.1

    def test_j_to_btu(self):
        from chuk_mcp_physics.units import j_to_btu, btu_to_j

        assert abs(j_to_btu(1055.06) - 1.0) < 0.01
        assert abs(btu_to_j(1) - 1055.06) < 0.1

    def test_j_to_kwh(self):
        from chuk_mcp_physics.units import j_to_kwh, kwh_to_j

        assert abs(j_to_kwh(3600000) - 1.0) < 0.001
        assert abs(kwh_to_j(1) - 3600000) < 1


class TestPowerConversions:
    """Test power unit conversions."""

    def test_w_to_kw(self):
        from chuk_mcp_physics.units import w_to_kw, kw_to_w

        assert w_to_kw(1000) == 1.0
        assert kw_to_w(1) == 1000

    def test_w_to_hp(self):
        from chuk_mcp_physics.units import w_to_hp, hp_to_w

        assert abs(w_to_hp(745.7) - 1.0) < 0.01
        assert abs(hp_to_w(1) - 745.7) < 0.1


class TestPressureConversions:
    """Test pressure unit conversions."""

    def test_pa_to_kpa(self):
        from chuk_mcp_physics.units import pa_to_kpa, kpa_to_pa

        assert pa_to_kpa(1000) == 1.0
        assert kpa_to_pa(1) == 1000

    def test_pa_to_bar(self):
        from chuk_mcp_physics.units import pa_to_bar, bar_to_pa

        assert abs(pa_to_bar(100000) - 1.0) < 0.001
        assert abs(bar_to_pa(1) - 100000) < 1

    def test_pa_to_psi(self):
        from chuk_mcp_physics.units import pa_to_psi, psi_to_pa

        assert abs(pa_to_psi(101325) - 14.696) < 0.01
        assert abs(psi_to_pa(14.696) - 101325) < 10

    def test_pa_to_atm(self):
        from chuk_mcp_physics.units import pa_to_atm, atm_to_pa

        assert abs(pa_to_atm(101325) - 1.0) < 0.001
        assert abs(atm_to_pa(1) - 101325) < 1


class TestAreaConversions:
    """Test area unit conversions."""

    def test_m2_to_km2(self):
        from chuk_mcp_physics.units import m2_to_km2, km2_to_m2

        assert m2_to_km2(1000000) == 1.0
        assert km2_to_m2(1) == 1000000

    def test_m2_to_ft2(self):
        from chuk_mcp_physics.units import m2_to_ft2, ft2_to_m2

        assert abs(m2_to_ft2(1) - 10.7639) < 0.001
        assert abs(ft2_to_m2(10.7639) - 1.0) < 0.001

    def test_m2_to_acre(self):
        from chuk_mcp_physics.units import m2_to_acre, acre_to_m2

        assert abs(m2_to_acre(4046.86) - 1.0) < 0.01
        assert abs(acre_to_m2(1) - 4046.86) < 0.1


class TestVolumeConversions:
    """Test volume unit conversions."""

    def test_m3_to_l(self):
        from chuk_mcp_physics.units import m3_to_l, l_to_m3

        assert m3_to_l(1) == 1000
        assert l_to_m3(1000) == 1.0

    def test_m3_to_gal(self):
        from chuk_mcp_physics.units import m3_to_gal, gal_to_m3

        assert abs(m3_to_gal(1) - 264.172) < 0.001
        assert abs(gal_to_m3(264.172) - 1.0) < 0.001

    def test_m3_to_ft3(self):
        from chuk_mcp_physics.units import m3_to_ft3, ft3_to_m3

        assert abs(m3_to_ft3(1) - 35.3147) < 0.001
        assert abs(ft3_to_m3(35.3147) - 1.0) < 0.001


class TestMCPTools:
    """Test the MCP tool interface functions."""

    def test_convert_unit_success(self):
        from chuk_mcp_physics.tools.convert_units import convert_unit

        result = convert_unit(100, "m/s", "mph")
        assert result["original_value"] == 100
        assert result["original_unit"] == "m/s"
        assert abs(result["converted_value"] - 223.694) < 0.001
        assert result["converted_unit"] == "mph"
        assert "formatted" in result
        assert "100 m/s" in result["formatted"]
        assert "mph" in result["formatted"]

    def test_convert_unit_error(self):
        from chuk_mcp_physics.tools.convert_units import convert_unit

        # Test invalid conversion
        result = convert_unit(100, "kg", "Hz")
        assert "error" in result
        assert "supported_units" in result

    def test_list_unit_conversions(self):
        from chuk_mcp_physics.tools.convert_units import list_unit_conversions

        result = list_unit_conversions()
        assert "categories" in result
        assert "examples" in result
        assert "total_conversions" in result

        # Check that all expected categories are present
        categories = result["categories"]
        assert "velocity" in categories
        assert "distance" in categories
        assert "mass" in categories
        assert "force" in categories
        assert "energy" in categories
        assert "power" in categories
        assert "temperature" in categories
        assert "angle" in categories
        assert "pressure" in categories
        assert "area" in categories
        assert "volume" in categories
        assert "time" in categories
        assert "acceleration" in categories
        assert "torque" in categories
        assert "frequency" in categories
        assert "data_size" in categories

        # Verify total count
        assert result["total_conversions"] == 62
