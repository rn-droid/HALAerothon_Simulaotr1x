"""Unit tests for sim.aircraft.Aircraft."""

import math
import sys
import os

# Ensure the project root is importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.aircraft import Aircraft


# ── Helpers ──────────────────────────────────────────────────
G = 9.80665
AC = Aircraft()          # default 1000 kg UAV
W = AC.mtow_kg * G       # weight in newtons ≈ 9806.65 N
V_CRUISE = 250 / 3.6     # 250 km/h → 69.44 m/s
ALT_CRUISE = 3000.0      # cruise altitude [m]


# ── Test 1: ISA density ─────────────────────────────────────
def test_sea_level_density():
    """ρ at sea level should be 1.225 kg/m³ (within 1%)."""
    rho0 = AC.air_density(0.0)
    assert abs(rho0 - 1.225) / 1.225 < 0.01, f"Sea-level ρ = {rho0:.4f}, expected ~1.225"


def test_density_decreases_with_altitude():
    """Density must strictly decrease as altitude increases."""
    altitudes = [0, 1000, 3000, 5000, 8000, 11000]
    densities = [AC.air_density(h) for h in altitudes]
    for i in range(1, len(densities)):
        assert densities[i] < densities[i - 1], (
            f"ρ did not decrease from {altitudes[i-1]}m to {altitudes[i]}m: "
            f"{densities[i-1]:.4f} → {densities[i]:.4f}"
        )


# ── Test 2: Cruise power sanity ─────────────────────────────
def test_cruise_power_reasonable():
    """
    Shaft power at 250 km/h, 3000 m, level flight should be
    a positive number in a physically credible range.

    Hand-calculation gives ≈ 53 kW for these parameters
    (breakdown printed below), which is consistent with
    the problem's ~60 kW turboshaft running at ~88 % rated.
    """
    P = AC.shaft_power_required(W, V_CRUISE, ALT_CRUISE, 0.0, "cruise")

    # Print the full breakdown so the user can verify
    rho = AC.air_density(ALT_CRUISE)
    CL = AC.required_CL(W, V_CRUISE, ALT_CRUISE)
    CD = AC.CD0 + AC.k * CL ** 2
    D = AC.drag_force(W, V_CRUISE, ALT_CRUISE)
    print(
        f"\n{'='*50}"
        f"\n  Cruise power breakdown (250 km/h, 3000 m):"
        f"\n    rho       = {rho:.4f} kg/m3"
        f"\n    V         = {V_CRUISE:.2f} m/s"
        f"\n    q         = {0.5*rho*V_CRUISE**2:.1f} Pa"
        f"\n    CL        = {CL:.4f}"
        f"\n    CD        = {CD:.5f}"
        f"\n    L/D       = {CL/CD:.1f}"
        f"\n    Drag      = {D:.1f} N"
        f"\n    eta_prop  = {AC.prop_eff['cruise']}"
        f"\n    P_shaft   = {P:.0f} W  ({P/1000:.1f} kW)"
        f"\n{'='*50}"
    )

    assert P > 0, "Power must be positive"
    # Physically: a 1000 kg UAV at 250 km/h at 3000 m should
    # need roughly 40-70 kW shaft power.  The range is generous
    # to serve as a sanity guard, not a precision test.
    assert 20_000 < P < 80_000, f"P_shaft = {P/1000:.1f} kW looks unreasonable"


# -- Test 3: Climbing requires more power --------------------
def test_climb_more_power_than_level():
    """
    Adding a positive climb angle (gamma > 0) adds W*sin(gamma) to
    thrust, so shaft power must increase vs level flight.
    """
    P_level = AC.shaft_power_required(W, V_CRUISE, ALT_CRUISE, 0.0, "cruise")

    # 5 m/s ROC at 69.4 m/s -> gamma = arcsin(5/69.4) ~ 4.13 deg
    gamma = math.asin(5.0 / V_CRUISE)
    P_climb = AC.shaft_power_required(W, V_CRUISE, ALT_CRUISE, gamma, "climb")

    print(
        f"\n  Climb comparison:"
        f"\n    gamma     = {math.degrees(gamma):.2f} deg"
        f"\n    P_level   = {P_level/1000:.1f} kW  (cruise eta={AC.prop_eff['cruise']})"
        f"\n    P_climb   = {P_climb/1000:.1f} kW  (climb  eta={AC.prop_eff['climb']})"
        f"\n    dP        = +{(P_climb-P_level)/1000:.1f} kW"
    )

    assert P_climb > P_level, (
        f"Climb power ({P_climb:.0f} W) should exceed level ({P_level:.0f} W)"
    )
