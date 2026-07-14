"""Tests for Battery and ElectricMotor components."""

import sys
import os
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.battery import Battery
from sim.motor import ElectricMotor


# ================================================================
# Battery tests
# ================================================================

def test_soc_discharge_hand_calc():
    """
    Constant discharge hand-calculation check (within 1%).

    Setup
    -----
    Battery : 10 kWh, eta_rt = 0.95, SoC starts at 0.90
    Load    : 10 kW discharge for 100 s  (100 one-second steps)

    Hand calculation
    ----------------
    Energy delivered  = 10 000 W  x  100 s / 3600  =  277.78 Wh
    eta_discharge     = sqrt(0.95)  =  0.97468
    Internal drain    = 277.78 / 0.97468  =  285.02 Wh
    delta_SoC         = 285.02 / 10 000   =  0.028502
    Expected final SoC = 0.90 - 0.028502  =  0.871498
    """
    bat = Battery(capacity_kWh=10.0)

    # Run 100 one-second steps at 10 kW discharge
    for _ in range(100):
        bat.step(10_000.0, dt_s=1.0)

    # Hand-calc expected
    eta_d = math.sqrt(0.95)
    energy_delivered_Wh = 10_000 * 100 / 3600
    internal_drain_Wh = energy_delivered_Wh / eta_d
    expected_soc = 0.90 - internal_drain_Wh / 10_000

    error = abs(bat.soc - expected_soc) / expected_soc
    print(f"\n  Battery discharge test (10 kW x 100 s):")
    print(f"    Expected SoC = {expected_soc:.6f}")
    print(f"    Actual   SoC = {bat.soc:.6f}")
    print(f"    Error        = {error*100:.4f} %")

    assert error < 0.01, (
        f"SoC error {error*100:.2f}% exceeds 1% tolerance: "
        f"expected {expected_soc:.6f}, got {bat.soc:.6f}"
    )


def test_soc_never_below_min():
    """
    Discharge until battery is empty; SoC must never drop
    below soc_min (0.20).

    A 5 kWh battery at 4C can discharge at 20 kW.
    Usable energy = (0.90 - 0.20) x 5000 = 3500 Wh.
    At 20 kW that lasts 3500/20000*3600 = 630 s.
    We run for 1000 s (well past empty).
    """
    bat = Battery(capacity_kWh=5.0)
    min_soc_seen = bat.soc

    for _ in range(1000):
        bat.step(20_000.0, dt_s=1.0)
        min_soc_seen = min(min_soc_seen, bat.soc)

    print(f"\n  Battery SoC-floor test (5 kWh, 20 kW discharge, 1000 s):")
    print(f"    Final SoC    = {bat.soc:.6f}")
    print(f"    Min SoC seen = {min_soc_seen:.6f}")
    print(f"    soc_min      = {bat.soc_min}")

    assert bat.soc >= bat.soc_min, (
        f"SoC {bat.soc:.6f} fell below soc_min {bat.soc_min}"
    )
    assert min_soc_seen >= bat.soc_min, (
        f"SoC reached {min_soc_seen:.6f} during run, below soc_min {bat.soc_min}"
    )


def test_soc_never_above_max():
    """
    Charge a depleted battery; SoC must never exceed soc_max (0.90).
    """
    bat = Battery(capacity_kWh=5.0)
    bat.soc = bat.soc_min  # start empty

    for _ in range(2000):
        bat.step(-10_000.0, dt_s=1.0)  # 10 kW charge

    print(f"\n  Battery SoC-ceiling test (5 kWh, 10 kW charge, 2000 s):")
    print(f"    Final SoC    = {bat.soc:.6f}")
    print(f"    soc_max      = {bat.soc_max}")

    assert bat.soc <= bat.soc_max, (
        f"SoC {bat.soc:.6f} exceeded soc_max {bat.soc_max}"
    )


def test_battery_mass():
    """Mass = capacity_Wh / specific_energy."""
    bat = Battery(capacity_kWh=10.0, specific_energy_Wh_per_kg=220.0)
    expected = 10_000 / 220.0  # 45.45 kg
    assert abs(bat.mass_kg - expected) < 0.01, (
        f"Battery mass {bat.mass_kg:.2f} != expected {expected:.2f}"
    )
    print(f"\n  Battery mass: {bat.mass_kg:.2f} kg  (10 kWh / 220 Wh/kg)")


# ================================================================
# Motor tests
# ================================================================

def test_motor_mass_scales():
    """
    Motor mass = rated_power_kW / power_density_kW_per_kg.
    50 kW @ 5 kW/kg = 10 kg,  100 kW = 20 kg.
    """
    m1 = ElectricMotor(rated_power_W=50_000)
    m2 = ElectricMotor(rated_power_W=100_000)

    print(f"\n  Motor mass scaling:")
    print(f"    50 kW  motor: {m1.mass_kg:.1f} kg  (expected 10.0)")
    print(f"    100 kW motor: {m2.mass_kg:.1f} kg  (expected 20.0)")

    assert abs(m1.mass_kg - 10.0) < 0.01
    assert abs(m2.mass_kg - 20.0) < 0.01
    assert abs(m2.mass_kg / m1.mass_kg - 2.0) < 0.01, "Mass should scale linearly"


def test_motor_electrical_power():
    """
    P_elec = P_mech / eta.
    At 94% efficiency, 50 kW mechanical needs 53.19 kW electrical.
    """
    motor = ElectricMotor(rated_power_W=50_000, efficiency=0.94)
    P_elec = motor.electrical_power_in(50_000.0)
    expected = 50_000 / 0.94

    print(f"\n  Motor efficiency test:")
    print(f"    P_mech = 50.0 kW -> P_elec = {P_elec/1000:.2f} kW")
    print(f"    Expected: {expected/1000:.2f} kW  (50 / 0.94)")

    assert abs(P_elec - expected) < 1.0, (
        f"P_elec {P_elec:.1f} != expected {expected:.1f}"
    )
