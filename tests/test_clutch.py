"""Tests for Clutch and clutch-integrated hybrid simulation."""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.clutch import Clutch
from sim.hybrid_sim import run_naive_hybrid


def test_clutch_mass():
    """Mass = 2 kg + 0.02 * engine_rated_kW."""
    c = Clutch(engine_rated_kW=60.0)
    expected = 2.0 + 0.02 * 60.0   # 3.2 kg
    assert abs(c.mass_kg - expected) < 0.01, (
        f"Clutch mass {c.mass_kg:.2f} != expected {expected:.2f}"
    )
    print(f"\n  Clutch mass: {c.mass_kg:.2f} kg  (2 + 0.02*60)")


def test_parallel_in_efficient_zone():
    """
    Verify parallel is selected when power falls in the
    engine's best-BSFC band  [0.68, 0.92] of 60 kW.
    """
    c = Clutch(engine_rated_kW=60.0)

    # Cruise ~53 kW  ->  pf = 0.887  (in band)
    assert c.select_mode(53_000, 60_000) == "parallel"
    # Loiter ~42 kW  ->  pf = 0.70   (in band)
    assert c.select_mode(42_000, 60_000) == "parallel"
    # Climb ~140 kW  ->  pf = 2.33   (out of band)
    assert c.select_mode(140_000, 60_000) == "series"
    # Negative power  ->  pf < 0     (out of band)
    assert c.select_mode(-5_000, 60_000) == "series"

    print("  Mode selection: cruise->parallel, loiter->parallel, "
          "climb->series, negative->series")


def test_fault_flag_always_series():
    """
    With fault_flag=True the clutch must NEVER select 'parallel',
    regardless of power level.  This is the fail-safe default.
    """
    c = Clutch(engine_rated_kW=60.0)

    # Even at the optimal power point, fault overrides
    assert c.select_mode(48_000, 60_000, fault_flag=True) == "series"

    # Full simulation with fault flag
    df = run_naive_hybrid(fault_flag=True)
    n_parallel = (df["mode"] == "parallel").sum()

    print(f"\n  Fault-flag simulation:")
    print(f"    Total timesteps : {len(df)}")
    print(f"    Parallel steps  : {n_parallel}")
    print(f"    SoC min / max   : {df['soc'].min():.4f} / {df['soc'].max():.4f}")

    assert n_parallel == 0, (
        f"fault_flag=True but {n_parallel} timesteps selected 'parallel'"
    )
    assert df["soc"].min() >= 0.20, "SoC violated floor"
    assert df["soc"].max() <= 0.90, "SoC violated ceiling"
    print("  [PASS] fault_flag=True -> all series, SoC bounds OK")
