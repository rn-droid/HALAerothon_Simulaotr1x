"""
ECMS equivalence-factor calibration.

Sweeps the equivalence factor s to find the charge-sustaining
value: the s* that makes final SoC approximately equal to initial
SoC (within 1 %) over a full mission.

Also compares the ECMS-calibrated result against the fixed 70/30
split (Task 5) baseline.
"""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim.hybrid_sim import run_naive_hybrid


INITIAL_SOC = 0.90
SOC_TOL = 0.01          # 1 % of initial SoC  ->  0.009


def run_for_s(s: float) -> float:
    """Run the full mission with ECMS at equivalence factor s.
    Return final SoC."""
    df = run_naive_hybrid(ecms_s=s)
    return float(df["soc"].iloc[-1])


def calibrate() -> dict:
    """Find charge-sustaining s and compare against fixed split."""

    # ── 1. Coarse sweep ─────────────────────────────────────
    s_values = np.arange(1.0, 9.5, 0.5)
    soc_values = []

    print(f"\n{'='*55}")
    print("  ECMS Calibration  -  Coarse sweep")
    print(f"{'='*55}")
    print(f"  {'s':>6s}   {'final_SoC':>10s}   {'delta':>8s}")
    print(f"  {'-'*6}   {'-'*10}   {'-'*8}")

    for s in s_values:
        fsoc = run_for_s(s)
        soc_values.append(fsoc)
        delta = fsoc - INITIAL_SOC
        print(f"  {s:6.2f}   {fsoc:10.4f}   {delta:+8.4f}")

    soc_arr = np.array(soc_values)

    # ── 2. Find bracketing interval ─────────────────────────
    target = INITIAL_SOC - SOC_TOL * INITIAL_SOC  # 0.891
    s_lo = s_values[0]
    s_hi = s_values[-1]

    for i in range(len(soc_arr) - 1):
        if soc_arr[i] < target <= soc_arr[i + 1]:
            s_lo = s_values[i]
            s_hi = s_values[i + 1]
            break

    # ── 3. Binary-search refinement ─────────────────────────
    print(f"\n  Refining in [{s_lo:.2f}, {s_hi:.2f}] ...")
    for _ in range(15):
        if s_hi - s_lo < 0.005:
            break
        s_mid = (s_lo + s_hi) / 2.0
        fsoc = run_for_s(s_mid)
        if fsoc < target:
            s_lo = s_mid
        else:
            s_hi = s_mid

    s_cal = (s_lo + s_hi) / 2.0
    print(f"  Calibrated s = {s_cal:.3f}")

    # ── 4. Run calibrated and fixed-split baselines ─────────
    print(f"\n{'='*55}")
    print("  Running final simulations ...")
    df_ecms = run_naive_hybrid(ecms_s=s_cal)
    df_fixed = run_naive_hybrid(ecms_s=None)

    fuel_ecms = float(df_ecms["fuel_burned_kg"].iloc[-1])
    fuel_fixed = float(df_fixed["fuel_burned_kg"].iloc[-1])
    soc_ecms = float(df_ecms["soc"].iloc[-1])
    soc_fixed = float(df_fixed["soc"].iloc[-1])
    delta_fuel = (fuel_ecms - fuel_fixed) / fuel_fixed * 100

    print(f"\n{'='*55}")
    print("  ECMS vs Fixed 70/30 Split  —  Comparison")
    print(f"{'='*55}")
    print(f"  {'Metric':25s}  {'Fixed 70/30':>12s}  {'ECMS (s={:.2f})':>14s}".format(s_cal))
    print(f"  {'-'*25}  {'-'*12}  {'-'*14}")
    print(f"  {'Fuel burned [kg]':25s}  {fuel_fixed:12.2f}  {fuel_ecms:14.2f}")
    print(f"  {'Final SoC':25s}  {soc_fixed:12.4f}  {soc_ecms:14.4f}")
    print(f"  {'SoC delta (vs initial)':25s}  "
          f"{soc_fixed - INITIAL_SOC:+12.4f}  "
          f"{soc_ecms - INITIAL_SOC:+14.4f}")
    print(f"  {'Fuel change':25s}  {'(baseline)':>12s}  {delta_fuel:+13.1f}%")

    soc_ok = abs(soc_ecms - INITIAL_SOC) / INITIAL_SOC < SOC_TOL
    print(f"\n  Charge-sustaining: {'[PASS]' if soc_ok else '[FAIL]'}  "
          f"(|dSoC|/SoC0 = "
          f"{abs(soc_ecms - INITIAL_SOC)/INITIAL_SOC*100:.2f}%)")
    print(f"{'='*55}")

    # ── 5. Plots ────────────────────────────────────────────
    report_dir = os.path.join(os.path.dirname(__file__), "..", "report")
    os.makedirs(report_dir, exist_ok=True)
    time_min = df_ecms["time_s"] / 60.0

    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    # Top: SoC overlay
    ax1 = axes[0]
    ax1.plot(time_min, df_fixed["soc"], color="#e74c3c", lw=2,
             label=f"Fixed 70/30 (final {soc_fixed:.3f})")
    ax1.plot(time_min, df_ecms["soc"], color="#2980b9", lw=2,
             label=f"ECMS s={s_cal:.2f} (final {soc_ecms:.3f})")
    ax1.axhline(INITIAL_SOC, color="green", ls="--", alpha=0.5,
                label="Initial SoC (0.90)")
    ax1.axhline(0.20, color="red", ls="--", alpha=0.4, label="SoC Min")
    ax1.set_ylabel("State of Charge")
    ax1.set_title("ECMS Calibration  |  SoC Trace Comparison")
    ax1.set_ylim(0.1, 1.0)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower left", fontsize=10)

    # Bottom: cumulative fuel
    ax2 = axes[1]
    ax2.plot(time_min, df_fixed["fuel_burned_kg"], color="#e74c3c",
             lw=2, label=f"Fixed 70/30 ({fuel_fixed:.1f} kg)")
    ax2.plot(time_min, df_ecms["fuel_burned_kg"], color="#2980b9",
             lw=2, label=f"ECMS s={s_cal:.2f} ({fuel_ecms:.1f} kg)")
    ax2.set_ylabel("Cumulative Fuel [kg]")
    ax2.set_xlabel("Time [min]")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left", fontsize=10)

    fig.tight_layout()
    save_path = os.path.join(report_dir, "ecms_calibration.png")
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  Plot saved: {os.path.abspath(save_path)}")

    return {
        "s_calibrated": s_cal,
        "fuel_ecms_kg": fuel_ecms,
        "fuel_fixed_kg": fuel_fixed,
        "soc_ecms_final": soc_ecms,
        "soc_fixed_final": soc_fixed,
        "sweep_s": list(s_values),
        "sweep_soc": list(soc_arr),
    }


if __name__ == "__main__":
    calibrate()
