"""
Compare ECMS (heuristic clutch) against offline PMP (optimal sequence).

Runs the final ECMS simulation (with its calibrated s) and the PMP
simulation (with its calibrated lambda), calculates the fuel burn gap,
and generates an overlay plot of the engine power trajectories.
"""

import sys
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.hybrid_sim import run_naive_hybrid
from sim.pmp import calibrate_pmp, run_pmp_sim
from validate.calibrate_ecms import calibrate as calibrate_ecms

def compare():
    print("Running ECMS calibration...")
    ecms_results = calibrate_ecms()
    s_cal = ecms_results["s_calibrated"]
    
    print("\nRunning PMP calibration...")
    lam_cal = calibrate_pmp()
    
    print("\nSimulating both strategies...")
    df_ecms = run_naive_hybrid(ecms_s=s_cal)
    df_pmp = run_pmp_sim(lam_cal)
    
    fuel_ecms = df_ecms["fuel_burned_kg"].iloc[-1]
    fuel_pmp = df_pmp["fuel_burned_kg"].iloc[-1]
    
    gap = (fuel_ecms - fuel_pmp) / fuel_pmp * 100
    
    print(f"\n{'='*55}")
    print("  ECMS vs PMP Validation")
    print(f"{'='*55}")
    print(f"  ECMS Fuel Burned (s={s_cal:.2f}): {fuel_ecms:7.2f} kg")
    print(f"  PMP Fuel Burned (lam={lam_cal:.2f}): {fuel_pmp:7.2f} kg")
    print(f"  Gap (ECMS vs PMP): {gap:+7.2f}%")
    print(f"{'='*55}")
    
    # Generate plot
    report_dir = os.path.join(os.path.dirname(__file__), "..", "report")
    os.makedirs(report_dir, exist_ok=True)
    
    time_min = df_ecms["time_s"] / 60.0
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    ax1 = axes[0]
    ax1.plot(time_min, df_ecms["p_eng_W"] / 1000.0, lw=2, alpha=0.8, color="#e74c3c", label=f"ECMS Engine (s={s_cal:.2f})")
    ax1.plot(time_min, df_pmp["p_eng_W"] / 1000.0, lw=2, alpha=0.8, color="#2980b9", label=f"PMP Engine (lam={lam_cal:.2f})")
    ax1.set_ylabel("Engine Power [kW]")
    ax1.set_title("ECMS vs PMP Power Split Trajectories")
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    ax2 = axes[1]
    ax2.plot(time_min, df_ecms["soc"], lw=2, alpha=0.8, color="#e74c3c", label="ECMS SoC")
    ax2.plot(time_min, df_pmp["soc"], lw=2, alpha=0.8, color="#2980b9", label="PMP SoC")
    ax2.axhline(0.90, color="green", ls="--", alpha=0.5, label="Initial SoC")
    ax2.set_ylabel("Battery State of Charge")
    ax2.set_xlabel("Time [min]")
    ax2.set_ylim(0.1, 1.0)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    fig.tight_layout()
    save_path = os.path.join(report_dir, "ecms_vs_pmp.png")
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved: {os.path.abspath(save_path)}")

if __name__ == "__main__":
    compare()
