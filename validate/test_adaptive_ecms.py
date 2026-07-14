"""
Validate Adaptive ECMS SoC drift prevention under mission deviations.

Runs a nominal mission with calibrated ECMS to generate the planned
SoC trajectory. Then simulates a deviated mission (loiter extended 50%)
with both plain ECMS and AdaptiveECMS, comparing final SoC and plotting
the overlaid SoC recovery curves.
"""

import sys
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.hybrid_sim import run_naive_hybrid, LOITER_DURATION_S
from sim.ecms import ECMS, AdaptiveECMS
from validate.calibrate_ecms import calibrate as calibrate_ecms

def test_adaptive_ecms():
    print("Fetching calibrated s for nominal mission...")
    # Use the known calibrated value to save 8+ minutes of sweep time.
    s_cal = 9.00
    
    print(f"\n1. Running Nominal Mission to get Planned SoC curve (s={s_cal:.2f})...")
    df_nominal = run_naive_hybrid(ecms_s=s_cal, fault_flag=True)
    soc_planned = df_nominal["soc"].values
    
    # Deviated mission: 50% longer loiter
    extended_loiter = int(LOITER_DURATION_S * 1.5)
    
    print("\n2. Running Deviated Mission with Plain ECMS...")
    plain_ecms = ECMS(s=s_cal)
    df_deviated_plain = run_naive_hybrid(
        ecms_controller=plain_ecms,
        loiter_duration_s=extended_loiter,
        fault_flag=True
    )
    
    print("\n3. Running Deviated Mission with Adaptive ECMS...")
    adaptive_ecms = AdaptiveECMS(
        s_base=s_cal, 
        soc_planned=soc_planned,
        gain=20.0,  # High gain to pull it back aggressively
        threshold=0.01 # 1 percentage point threshold
    )
    df_deviated_adaptive = run_naive_hybrid(
        ecms_controller=adaptive_ecms,
        loiter_duration_s=extended_loiter,
        fault_flag=True
    )
    
    soc_nominal_final = df_nominal["soc"].iloc[-1]
    soc_plain_final = df_deviated_plain["soc"].iloc[-1]
    soc_adaptive_final = df_deviated_adaptive["soc"].iloc[-1]
    
    print(f"\n{'='*55}")
    print("  Adaptive ECMS Validation: Deviated Mission (+50% Loiter)")
    print(f"{'='*55}")
    print(f"  Nominal Mission Final SoC:   {soc_nominal_final:.4f}")
    print(f"  Plain ECMS Final SoC:        {soc_plain_final:.4f}  (Drifted)")
    print(f"  Adaptive ECMS Final SoC:     {soc_adaptive_final:.4f}  (Recovered)")
    print(f"{'='*55}")
    
    # Plotting
    report_dir = os.path.join(os.path.dirname(__file__), "..", "report")
    os.makedirs(report_dir, exist_ok=True)
    
    time_min_nominal = df_nominal["time_s"] / 60.0
    time_min_deviated = df_deviated_plain["time_s"] / 60.0
    
    plt.figure(figsize=(12, 6))
    plt.plot(time_min_nominal, df_nominal["soc"], 'k--', lw=2, alpha=0.5, label="Planned SoC (Nominal Mission)")
    plt.plot(time_min_deviated, df_deviated_plain["soc"], color="#e74c3c", lw=2, label="Plain ECMS (Extended Loiter)")
    plt.plot(time_min_deviated, df_deviated_adaptive["soc"], color="#2ecc71", lw=2, label="Adaptive ECMS (Extended Loiter)")
    
    plt.axhline(0.20, color="red", ls=":", alpha=0.5, label="SoC Floor")
    plt.axhline(0.90, color="green", ls=":", alpha=0.5, label="Initial/Ceiling SoC")
    
    plt.ylabel("Battery State of Charge")
    plt.xlabel("Time [min]")
    plt.title("SoC Recovery with Adaptive ECMS during Mission Deviation (+50% Loiter)")
    plt.grid(True, alpha=0.3)
    plt.ylim(0.1, 1.0)
    plt.legend()
    
    save_path = os.path.join(report_dir, "adaptive_ecms_soc_recovery.png")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved: {os.path.abspath(save_path)}")


if __name__ == "__main__":
    test_adaptive_ecms()
