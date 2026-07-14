import sys
import os
import json
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.hybrid_sim import run_naive_hybrid
from sim.ecms import AdaptiveECMS
from sizing.sizing_equations import size_system
from validate.baseline_compare import compute_baseline_mass_consistent

# Ensure the report directory exists
REPORT_DIR = os.path.join(os.path.dirname(__file__), "..", "report")
os.makedirs(REPORT_DIR, exist_ok=True)

def validate_design(design_vector):
    """
    design_vector = [eng_kw, bat_kwh, sc_kwh, n_mot, mot_kw, ecms_s]
    """
    eng_kw, bat_kwh, sc_kwh, n_mot, mot_kw, ecms_s = design_vector
    
    print(f"Validating Best Design: {design_vector}")
    
    # 1. Baseline Endurance Comparison
    baseline_res = compute_baseline_mass_consistent()
    baseline_endurance_h = baseline_res["total_endurance_h"]
    
    mission = {"loiter_duration_s": 1200}
    
    # Run full hybrid with AdaptiveECMS and regen descent (which is the default in our sim now)
    res_hybrid = size_system(design_vector, mission, use_adaptive=True)
    
    # Calculate endurance for hybrid
    t_nom = 4600.0  # Nomimal mission length in seconds
    spare_mass = 1000.0 - res_hybrid["converged_mass_kg"]
    avg_fuel_flow = res_hybrid["fuel_burned_kg"] / t_nom
    
    if spare_mass > 0:
        hybrid_endurance_s = t_nom + (spare_mass / avg_fuel_flow)
    else:
        hybrid_endurance_s = t_nom
        
    hybrid_endurance_h = hybrid_endurance_s / 3600.0
    
    percent_improvement = ((hybrid_endurance_h - baseline_endurance_h) / baseline_endurance_h) * 100.0
    
    print(f"\n1. Endurance Comparison:")
    print(f"  Baseline Endurance: {baseline_endurance_h:.2f} h")
    print(f"  Hybrid Endurance  : {hybrid_endurance_h:.2f} h")
    print(f"  Improvement       : {percent_improvement:.2f}%")
    
    # 2. Literature Sanity Check
    print(f"\n2. Literature Sanity Check:")
    if 20.0 <= percent_improvement <= 25.0:
        print(f"  Result ({percent_improvement:.2f}%) falls INSIDE the literature benchmark range of 20-25%.")
    elif percent_improvement < 20.0:
        print(f"  Result ({percent_improvement:.2f}%) falls BELOW the literature benchmark range of 20-25%.")
    else:
        print(f"  Result ({percent_improvement:.2f}%) falls ABOVE the literature benchmark range of 20-25%.")
        
    # 3. Three-way fuel-savings breakdown
    print(f"\n3. Three-Way Fuel Savings Breakdown (Nominal 1200s Loiter Mission):")
    
    # (a) Baseline non-hybrid nominal fuel burn
    baseline_fuel_nominal = baseline_res["fuel_burned_nominal_kg"]
    
    # Setup hybrid system parameters for simulation
    mot_rated_W = mot_kw * 1000.0 * n_mot
    eng_rated_W = eng_kw * 1000.0
    sc_mass_kg = sc_kwh * 200.0
    mtow_kg = res_hybrid["converged_mass_kg"]
    
    # (b) Hybrid with constant-BSFC engine operation but NO power-split optimization
    # (Engine always at best-BSFC power, rest to battery, no ECMS)
    df_b = run_naive_hybrid(
        fault_flag=False,
        ecms_s=None,
        ecms_controller=None,
        loiter_duration_s=1200,
        mtow_kg=mtow_kg,
        engine_rated_W=eng_rated_W,
        battery_capacity_kWh=bat_kwh,
        supercap_mass_kg=sc_mass_kg,
        motor_rated_W=mot_rated_W,
        regen_efficiency=0.0  # No regen
    )
    fuel_b = df_b["fuel_burned_kg"].iloc[-1] if len(df_b) > 0 and df_b.attrs.get("feasible", True) else 1000.0
    
    # Get the planned SoC trajectory by running the plain ECMS once
    df_plan = run_naive_hybrid(
        fault_flag=False,
        ecms_s=ecms_s,
        ecms_controller=None,
        loiter_duration_s=1200,
        mtow_kg=mtow_kg,
        engine_rated_W=eng_rated_W,
        battery_capacity_kWh=bat_kwh,
        supercap_mass_kg=sc_mass_kg,
        motor_rated_W=mot_rated_W,
        regen_efficiency=0.0
    )
    soc_planned = df_plan["soc"].values
    
    # (c) Full hybrid with AdaptiveECMS power-split, NO regen
    controller_c = AdaptiveECMS(s_base=ecms_s, soc_planned=soc_planned, gain=2.0)
    df_c = run_naive_hybrid(
        fault_flag=False,
        ecms_s=None,
        ecms_controller=controller_c,
        loiter_duration_s=1200,
        mtow_kg=mtow_kg,
        engine_rated_W=eng_rated_W,
        battery_capacity_kWh=bat_kwh,
        supercap_mass_kg=sc_mass_kg,
        motor_rated_W=mot_rated_W,
        regen_efficiency=0.0
    )
    fuel_c = df_c["fuel_burned_kg"].iloc[-1] if len(df_c) > 0 and df_c.attrs.get("feasible", True) else 1000.0
    
    # (d) Full hybrid with AdaptiveECMS + regen descent enabled
    controller_d = AdaptiveECMS(s_base=ecms_s, soc_planned=soc_planned, gain=2.0)
    df_d = run_naive_hybrid(
        fault_flag=False,
        ecms_s=None,
        ecms_controller=controller_d,
        loiter_duration_s=1200,
        mtow_kg=mtow_kg,
        engine_rated_W=eng_rated_W,
        battery_capacity_kWh=bat_kwh,
        supercap_mass_kg=sc_mass_kg,
        motor_rated_W=mot_rated_W,
        regen_efficiency=0.5
    )
    fuel_d = df_d["fuel_burned_kg"].iloc[-1] if len(df_d) > 0 and df_d.attrs.get("feasible", True) else 1000.0
    
    print(f"  (a) Baseline non-hybrid        : {baseline_fuel_nominal:.2f} kg")
    print(f"  (b) Constant-BSFC (rule-based) : {fuel_b:.2f} kg")
    print(f"  (c) AdaptiveECMS (no regen)    : {fuel_c:.2f} kg")
    print(f"  (d) AdaptiveECMS + regen       : {fuel_d:.2f} kg")
    
    # Save results for the dashboard
    dashboard_data = {
        "design_vector": design_vector,
        "baseline_endurance_h": baseline_endurance_h,
        "hybrid_endurance_h": hybrid_endurance_h,
        "improvement_pct": percent_improvement,
        "fuel_breakdown": {
            "Baseline": baseline_fuel_nominal,
            "Constant-BSFC": fuel_b,
            "AdaptiveECMS": fuel_c,
            "AdaptiveECMS+Regen": fuel_d
        }
    }
    
    with open(os.path.join(REPORT_DIR, "validation_results.json"), "w") as f:
        json.dump(dashboard_data, f, indent=4)
        
    df_d.to_csv(os.path.join(REPORT_DIR, "best_design_trace.csv"), index=False)
    
    print(f"\nDashboard data saved to {REPORT_DIR}/validation_results.json")
    print(f"Simulation trace saved to {REPORT_DIR}/best_design_trace.csv")

if __name__ == "__main__":
    if len(sys.argv) == 7:
        dv = [float(x) for x in sys.argv[1:]]
        validate_design(dv)
    else:
        print("Please provide exactly 6 arguments for the design vector.")
