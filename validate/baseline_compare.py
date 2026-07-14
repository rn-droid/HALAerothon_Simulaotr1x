"""
Non-hybrid baseline analysis.

Runs the full mission with the turboshaft as the SOLE power source
(no battery / supercapacitor assist) to establish the "pure
turboshaft" reference endurance.

Method
------
1. Build the mission with takeoff + climb + 30 min cruise + descent
   (loiter duration = 0) and integrate fuel burn for these fixed phases.
2. Compute the steady-state loiter fuel-flow rate (constant conditions).
3. Allocate remaining fuel (from 150 kg budget) to loiter:
       max_loiter  =  (150  -  fixed_fuel)  /  loiter_fuel_rate
4. Total baseline endurance = fixed-phase time + max loiter time.

This endurance number is the denominator in the hybrid-improvement
ratio that later tasks will compute.
"""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.aircraft import Aircraft
from sim.engine import Turboshaft
from sim.mission import MissionProfile

# ── Configuration ───────────────────────────────────────────
USABLE_FUEL_KG = 150.0
CRUISE_DURATION_S = 30 * 60          # 30 min
CRUISE_ALT_M = 3000.0


def compute_baseline() -> dict:
    """
    Compute the non-hybrid baseline fuel burn and endurance.

    Returns a dict of all key results for programmatic use.
    """
    ac = Aircraft()
    eng = Turboshaft()
    W = ac.mtow_kg * ac.G

    # ── 1. Build mission with 0 loiter (fixed phases only) ──
    mp = MissionProfile(
        aircraft=ac,
        cruise_alt_m=CRUISE_ALT_M,
        cruise_duration_s=CRUISE_DURATION_S,
        loiter_duration_s=0,           # no loiter yet
    )
    df = mp.build()

    # ── 2. Fuel flow at every 1-s timestep ──────────────────
    df["fuel_flow_kg_s"] = df["power_required_W"].apply(eng.fuel_flow_kg_s)

    phase_fuel = {}
    phase_dur = {}
    phase_mean_P = {}
    for phase in ["takeoff", "climb", "cruise", "descent"]:
        mask = df["phase"] == phase
        phase_fuel[phase] = float(df.loc[mask, "fuel_flow_kg_s"].sum())    # kg  (dt=1 s)
        phase_dur[phase] = int(mask.sum())                                  # s
        phase_mean_P[phase] = float(df.loc[mask, "power_required_W"].mean())

    fixed_fuel = sum(phase_fuel.values())
    fixed_time = sum(phase_dur.values())

    # ── 3. Loiter fuel rate (constant conditions) ───────────
    V_loiter = mp.best_LD_speed(CRUISE_ALT_M)
    P_loiter = ac.shaft_power_required(W, V_loiter, CRUISE_ALT_M, 0.0, "loiter")
    loiter_fuel_rate = eng.fuel_flow_kg_s(P_loiter)      # kg/s

    pf_loiter = P_loiter / eng.rated_power_W
    bsfc_loiter = eng.bsfc(pf_loiter)

    # ── 4. Max loiter from remaining fuel ───────────────────
    remaining_fuel = USABLE_FUEL_KG - fixed_fuel
    max_loiter_s = remaining_fuel / loiter_fuel_rate

    phase_fuel["loiter"] = remaining_fuel
    phase_dur["loiter"] = max_loiter_s
    phase_mean_P["loiter"] = P_loiter

    total_endurance_s = fixed_time + max_loiter_s
    total_endurance_h = total_endurance_s / 3600.0

    return {
        "phase_fuel_kg":        phase_fuel,
        "phase_dur_s":          phase_dur,
        "phase_mean_P_W":       phase_mean_P,
        "fixed_fuel_kg":        fixed_fuel,
        "remaining_fuel_kg":    remaining_fuel,
        "loiter_fuel_rate_kg_s": loiter_fuel_rate,
        "loiter_fuel_rate_kg_h": loiter_fuel_rate * 3600,
        "max_loiter_s":         max_loiter_s,
        "total_endurance_s":    total_endurance_s,
        "total_endurance_h":    total_endurance_h,
        "P_loiter_kW":          P_loiter / 1000,
        "bsfc_loiter":          bsfc_loiter,
        "V_loiter_ms":          V_loiter,
        "usable_fuel_kg":       USABLE_FUEL_KG,
    }


def print_results(r: dict) -> None:
    """Pretty-print baseline results."""
    print(f"\n{'='*62}")
    print("  NON-HYBRID BASELINE  (turboshaft-only, 150 kg usable fuel)")
    print(f"{'='*62}")

    print(f"\n  Fuel burn by phase:")
    print(f"  {'Phase':10s}  {'Duration':>10s}  {'Mean P[kW]':>10s}  "
          f"{'Fuel[kg]':>9s}  {'Rate[kg/h]':>10s}")
    print(f"  {'-'*10}  {'-'*10}  {'-'*10}  {'-'*9}  {'-'*10}")

    for p in ["takeoff", "climb", "cruise", "loiter", "descent"]:
        dur = r["phase_dur_s"][p]
        fuel = r["phase_fuel_kg"][p]
        rate = (fuel / dur * 3600) if dur > 0 else 0
        mean_P = r["phase_mean_P_W"].get(p, 0) / 1000

        if dur >= 3600:
            dur_str = f"{dur/3600:.2f} h"
        elif dur >= 60:
            dur_str = f"{dur/60:.1f} min"
        else:
            dur_str = f"{dur:.0f} s"

        print(f"  {p:10s}  {dur_str:>10s}  {mean_P:10.1f}  "
              f"{fuel:9.2f}  {rate:10.2f}")

    total_fuel = sum(r["phase_fuel_kg"].values())
    print(f"  {'':10s}  {'':>10s}  {'':>10s}  {'-'*9}")
    print(f"  {'TOTAL':10s}  {'':>10s}  {'':>10s}  {total_fuel:9.2f}")

    print(f"\n  Loiter conditions:")
    print(f"    V_loiter     = {r['V_loiter_ms']:.1f} m/s  "
          f"({r['V_loiter_ms']*3.6:.0f} km/h)")
    print(f"    P_shaft      = {r['P_loiter_kW']:.1f} kW  "
          f"(pf = {r['P_loiter_kW']*1000/60000:.2f})")
    print(f"    BSFC         = {r['bsfc_loiter']:.0f} g/kWh")
    print(f"    Fuel rate    = {r['loiter_fuel_rate_kg_h']:.2f} kg/h")

    print(f"\n  Fixed-phase fuel  = {r['fixed_fuel_kg']:.2f} kg")
    print(f"  Remaining (loiter)= {r['remaining_fuel_kg']:.2f} kg")
    print(f"  Max loiter        = {r['max_loiter_s']/60:.1f} min  "
          f"({r['max_loiter_s']/3600:.2f} h)")

    print(f"\n  *** BASELINE ENDURANCE = {r['total_endurance_h']:.2f} hours ***")
    print(f"      ({r['total_endurance_s']:.0f} s  =  "
          f"{r['total_endurance_s']/60:.1f} min)")
    print(f"{'='*62}")


def compute_baseline_mass_consistent() -> dict:
    """
    Compute baseline using the same iterative mass-sizing loop as the hybrid.
    Assumes an engine sized to meet climb (120 kW).
    """
    engine_rated_W = 120_000.0
    eng = Turboshaft(rated_power_W=engine_rated_W)
    
    PAYLOAD_KG = 200.0
    AIRFRAME_KG = 300.0
    eng_mass = engine_rated_W / 2000.0
    
    total_mass = PAYLOAD_KG + AIRFRAME_KG + eng_mass + 200.0
    
    max_iter = 50
    tol = 0.5
    
    for i in range(max_iter):
        # We need an Aircraft instance with the updated mass
        ac = Aircraft(mtow_kg=total_mass)
        mp = MissionProfile(
            aircraft=ac,
            cruise_alt_m=CRUISE_ALT_M,
            cruise_duration_s=CRUISE_DURATION_S,
            loiter_duration_s=1200,  # Same as hybrid nominal mission
        )
        df = mp.build()
        df["fuel_flow_kg_s"] = df["power_required_W"].apply(eng.fuel_flow_kg_s)
        
        fuel_burned = df["fuel_flow_kg_s"].sum() * 1.0
        
        new_mass = PAYLOAD_KG + AIRFRAME_KG + eng_mass + fuel_burned
        if abs(new_mass - total_mass) < tol:
            total_mass = new_mass
            break
        total_mass = new_mass
        
    spare_mass = 1000.0 - total_mass
    t_nom = len(df)
    avg_fuel_flow = fuel_burned / t_nom
    
    if spare_mass > 0:
        endurance_s = t_nom + (spare_mass / avg_fuel_flow)
    else:
        endurance_s = t_nom
        
    return {
        "converged_mass_kg": total_mass,
        "spare_fuel_capacity_kg": spare_mass,
        "fuel_burned_nominal_kg": fuel_burned,
        "total_endurance_h": endurance_s / 3600.0,
        "engine_mass_kg": eng_mass
    }

if __name__ == "__main__":
    results = compute_baseline()
    print_results(results)
    
    print("\n" + "="*62)
    print("  MASS-CONSISTENT BASELINE (Task 14 Update)")
    print("="*62)
    mc_results = compute_baseline_mass_consistent()
    print(f"  Engine Mass          : {mc_results['engine_mass_kg']:.1f} kg (120 kW)")
    print(f"  Converged Empty Mass : {mc_results['converged_mass_kg']:.1f} kg")
    print(f"  Spare Fuel Capacity  : {mc_results['spare_fuel_capacity_kg']:.1f} kg")
    print(f"  Nominal Fuel Burned  : {mc_results['fuel_burned_nominal_kg']:.2f} kg")
    print(f"  *** CORRECTED ENDURANCE = {mc_results['total_endurance_h']:.2f} hours ***")
    print("="*62)
