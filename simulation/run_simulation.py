"""
Hybrid-Electric UAV Propulsion — Main Simulation Runner (Phase 5)

Integrates all subsystem models into a 1-second time-stepped simulation,
producing the full time-series dataset that feeds the dashboard.

Outputs per timestep:
  t, phase, altitude_m, airspeed_ms,
  P_engine_W, P_battery_W, P_shaft_W,
  soc, soc_pct, fuel_remaining_kg,
  c_rate, motor_rpm_proxy, torque_Nm,
  eta_system, endurance_remaining_s,
  mode, violation_flags

Usage:
  python simulation/run_simulation.py            # default mission, saves CSV+JSON
  from simulation.run_simulation import run_sim  # call from other code
"""

import sys
import os
import json
import math
import csv
from pathlib import Path

# Allow imports from project root
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import AIRCRAFT, AERO, ENGINE, BATTERY, FUEL_LHV, G
from simulation.atmosphere   import isa_density
from simulation.aerodynamics import required_shaft_power, best_endurance_speed
from simulation.engine_model      import TurboshaftEngine
from simulation.battery_model     import BatteryPack
from simulation.power_electronics import PowerElectronicsChain
from simulation.mission_profile   import build_mission_profile, MissionSegment
from simulation.controller        import PowerSplitController


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATION PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
TOTAL_FUEL_KG   = 18.88    # From Phase 3 sizing
ENGINE_RATED_W  = 60_000.0
BAT_CAPACITY    = 55.0     # kWh
INITIAL_SOC     = 0.90
MOTOR_RPM_CRUISE = 4500.0  # proxy cruise RPM for torque estimate
DT              = 1.0      # timestep [s]

# ─────────────────────────────────────────────────────────────────────────────
# VALIDATION CHECKPOINTS (Phase 2/3/4 hand-calculated values)
# ─────────────────────────────────────────────────────────────────────────────
CHECKPOINTS = {
    "cruise_shaft_kW":  52.9,
    "climb_shaft_kW":  111.2,
    "loiter_shaft_kW":  48.7,
    "total_fuel_kg":    17.6,
    "final_soc_pct":    20.9,
    "total_energy_kWh": 74.16,
}


def run_sim(
    cruise_duration_s: float  = None,
    loiter_duration_s: float  = None,
    cruise_alt_m: float       = None,
    loiter_alt_m: float       = 3000.0,
    climb_rate_ms: float      = None,
    initial_soc: float        = INITIAL_SOC,
    fuel_start_kg: float      = TOTAL_FUEL_KG,
    verbose: bool             = True,
    export_csv: bool          = True,
    export_json: bool         = True,
) -> list[dict]:
    """
    Run the full hybrid-electric UAV mission simulation.

    Returns:
        List of per-timestep data dicts (also exported to data/outputs/).
    """
    # ── Initialise subsystems ────────────────────────────────────────
    engine  = TurboshaftEngine(ENGINE_RATED_W)
    battery = BatteryPack(BAT_CAPACITY, initial_soc)
    pe      = PowerElectronicsChain()
    ctrl    = PowerSplitController(ENGINE_RATED_W, BAT_CAPACITY)
    segments = build_mission_profile(
        cruise_duration_s=cruise_duration_s,
        loiter_duration_s=loiter_duration_s,
        cruise_alt_m=cruise_alt_m,
        loiter_alt_m=loiter_alt_m,
        climb_rate_ms=climb_rate_ms,
    )

    W_N = AIRCRAFT.mtow_kg * G
    fuel_remaining = fuel_start_kg
    records = []
    t = 0.0
    total_shaft_energy_J = 0.0

    # ── Time-step loop ───────────────────────────────────────────────
    for seg in segments:
        n_steps = int(seg.duration_s / DT)
        seg_fraction = 0.0

        for step in range(n_steps):
            seg_fraction = step / max(n_steps - 1, 1)

            # Interpolate altitude within segment
            h = seg.h_start_m + (seg.h_end_m - seg.h_start_m) * seg_fraction
            V = seg.V_ms
            roc = seg.roc_ms
            eta_p = seg.eta_prop

            # Required shaft power
            P_shaft_req = required_shaft_power(V, h, W_N, roc, eta_p)
            P_shaft_req = max(P_shaft_req, 0.0)

            # Controller decision
            split = ctrl.compute_split(
                phase_name=seg.name,
                P_shaft_demand_W=P_shaft_req,
                eta_eng_to_shaft=pe.eta_eng_to_shaft,
                eta_bat_to_shaft=pe.eta_bat_to_shaft,
                current_soc=battery.soc,
            )

            # Engine step
            is_peak = (seg.name in {"TAKEOFF"})
            eng_out = engine.step(split["P_engine_cmd_W"], DT)
            fuel_burn = eng_out["fuel_burn_kg"]
            fuel_remaining = max(0.0, fuel_remaining - fuel_burn)

            # Fuel equivalent power for efficiency calculation
            fuel_energy_W = (fuel_burn * FUEL_LHV / DT) if fuel_burn > 0 else 1e-9

            # Battery step
            bat_out = battery.step(split["P_bat_elec_W"], DT, is_peak=is_peak)

            # Delivered shaft power
            P_shaft_actual = (
                pe.engine_to_prop_shaft(eng_out["P_engine_W"]) +
                pe.bat_to_prop_shaft(max(split["P_bat_elec_W"], 0.0))
            )
            total_shaft_energy_J += P_shaft_actual * DT

            # System efficiency
            eta_sys = pe.system_efficiency(fuel_energy_W, P_shaft_actual)

            # Motor RPM proxy (scales with power vs rated)
            rated_shaft = ENGINE_RATED_W * pe.eta_eng_to_shaft
            rpm_proxy = MOTOR_RPM_CRUISE * (P_shaft_actual / max(rated_shaft, 1.0))
            rpm_proxy = max(500.0, min(rpm_proxy, MOTOR_RPM_CRUISE * 1.2))

            # Torque proxy
            tau = pe.motor_torque_proxy(P_shaft_actual, rpm_proxy)

            # Endurance remaining (estimate from battery energy left)
            E_bat_remain_W = battery.remaining_energy_Wh() * 3600.0
            P_lo_ref = 48700.0  # loiter shaft power from Phase 2
            endurance_s = (E_bat_remain_W / P_lo_ref) if P_lo_ref > 0 else 0.0

            records.append({
                "t_s":                round(t, 1),
                "phase":              seg.name,
                "altitude_m":         round(h, 1),
                "airspeed_ms":        round(V, 2),
                "roc_ms":             round(roc, 2),
                "P_engine_W":         round(eng_out["P_engine_W"], 1),
                "P_battery_W":        round(split["P_bat_elec_W"], 1),
                "P_shaft_W":          round(P_shaft_actual, 1),
                "soc":                round(bat_out["soc"], 5),
                "soc_pct":            round(bat_out["soc_pct"], 3),
                "c_rate":             round(bat_out["c_rate"], 4),
                "fuel_remaining_kg":  round(fuel_remaining, 4),
                "fuel_burned_kg":     round(engine.cumulative_fuel_kg, 4),
                "bsfc_g_kWh":         round(eng_out["bsfc_g_kWh"], 2),
                "motor_rpm":          round(rpm_proxy, 1),
                "motor_torque_Nm":    round(tau, 2),
                "eta_system":         round(eta_sys, 4),
                "endurance_remain_s": round(endurance_s, 1),
                "mode":               split["mode"],
                "engine_on":          eng_out["is_running"],
                "violation_flags":    "|".join(bat_out["violation_flags"]) if bat_out["violation_flags"] else "",
            })

            t += DT

    # ── Validation Report ────────────────────────────────────────────
    if verbose:
        print("\n" + "=" * 62)
        print("  PHASE 5 SIMULATION VALIDATION REPORT")
        print("=" * 62)
        _validate(records, total_shaft_energy_J)

    # ── Export ───────────────────────────────────────────────────────
    out_dir = Path(__file__).parent.parent / "data" / "outputs"
    out_dir.mkdir(parents=True, exist_ok=True)

    if export_csv:
        csv_path = out_dir / "mission_timeseries.csv"
        _write_csv(records, csv_path)
        if verbose:
            print(f"\n  CSV saved  -> {csv_path}")

    if export_json:
        json_path = out_dir / "mission_timeseries.json"
        with open(json_path, "w") as f:
            json.dump(records, f, indent=2)
        if verbose:
            print(f"  JSON saved -> {json_path}")

    return records


# ─────────────────────────────────────────────────────────────────────────────
def _validate(records, total_shaft_J):
    """Compare key simulation outputs against Phase 2/3/4 checkpoints."""
    tol_pct = 10.0  # acceptable relative tolerance [%]

    def chk(name, simval, refval, unit=""):
        err = abs(simval - refval) / max(abs(refval), 1e-9) * 100.0
        flag = "OK" if err < tol_pct else "WARN"
        print(f"  [{flag}] {name:<30} sim={simval:8.2f}  ref={refval:8.2f}  "
              f"err={err:.1f}%  {unit}")

    # Extract phase-level averages
    def phase_avg(phase_name, field):
        vals = [r[field] for r in records if r["phase"] == phase_name]
        return sum(vals) / len(vals) if vals else 0.0

    print()
    chk("Cruise shaft power",
        phase_avg("CRUISE",  "P_shaft_W") / 1000.0,
        CHECKPOINTS["cruise_shaft_kW"], "kW")

    chk("Climb shaft power",
        phase_avg("CLIMB",   "P_shaft_W") / 1000.0,
        CHECKPOINTS["climb_shaft_kW"],  "kW")

    chk("Loiter shaft power",
        phase_avg("LOITER",  "P_shaft_W") / 1000.0,
        CHECKPOINTS["loiter_shaft_kW"], "kW")

    final = records[-1]
    chk("Final SoC",
        final["soc_pct"],
        CHECKPOINTS["final_soc_pct"], "%")

    chk("Total fuel burned",
        final["fuel_burned_kg"],
        CHECKPOINTS["total_fuel_kg"], "kg")

    chk("Total shaft energy",
        total_shaft_J / 3.6e6,
        CHECKPOINTS["total_energy_kWh"], "kWh")

    print()
    n_viols = sum(1 for r in records if r["violation_flags"])
    print(f"  Constraint violations: {n_viols} timesteps flagged")
    print(f"  Total mission time:    {records[-1]['t_s']:.0f} s  "
          f"({records[-1]['t_s']/60:.1f} min)")
    print(f"  Final fuel remaining:  {final['fuel_remaining_kg']:.2f} kg")
    print(f"  Final SoC:             {final['soc_pct']:.1f}%")
    print("=" * 62)


def _write_csv(records, path):
    if not records:
        return
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    run_sim(verbose=True, export_csv=True, export_json=True)
