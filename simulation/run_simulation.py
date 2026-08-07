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
from simulation.aerodynamics import flight_state_forces
from simulation.engine_model      import TurboshaftEngine
from simulation.battery_model     import BatteryPack
from simulation.power_electronics import PowerElectronicsChain
from simulation.mission_profile   import build_mission_profile, build_mission1_profile, MISSION1_PRESET, MissionSegment
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
    mission_name: str            = "default",
    cruise_duration_s: float  = None,
    loiter_duration_s: float  = None,
    cruise_alt_m: float       = None,
    loiter_alt_m: float       = 3000.0,
    climb_rate_ms: float      = None,
    descent_rate_ms: float    = None,
    cruise_speed_ms: float    = None,
    takeoff_enabled: bool     = True,
    descent_enabled: bool     = True,
    climb_duration_s: float   = None,
    climb_distance_m: float   = None,
    cruise_distance_m: float  = None,
    headwind_ms: float        = 0.0,
    mtow_kg: float            = None,
    wing_area_m2: float       = None,
    aspect_ratio: float       = None,
    oswald_efficiency: float  = None,
    cd0: float                = None,
    engine_rated_W: float     = ENGINE_RATED_W,
    battery_capacity_kWh: float = BAT_CAPACITY,
    supercap_energy_kWh: float = 1.5,
    motor_rated_W: float      = 120_000.0,
    bsfc_scale: float         = None,
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
    is_mission1 = str(mission_name).lower() == "mission1"
    mtow = mtow_kg or (MISSION1_PRESET["mtow_kg"] if is_mission1 else AIRCRAFT.mtow_kg)
    wing_area = wing_area_m2 if wing_area_m2 is not None else (MISSION1_PRESET["wing_area_m2"] if is_mission1 else AERO.S_ref)
    ar = aspect_ratio if aspect_ratio is not None else (MISSION1_PRESET["aspect_ratio"] if is_mission1 else AERO.AR)
    oswald = oswald_efficiency if oswald_efficiency is not None else (MISSION1_PRESET["oswald_efficiency"] if is_mission1 else AERO.e0)
    drag_cd0 = cd0 if cd0 is not None else (MISSION1_PRESET["cd0"] if is_mission1 else AERO.CD0)
    engine_power = engine_rated_W or (MISSION1_PRESET["engine_rated_W"] if is_mission1 else ENGINE_RATED_W)
    battery_capacity = battery_capacity_kWh or (MISSION1_PRESET["battery_capacity_kWh"] if is_mission1 else BAT_CAPACITY)
    if is_mission1:
        if engine_rated_W == ENGINE_RATED_W:
            engine_power = MISSION1_PRESET["engine_rated_W"]
        if battery_capacity_kWh == BAT_CAPACITY:
            battery_capacity = MISSION1_PRESET["battery_capacity_kWh"]
        if supercap_energy_kWh == 1.5:
            supercap_energy_kWh = MISSION1_PRESET["supercap_energy_kWh"]
        if fuel_start_kg == TOTAL_FUEL_KG:
            fuel_start_kg = MISSION1_PRESET["fuel_start_kg"]
        if initial_soc == INITIAL_SOC:
            initial_soc = MISSION1_PRESET["initial_soc"]
        if motor_rated_W == 120_000.0:
            motor_rated_W = MISSION1_PRESET["motor_rated_W"]
        if bsfc_scale is None:
            bsfc_scale = MISSION1_PRESET["bsfc_scale"]
    if bsfc_scale is None:
        bsfc_scale = 1.0

    engine  = TurboshaftEngine(engine_power, bsfc_scale=bsfc_scale)
    battery = BatteryPack(battery_capacity, initial_soc)
    pe      = PowerElectronicsChain()
    ctrl    = PowerSplitController(engine_power, battery_capacity, supercap_energy_kWh=supercap_energy_kWh, motor_rated_W=motor_rated_W)
    mission_builder = build_mission1_profile if is_mission1 else build_mission_profile
    segments = mission_builder(
            cruise_duration_s=cruise_duration_s,
            loiter_duration_s=loiter_duration_s,
            cruise_alt_m=cruise_alt_m,
            loiter_alt_m=loiter_alt_m,
            climb_rate_ms=climb_rate_ms,
            descent_rate_ms=descent_rate_ms,
            cruise_speed_ms=cruise_speed_ms,
            takeoff_enabled=takeoff_enabled,
            descent_enabled=descent_enabled,
            climb_duration_s=climb_duration_s,
            climb_distance_m=climb_distance_m,
            cruise_distance_m=cruise_distance_m,
            headwind_ms=headwind_ms,
            mtow_kg=mtow,
            wing_area_m2=wing_area,
            aspect_ratio=ar,
            oswald_efficiency=oswald,
            cd0=drag_cd0,
        )

    initial_weight_N = mtow * G
    fuel_remaining = fuel_start_kg
    records = []
    t = 0.0
    total_shaft_energy_J = 0.0
    distance_m = 0.0

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
            ground_speed_ms = max(0.0, V - headwind_ms)
            current_mass_kg = max(mtow - engine.cumulative_fuel_kg, mtow - fuel_start_kg)
            W_N = current_mass_kg * G

            # Required shaft power. During regenerative descent, a controlled
            # windmilling propeller extracts a bounded fraction of potential
            # energy instead of clamping demand to zero.
            forces = flight_state_forces(
                V, h, W_N, roc, eta_p,
                load_factor=seg.load_factor,
                S_ref=wing_area,
                AR=ar,
                e0=oswald,
                CD0=drag_cd0,
            )
            P_shaft_req = forces["shaft_power_W"]
            if is_mission1 and seg.name == "TAKEOFF":
                # Ground-roll acceleration is not captured by steady lift/drag;
                # use the report baseline transient demand for the 3 s rotation.
                P_shaft_req = 123_000.0
            if is_mission1 and seg.name == "LANDING":
                P_shaft_req = 0.0
            if seg.regen:
                recoverable_W = max(0.0, W_N * abs(roc) * 0.09)
                aero_drag_limit_W = max(0.0, P_shaft_req)
                P_shaft_req = -min(3_000.0, recoverable_W, aero_drag_limit_W + recoverable_W * 0.20)
            else:
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
            timestep_flags = list(bat_out["violation_flags"])
            if seg.name != "LANDING" and V < forces["min_safe_speed_ms"]:
                timestep_flags.append(
                    f"STALL_MARGIN_LOW: V={V:.2f} < 1.2Vs={forces['min_safe_speed_ms']:.2f}"
                )

            # Delivered shaft power. Parallel mode bypasses the generator and
            # motor chain through a mechanical clutch; series mode uses it.
            if split["architecture"] == "PARALLEL":
                P_engine_shaft_actual = eng_out["P_engine_W"] * ctrl.eta_parallel
            else:
                P_engine_shaft_actual = pe.engine_to_prop_shaft(eng_out["P_engine_W"])

            P_bat_shaft_actual = pe.bat_to_prop_shaft(max(split["P_bat_elec_W"], 0.0))
            P_supercap_shaft = max(split.get("P_supercap_W", 0.0), 0.0)
            if P_shaft_req < 0.0:
                P_shaft_actual = P_shaft_req
            else:
                P_shaft_available = P_engine_shaft_actual + P_bat_shaft_actual + P_supercap_shaft
                P_shaft_actual = min(P_shaft_req, P_shaft_available)
                if P_shaft_available + 1.0 < P_shaft_req:
                    timestep_flags.append(
                        f"POWER_DEFICIT: available={P_shaft_available:.0f}W < required={P_shaft_req:.0f}W"
                    )
            total_shaft_energy_J += max(P_shaft_actual, 0.0) * DT
            distance_m += ground_speed_ms * DT

            # System efficiency
            eta_sys = pe.system_efficiency(fuel_energy_W, P_shaft_actual)

            # Motor RPM proxy (scales with power vs rated)
            rated_shaft = engine_power * pe.eta_eng_to_shaft
            rpm_proxy = MOTOR_RPM_CRUISE * (abs(P_shaft_actual) / max(rated_shaft, 1.0))
            rpm_proxy = max(500.0, min(rpm_proxy, MOTOR_RPM_CRUISE * 1.2))

            # Torque proxy
            tau = pe.motor_torque_proxy(abs(P_shaft_actual), rpm_proxy)

            # Endurance remaining (estimate from battery energy left)
            E_bat_remain_W = battery.remaining_energy_Wh() * 3600.0
            P_lo_ref = 48700.0  # loiter shaft power from Phase 2
            endurance_s = (E_bat_remain_W / P_lo_ref) if P_lo_ref > 0 else 0.0

            records.append({
                "t_s":                round(t, 1),
                "phase":              seg.name,
                "mission_name":       mission_name,
                "altitude_m":         round(h, 1),
                "airspeed_ms":        round(V, 2),
                "ground_speed_ms":     round(ground_speed_ms, 2),
                "headwind_ms":         round(headwind_ms, 2),
                "distance_m":          round(distance_m, 1),
                "roc_ms":             round(roc, 2),
                "load_factor":        round(forces["load_factor"], 4),
                "mass_kg":            round(current_mass_kg, 3),
                "weight_N":           round(W_N, 2),
                "initial_weight_N":   round(initial_weight_N, 2),
                "rho_kg_m3":          round(forces["rho"], 5),
                "q_Pa":               round(forces["q"], 2),
                "gamma_deg":          round(forces["gamma_deg"], 3),
                "CL":                 round(forces["CL"], 5),
                "CD":                 round(forces["CD"], 5),
                "CD_induced":         round(forces["CD_i"], 5),
                "drag_N":             round(forces["D_N"], 2),
                "lift_required_N":    round(forces["lift_required_N"], 2),
                "thrust_required_N":  round(forces["thrust_required_N"], 2),
                "propulsive_power_W": round(forces["propulsive_power_W"], 1),
                "stall_speed_ms":     round(forces["stall_speed_ms"], 2),
                "min_safe_speed_ms":  round(forces["min_safe_speed_ms"], 2),
                "stall_margin":       round(forces["stall_margin"], 3),
                "P_engine_W":         round(eng_out["P_engine_W"], 1),
                "P_battery_W":        round(split["P_bat_elec_W"], 1),
                "P_supercap_W":       round(split.get("P_supercap_W", 0.0), 1),
                "P_shaft_W":          round(P_shaft_actual, 1),
                "P_shaft_required_W": round(P_shaft_req, 1),
                "P_engine_shaft_available_W": round(P_engine_shaft_actual, 1),
                "P_battery_shaft_available_W": round(P_bat_shaft_actual, 1),
                "P_engine_alone_deficit_W": round(max(0.0, P_shaft_req - engine_power), 1),
                "P_regen_W":          round(abs(P_shaft_req) if P_shaft_req < 0.0 else 0.0, 1),
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
                "architecture":       split["architecture"],
                "clutch_state":       split["clutch_state"],
                "ecms_s":             round(split["ecms_s"], 3),
                "soc_target_pct":     round(split["soc_target"] * 100.0, 2),
                "engine_on":          eng_out["is_running"],
                "violation_flags":    "|".join(timestep_flags) if timestep_flags else "",
                "mtow_kg":            round(mtow, 2),
                "wing_area_m2":       round(wing_area, 3),
                "aspect_ratio":       round(ar, 3),
                "cd0":                round(drag_cd0, 4),
                "engine_rated_W":     round(engine_power, 1),
                "bsfc_scale":         round(bsfc_scale, 4),
                "motor_rated_W":      round(motor_rated_W, 1),
                "battery_capacity_kWh": round(battery_capacity, 3),
                "supercap_energy_kWh": round(supercap_energy_kWh, 3),
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
    run_sim(mission_name="mission1", verbose=True, export_csv=True, export_json=True)
