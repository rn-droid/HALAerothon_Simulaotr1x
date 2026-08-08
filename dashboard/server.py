"""
Dashboard Server — Flask backend for the Hybrid-Electric UAV Dashboard.

Serves the single-page dashboard and provides a /api/simulate endpoint
that re-runs the Phase 5 simulation with user-tweaked parameters.
"""

import sys, json, os
from pathlib import Path

# Project root for imports
ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from flask import Flask, send_from_directory, jsonify, request
from simulation.run_simulation import run_sim
from simulation.mission_profile import MISSION1_PRESET

app = Flask(__name__, static_folder=str(ROOT / "dashboard"))


def _float_param(params, name, default):
    try:
        value = params.get(name, default)
        if value == "":
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _optional_float_param(params, name):
    value = params.get(name)
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _bool_param(params, name, default=True):
    value = params.get(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "on"}
    return bool(value)


def _physical_params(params):
    """Dashboard-editable physical and mission parameters."""
    mission_type = str(params.get("mission_type", "ISR")).upper()
    fuel_capacity_kg = _float_param(params, "fuel_capacity_kg", 80.0)
    initial_soc = _optional_float_param(params, "initial_soc_pct")
    fuel_start = _optional_float_param(params, "fuel_start_kg")
    wind_ms = _float_param(params, "wind_speed_ms", 0.0)
    gust_ms = _float_param(params, "gust_speed_ms", 0.0)
    temp_offset_c = _float_param(params, "temperature_offset_c", 0.0)
    humidity_pct = _float_param(params, "humidity_pct", 45.0)
    rain_rate_mm_h = _float_param(params, "rain_rate_mm_h", 0.0)
    dust_factor = _float_param(params, "dust_factor", 0.0)
    aero_penalty = (
        max(0.0, gust_ms) * 0.0007 +
        max(0.0, rain_rate_mm_h) * 0.0008 +
        max(0.0, humidity_pct - 60.0) * 0.00008 +
        max(0.0, temp_offset_c) * 0.00025 +
        max(0.0, dust_factor) * 0.002
    )
    physical = {
        "mission_name": "mission1" if mission_type == "MISSION1" else "default",
        "mission_type": mission_type,
        "mtow_kg": _float_param(params, "mtow_kg", 1000.0),
        "payload_kg": _float_param(params, "payload_kg", 200.0),
        "airframe_kg": _float_param(params, "airframe_kg", 300.0),
        "wing_area_m2": _float_param(params, "wing_area_m2", 21.8),
        "aspect_ratio": _float_param(params, "aspect_ratio", 11.75),
        "cd0": _float_param(params, "cd0", 0.025) + aero_penalty,
        "base_cd0": _float_param(params, "cd0", 0.025),
        "oswald_efficiency": _float_param(params, "oswald_efficiency", 0.80),
        "cruise_speed_ms": _float_param(params, "cruise_speed_ms", 69.44),
        "cruise_alt_m": _float_param(params, "cruise_alt_m", 5000.0),
        "loiter_alt_m": _float_param(params, "loiter_alt_m", 3000.0),
        "climb_rate_ms": _float_param(params, "climb_rate_ms", 5.0),
        "descent_rate_ms": _float_param(params, "descent_rate_ms", 3.0),
        "engine_rated_W": _float_param(params, "engine_rated_W", 60000.0),
        "bsfc_scale": _float_param(params, "bsfc_scale", 1.0),
        "motor_rated_W": _float_param(params, "motor_rated_W", 70000.0),
        "battery_capacity_kWh": _float_param(params, "battery_capacity_kWh", 55.0),
        "supercap_energy_kWh": _float_param(params, "supercap_energy_kWh", 1.5),
        "fuel_capacity_kg": fuel_capacity_kg,
        "fuel_start_kg": fuel_start if fuel_start is not None else fuel_capacity_kg,
        "initial_soc": (initial_soc / 100.0) if initial_soc is not None else 1.0,
        "cruise_duration_s": _float_param(params, "cruise_duration_min", 30.0) * 60.0,
        "loiter_duration_s": _float_param(params, "loiter_duration_min", 20.0) * 60.0,
        "takeoff_enabled": _bool_param(params, "takeoff_enabled", True),
        "descent_enabled": _bool_param(params, "descent_enabled", True),
        "climb_duration_s": (_optional_float_param(params, "climb_duration_min") or 0.0) * 60.0,
        "climb_distance_m": (_optional_float_param(params, "climb_distance_km") or 0.0) * 1000.0,
        "cruise_distance_m": (_optional_float_param(params, "cruise_distance_km") or 0.0) * 1000.0,
        "wind_speed_ms": wind_ms,
        "gust_speed_ms": gust_ms,
        "temperature_offset_c": temp_offset_c,
        "humidity_pct": humidity_pct,
        "rain_rate_mm_h": rain_rate_mm_h,
        "dust_factor": dust_factor,
        "aero_penalty_cd0": aero_penalty,
    }
    if mission_type == "MISSION1":
        physical.update({
            "mtow_kg": MISSION1_PRESET["mtow_kg"],
            "payload_kg": MISSION1_PRESET["payload_kg"],
            "fuel_capacity_kg": MISSION1_PRESET["fuel_start_kg"],
            "fuel_start_kg": MISSION1_PRESET["fuel_start_kg"],
            "battery_capacity_kWh": MISSION1_PRESET["battery_capacity_kWh"],
            "initial_soc": MISSION1_PRESET["initial_soc"],
            "supercap_energy_kWh": MISSION1_PRESET["supercap_energy_kWh"],
            "engine_rated_W": MISSION1_PRESET["engine_rated_W"],
            "bsfc_scale": MISSION1_PRESET["bsfc_scale"],
            "motor_rated_W": MISSION1_PRESET["motor_rated_W"],
            "wing_area_m2": MISSION1_PRESET["wing_area_m2"],
            "aspect_ratio": MISSION1_PRESET["aspect_ratio"],
            "cd0": MISSION1_PRESET["cd0"],
            "base_cd0": MISSION1_PRESET["cd0"],
            "oswald_efficiency": MISSION1_PRESET["oswald_efficiency"],
            "cruise_speed_ms": MISSION1_PRESET["cruise_speed_ms"],
            "cruise_alt_m": MISSION1_PRESET["cruise_alt_m"],
            "loiter_alt_m": MISSION1_PRESET["loiter_alt_m"],
            "climb_rate_ms": MISSION1_PRESET["climb_rate_ms"],
            "descent_rate_ms": MISSION1_PRESET["descent_rate_ms"],
            "cruise_duration_s": MISSION1_PRESET["cruise_duration_s"],
            "loiter_duration_s": MISSION1_PRESET["loiter_duration_s"],
            "takeoff_enabled": MISSION1_PRESET["takeoff_enabled"],
            "descent_enabled": MISSION1_PRESET["descent_enabled"],
        })
    return physical


def _run_dashboard_sim(p):
    return run_sim(
        mission_name=p["mission_name"],
        cruise_duration_s=p["cruise_duration_s"],
        loiter_duration_s=p["loiter_duration_s"],
        cruise_alt_m=p["cruise_alt_m"],
        loiter_alt_m=p["loiter_alt_m"],
        climb_rate_ms=p["climb_rate_ms"],
        descent_rate_ms=p["descent_rate_ms"],
        cruise_speed_ms=p["cruise_speed_ms"],
        takeoff_enabled=p["takeoff_enabled"],
        descent_enabled=p["descent_enabled"],
        climb_duration_s=p["climb_duration_s"],
        climb_distance_m=p["climb_distance_m"],
        cruise_distance_m=p["cruise_distance_m"],
        headwind_ms=p["wind_speed_ms"],
        mtow_kg=p["mtow_kg"],
        wing_area_m2=p["wing_area_m2"],
        aspect_ratio=p["aspect_ratio"],
        oswald_efficiency=p["oswald_efficiency"],
        cd0=p["cd0"],
        engine_rated_W=p["engine_rated_W"],
        bsfc_scale=p["bsfc_scale"],
        battery_capacity_kWh=p["battery_capacity_kWh"],
        supercap_energy_kWh=p["supercap_energy_kWh"],
        motor_rated_W=p["motor_rated_W"],
        initial_soc=p["initial_soc"],
        fuel_start_kg=p["fuel_start_kg"],
        verbose=False,
        export_csv=False,
        export_json=False,
    )


def _estimate_mass_kg(p):
    if p.get("mission_name") == "mission1":
        return 1000.0
    engine_kg = (p["engine_rated_W"] / 1000.0) / 2.0
    generator_kg = (p["engine_rated_W"] / 1000.0) / 5.0
    motor_kg = max(p["engine_rated_W"] / 1000.0, 60.0) / 5.0
    battery_kg = p["battery_capacity_kWh"] * 1000.0 / 200.0
    supercap_kg = p["supercap_energy_kWh"] * 200.0
    electronics_kg = 28.0
    gear_margin_kg = 35.0
    return (
        p["payload_kg"] + p["airframe_kg"] + engine_kg + generator_kg +
        motor_kg + battery_kg + supercap_kg + p["fuel_start_kg"] +
        electronics_kg + gear_margin_kg
    )


def _summarize_candidate(records, p):
    def _f(value):
        return float(value)

    final = records[-1]
    mission_time_min = _f(final["t_s"]) / 60.0
    fuel_burned = _f(final["fuel_burned_kg"])
    mass = _estimate_mass_kg(p)
    violations = sum(1 for r in records if r.get("violation_flags"))
    fuel_ok = _f(final["fuel_remaining_kg"]) > 0.05
    soc_ok = _f(final["soc_pct"]) > 20.05
    mass_ok = mass <= p["mtow_kg"]
    mission_feasible = fuel_ok and soc_ok and violations == 0
    feasible = mission_feasible and mass_ok
    reserve_score = _f(final["fuel_remaining_kg"]) * 2.0 + max(0.0, _f(final["soc_pct"]) - 20.0) * 0.7
    score = mission_time_min + reserve_score - max(0.0, mass - p["mtow_kg"]) * 0.6 - violations * 3.0
    phase_stats = {}
    for phase in sorted({r["phase"] for r in records}):
        rows = [r for r in records if r["phase"] == phase]
        phase_stats[phase] = {
            "duration_min": round(len(rows) / 60.0, 2),
            "avg_power_kW": round(sum(abs(_f(r.get("P_shaft_required_W", r["P_shaft_W"]))) for r in rows) / max(len(rows), 1) / 1000.0, 2),
            "avg_engine_kW": round(sum(_f(r.get("P_engine_W", 0.0)) for r in rows) / max(len(rows), 1) / 1000.0, 2),
            "avg_battery_kW": round(sum(_f(r.get("P_battery_W", 0.0)) for r in rows) / max(len(rows), 1) / 1000.0, 2),
            "avg_supercap_kW": round(sum(_f(r.get("P_supercap_W", 0.0)) for r in rows) / max(len(rows), 1) / 1000.0, 2),
            "fuel_burn_kg": round(_f(rows[-1]["fuel_burned_kg"]) - _f(rows[0]["fuel_burned_kg"]), 3),
            "soc_drop_pct": round(_f(rows[0]["soc_pct"]) - _f(rows[-1]["soc_pct"]), 2),
            "architecture": max({r.get("architecture", "SERIES") for r in rows}, key=[r.get("architecture", "SERIES") for r in rows].count),
            "mode": max({r.get("mode", "SERIES") for r in rows}, key=[r.get("mode", "SERIES") for r in rows].count),
        }
    architecture_counts = {}
    for r in records:
        architecture_counts[r.get("architecture", "SERIES")] = architecture_counts.get(r.get("architecture", "SERIES"), 0) + 1
    architecture_share = {
        name: round(count / max(len(records), 1) * 100.0, 1)
        for name, count in architecture_counts.items()
    }
    regen_energy_kWh = sum(_f(r.get("P_regen_W", 0.0)) for r in records) / 3600.0 / 1000.0
    supercap_peak_kW = max(abs(_f(r.get("P_supercap_W", 0.0))) for r in records) / 1000.0
    avg_ecms_s = sum(_f(r.get("ecms_s", 0.0)) for r in records) / max(len(records), 1)
    stall_min_margin = min(_f(r.get("stall_margin", 999.0)) for r in records)
    max_cl = max(_f(r.get("CL", 0.0)) for r in records)
    avg_ld = sum(_f(r.get("CL", 0.0)) / max(_f(r.get("CD", 1e-9)), 1e-9) for r in records) / max(len(records), 1)
    conventional_fuel_kg = 25.68 if p.get("mission_name") == "mission1" else None
    fuel_reduction_pct = (
        (conventional_fuel_kg - fuel_burned) / conventional_fuel_kg * 100.0
        if conventional_fuel_kg else None
    )
    return {
        "mission_name": p.get("mission_name", "default"),
        "mission_time_min": round(mission_time_min, 2),
        "fuel_burned_kg": round(fuel_burned, 3),
        "conventional_fuel_kg": round(conventional_fuel_kg, 3) if conventional_fuel_kg else None,
        "fuel_reduction_pct": round(fuel_reduction_pct, 2) if fuel_reduction_pct is not None else None,
        "fuel_remaining_kg": round(_f(final["fuel_remaining_kg"]), 4),
        "final_soc_pct": round(_f(final["soc_pct"]), 3),
        "min_soc_pct": round(min(_f(r["soc_pct"]) for r in records), 3),
        "final_mass_kg": round(_f(final["mass_kg"]), 3),
        "shaft_energy_kWh": round(sum(max(_f(r["P_shaft_W"]), 0.0) for r in records) / 3600.0 / 1000.0, 3),
        "regen_energy_kWh": round(regen_energy_kWh, 3),
        "avg_shaft_power_kW": round(sum(abs(_f(r["P_shaft_W"])) for r in records) / max(len(records), 1) / 1000.0, 2),
        "max_shaft_power_kW": round(max(abs(_f(r["P_shaft_W"])) for r in records) / 1000.0, 2),
        "supercap_peak_kW": round(supercap_peak_kW, 2),
        "engine_alone_cruise_deficit_kW": round(
            max((_f(r.get("P_engine_alone_deficit_W", 0.0)) for r in records if r["phase"] == "CRUISE"), default=0.0) / 1000.0,
            2,
        ),
        "avg_ecms_s": round(avg_ecms_s, 3),
        "min_stall_margin": round(stall_min_margin, 3),
        "max_CL": round(max_cl, 3),
        "avg_LD": round(avg_ld, 2),
        "distance_km": round(_f(records[-1].get("distance_m", 0.0)) / 1000.0, 2),
        "architecture_share": architecture_share,
        "estimated_mass_kg": round(mass, 2),
        "mass_margin_kg": round(p["mtow_kg"] - mass, 2),
        "violations": violations,
        "mission_feasible": mission_feasible,
        "mass_feasible": mass_ok,
        "feasible": feasible,
        "score": round(score, 3),
        "phase_stats": phase_stats,
        "assumptions": {
            "initial_soc_pct": round(p["initial_soc"] * 100.0, 1),
            "fuel_start_kg": round(p["fuel_start_kg"], 2),
            "fuel_capacity_kg": round(p["fuel_capacity_kg"], 2),
            "engine": "60 kW-class turboshaft BSFC part-load model" if p["engine_rated_W"] == 60000.0 else "scaled turboshaft BSFC model",
            "bsfc_scale": round(p.get("bsfc_scale", 1.0), 4),
            "aero_cd0_used": round(p["cd0"], 4),
        },
    }


@app.route("/")
def index():
    return send_from_directory(str(ROOT / "dashboard"), "index.html")


@app.route("/api/data/timeseries")
def get_timeseries():
    """Return the pre-computed baseline simulation time-series."""
    # Try Mission 1 new output first, fall back to old path
    for candidate in [
        ROOT / "data" / "outputs" / "mission1" / "mission1_timeseries.json",
        ROOT / "data" / "outputs" / "mission_timeseries.json",
    ]:
        if candidate.exists():
            with open(candidate) as f:
                data = json.load(f)
            return jsonify(data)
    return jsonify({"error": "No timeseries data found"}), 404


@app.route("/api/mission1/timeseries")
def get_mission1_timeseries():
    """Return Mission 1 pre-computed full timeseries (6427 steps)."""
    json_path = ROOT / "data" / "outputs" / "mission1" / "mission1_timeseries.json"
    if not json_path.exists():
        return jsonify({"error": "Mission 1 timeseries not found. Run mission1_sim.py first."}), 404
    with open(json_path) as f:
        data = json.load(f)
    # Thin to every 5th step for fast transfer (~1285 points)
    return jsonify(data[::5])


@app.route("/api/mission1/timeseries/full")
def get_mission1_timeseries_full():
    """Return Mission 1 timeseries without thinning (for final result mode)."""
    json_path = ROOT / "data" / "outputs" / "mission1" / "mission1_timeseries.json"
    if not json_path.exists():
        return jsonify({"error": "Mission 1 timeseries not found."}), 404
    with open(json_path) as f:
        data = json.load(f)
    return jsonify(data)


@app.route("/api/mission1/summary")
def get_mission1_summary():
    """Return Mission 1 final summary metrics."""
    json_path = ROOT / "data" / "outputs" / "mission1" / "mission1_summary.json"
    if not json_path.exists():
        return jsonify({"error": "Mission 1 summary not found."}), 404
    with open(json_path) as f:
        data = json.load(f)
    return jsonify(data)



@app.route("/api/data/sweep")
def get_sweep():
    """Return the Phase 6 sweep results."""
    csv_path = ROOT / "data" / "outputs" / "sweep_results.csv"
    import csv as csvmod
    rows = []
    with open(csv_path) as f:
        reader = csvmod.DictReader(f)
        for row in reader:
            # Convert numeric strings
            parsed = {}
            for k, v in row.items():
                try:
                    parsed[k] = float(v)
                except (ValueError, TypeError):
                    if v.lower() == "true":
                        parsed[k] = True
                    elif v.lower() == "false":
                        parsed[k] = False
                    else:
                        parsed[k] = v
            rows.append(parsed)
    return jsonify(rows)


@app.route("/api/data/optimised")
def get_optimised():
    """Return the recommended optimised configuration."""
    path = ROOT / "data" / "outputs" / "optimised_config.json"
    with open(path) as f:
        data = json.load(f)
    return jsonify(data)


@app.route("/api/simulate", methods=["POST"])
def simulate():
    """
    Re-run the simulation with user-tweaked parameters.
    
    Expected JSON body:
      { "bat_kWh": 55, "cruise_duration_min": 30, "loiter_duration_min": 20 }
    """
    params = request.json or {}

    p = _physical_params(params)
    if "bat_kWh" in params:
        p["battery_capacity_kWh"] = _float_param(params, "bat_kWh", p["battery_capacity_kWh"])

    try:
        records = _run_dashboard_sim(p)
        # Thin the data for fast transfer (every 10th second)
        thinned = records[::10]
        return jsonify({
            "status": "ok",
            "data": thinned,
            "summary": _summarize_candidate(records, p),
            "physical": p,
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/api/mission-optimize", methods=["POST"])
def mission_optimize():
    """
    Sweep a compact design space around the edited physical parameters and
    return the best feasible hybrid configuration for the fixed mission.
    """
    base = _physical_params(request.json or {})
    engine_scales = [0.85, 1.0, 1.15, 1.30]
    battery_scales = [0.75, 1.0, 1.25, 1.50]
    fuel_scales = [0.80, 1.0, 1.20, 1.40]
    candidates = []
    best = None

    try:
        for e_scale in engine_scales:
            for b_scale in battery_scales:
                for f_scale in fuel_scales:
                    p = dict(base)
                    p["engine_rated_W"] = max(30000.0, base["engine_rated_W"] * e_scale)
                    p["battery_capacity_kWh"] = max(10.0, base["battery_capacity_kWh"] * b_scale)
                    p["fuel_start_kg"] = max(5.0, base["fuel_start_kg"] * f_scale)
                    records = _run_dashboard_sim(p)
                    summary = _summarize_candidate(records, p)
                    summary["engine_kW"] = round(p["engine_rated_W"] / 1000.0, 1)
                    summary["battery_kWh"] = round(p["battery_capacity_kWh"], 1)
                    summary["fuel_start_kg"] = round(p["fuel_start_kg"], 1)
                    candidates.append(summary)
                    if best is None:
                        best = {"params": p, "summary": summary, "records": records}
                    else:
                        best_feasible = best["summary"]["feasible"]
                        if (
                            (summary["feasible"] and not best_feasible) or
                            (summary["feasible"] == best_feasible and summary["score"] > best["summary"]["score"])
                        ):
                            best = {"params": p, "summary": summary, "records": records}

        candidates = sorted(candidates, key=lambda x: (not x["feasible"], -x["score"]))[:24]
        return jsonify({
            "status": "ok",
            "best": {
                "physical": best["params"],
                "summary": best["summary"],
                "data": best["records"][::10],
            },
            "candidates": candidates,
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


if __name__ == "__main__":
    host = os.environ.get("AEROTHON_HOST", "127.0.0.1")
    port = int(os.environ.get("AEROTHON_PORT", "5050"))
    print("\n  Hybrid-Electric UAV Dashboard")
    print(f"  http://{host}:{port}\n")
    app.run(debug=False, host=host, port=port)
