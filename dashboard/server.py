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

app = Flask(__name__, static_folder=str(ROOT / "dashboard"))


@app.route("/")
def index():
    return send_from_directory(str(ROOT / "dashboard"), "index.html")


@app.route("/api/data/timeseries")
def get_timeseries():
    """Return the pre-computed baseline simulation time-series."""
    json_path = ROOT / "data" / "outputs" / "mission_timeseries.json"
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

    bat_kWh   = float(params.get("bat_kWh", 55.0))
    cruise_m  = float(params.get("cruise_duration_min", 30.0))
    loiter_m  = float(params.get("loiter_duration_min", 20.0))
    cruise_alt = float(params.get("cruise_alt_m", 5000.0))

    try:
        records = run_sim(
            cruise_duration_s=cruise_m * 60.0,
            loiter_duration_s=loiter_m * 60.0,
            cruise_alt_m=cruise_alt,
            initial_soc=0.90,
            fuel_start_kg=20.0,
            verbose=False,
            export_csv=False,
            export_json=False,
        )
        # Thin the data for fast transfer (every 10th second)
        thinned = records[::10]
        return jsonify({"status": "ok", "data": thinned})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


if __name__ == "__main__":
    print("\n  Hybrid-Electric UAV Dashboard")
    print("  http://localhost:5000\n")
    app.run(debug=False, port=5000)
