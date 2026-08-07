"""
Mission 1 analysis plots.

Runs the named Mission 1 scenario and exports engineering plots that expose
the actual physics used by the time-stepped simulator.

Usage:
  python3 -m simulation.mission1_analysis
"""

from pathlib import Path
import csv
import json
import os
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "aerothon_matplotlib"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from simulation.run_simulation import run_sim
from simulation.mission_profile import MISSION1_PRESET


OUT_DIR = Path(__file__).parent.parent / "data" / "outputs"


PHASE_COLORS = {
    "TAKEOFF": "#e84040",
    "CLIMB": "#f5a623",
    "CRUISE": "#3b9eff",
    "LOITER": "#8b6cf7",
    "DESCENT": "#10d48e",
    "LANDING": "#8a8a9a",
}

CONVENTIONAL_FUEL_KG = 25.68


def _series(records, key):
    return [float(r.get(key, 0.0)) for r in records]


def _time_min(records):
    return [float(r["t_s"]) / 60.0 for r in records]


def _shade_phases(ax, records):
    phases = []
    start_idx = 0
    current = records[0]["phase"]
    for idx, row in enumerate(records[1:], start=1):
        if row["phase"] != current:
            phases.append((current, start_idx, idx - 1))
            current = row["phase"]
            start_idx = idx
    phases.append((current, start_idx, len(records) - 1))

    for phase, i0, i1 in phases:
        ax.axvspan(
            records[i0]["t_s"] / 60.0,
            records[i1]["t_s"] / 60.0,
            color=PHASE_COLORS.get(phase, "#888888"),
            alpha=0.08,
            linewidth=0,
        )


def _finish(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def plot_power_split(records):
    t = _time_min(records)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    _shade_phases(ax1, records)
    ax1.plot(t, [p / 1000.0 for p in _series(records, "P_shaft_W")], label="Delivered shaft", color="#f0ece6", linewidth=1.6)
    ax1.plot(t, [p / 1000.0 for p in _series(records, "P_engine_W")], label="Engine", color="#c87941", linewidth=1.2)
    ax1.plot(t, [p / 1000.0 for p in _series(records, "P_battery_W")], label="Battery electric", color="#10d48e", linewidth=1.0)
    ax1.plot(t, [p / 1000.0 for p in _series(records, "P_supercap_W")], label="Supercap", color="#f5a623", linewidth=1.0)
    ax1.plot(t, [-p / 1000.0 for p in _series(records, "P_regen_W")], label="Regen", color="#2dd4bf", linewidth=1.0)
    ax1.axhline(0.0, color="#555555", linewidth=0.8)
    ax1.set_ylabel("Power [kW]")
    ax1.set_title("Mission 1 Power Split and Recovery")
    ax1.grid(alpha=0.25)
    ax1.legend(ncol=5, fontsize=9)

    ax2.plot(t, _series(records, "soc_pct"), label="Battery SoC", color="#10d48e")
    ax2.plot(t, _series(records, "fuel_burned_kg"), label="Fuel burned", color="#8b6cf7")
    ax2.set_ylabel("SoC [%] / fuel [kg]")
    ax2.set_xlabel("Mission time [min]")
    ax2.grid(alpha=0.25)
    ax2.legend(ncol=2, fontsize=9)
    _finish(fig, OUT_DIR / "mission1_power_split.png")


def plot_aero_forces(records):
    t = _time_min(records)
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    _shade_phases(axes[0], records)
    axes[0].plot(t, _series(records, "lift_required_N"), label="Required lift", color="#3b9eff")
    axes[0].plot(t, _series(records, "drag_N"), label="Drag", color="#e84040")
    axes[0].plot(t, _series(records, "thrust_required_N"), label="Required thrust", color="#f5a623")
    axes[0].set_ylabel("Force [N]")
    axes[0].set_title("Mission 1 Force Balance: T - D - W sin(gamma)")
    axes[0].grid(alpha=0.25)
    axes[0].legend(ncol=3, fontsize=9)

    axes[1].plot(t, _series(records, "CL"), label="CL", color="#3b9eff")
    axes[1].plot(t, _series(records, "CD"), label="CD", color="#e84040")
    axes[1].plot(t, _series(records, "CD_induced"), label="Induced CD", color="#8b6cf7")
    axes[1].set_ylabel("Coefficient")
    axes[1].grid(alpha=0.25)
    axes[1].legend(ncol=3, fontsize=9)

    axes[2].plot(t, _series(records, "q_Pa"), label="Dynamic pressure", color="#2dd4bf")
    axes[2].set_ylabel("q [Pa]")
    axes[2].set_xlabel("Mission time [min]")
    axes[2].grid(alpha=0.25)
    axes[2].legend(fontsize=9)
    _finish(fig, OUT_DIR / "mission1_aero_forces.png")


def plot_atmosphere_stall(records):
    t = _time_min(records)
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    _shade_phases(axes[0], records)
    axes[0].plot(t, _series(records, "altitude_m"), color="#3b9eff")
    axes[0].set_ylabel("Altitude [m]")
    axes[0].set_title("Mission 1 Atmosphere, Airspeed, and Stall Constraint")
    axes[0].grid(alpha=0.25)

    axes[1].plot(t, _series(records, "airspeed_ms"), label="True airspeed", color="#f0ece6")
    axes[1].plot(t, _series(records, "ground_speed_ms"), label="Ground speed", color="#c87941")
    axes[1].plot(t, _series(records, "min_safe_speed_ms"), label="1.2 Vs", color="#e84040", linestyle="--")
    axes[1].set_ylabel("Speed [m/s]")
    axes[1].grid(alpha=0.25)
    axes[1].legend(ncol=3, fontsize=9)

    axes[2].plot(t, _series(records, "rho_kg_m3"), label="ISA density", color="#2dd4bf")
    axes[2].plot(t, _series(records, "stall_margin"), label="V/Vs", color="#f5a623")
    axes[2].axhline(1.2, color="#e84040", linestyle="--", linewidth=1.0, label="required margin")
    axes[2].set_ylabel("Density / margin")
    axes[2].set_xlabel("Mission time [min]")
    axes[2].grid(alpha=0.25)
    axes[2].legend(ncol=3, fontsize=9)
    _finish(fig, OUT_DIR / "mission1_atmosphere_stall.png")


def plot_mass_range(records):
    t = _time_min(records)
    fig, ax1 = plt.subplots(figsize=(14, 6))
    _shade_phases(ax1, records)
    ax1.plot(t, _series(records, "mass_kg"), color="#c87941", label="Aircraft mass")
    ax1.set_ylabel("Mass [kg]")
    ax1.set_xlabel("Mission time [min]")
    ax1.grid(alpha=0.25)

    ax2 = ax1.twinx()
    ax2.plot(t, [d / 1000.0 for d in _series(records, "distance_m")], color="#10d48e", label="Ground distance")
    ax2.set_ylabel("Ground distance [km]")

    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [line.get_label() for line in lines], fontsize=9)
    ax1.set_title("Mission 1 Variable Mass and Wind-Aware Ground Range")
    _finish(fig, OUT_DIR / "mission1_mass_range.png")


def plot_fuel_comparison(summary):
    fig, ax = plt.subplots(figsize=(8, 5))
    labels = ["120 kW conventional", "Mission 1 hybrid"]
    values = [CONVENTIONAL_FUEL_KG, summary["fuel_burned_kg"]]
    colors = ["#8a8a9a", "#10d48e"]
    ax.bar(labels, values, color=colors)
    ax.set_ylabel("Fuel burned [kg]")
    ax.set_title(f"Mission 1 Fuel Comparison: {summary['fuel_reduction_pct']:.1f}% reduction")
    for idx, value in enumerate(values):
        ax.text(idx, value + 0.35, f"{value:.2f} kg", ha="center", fontsize=10)
    ax.grid(axis="y", alpha=0.22)
    _finish(fig, OUT_DIR / "mission1_fuel_comparison.png")


def phase_summary(records):
    rows = []
    for phase in ["TAKEOFF", "CLIMB", "CRUISE", "LOITER", "DESCENT", "LANDING"]:
        phase_rows = [r for r in records if r["phase"] == phase]
        if not phase_rows:
            continue
        rows.append({
            "phase": phase,
            "duration_min": round(len(phase_rows) / 60.0, 3),
            "mode": max({r["mode"] for r in phase_rows}, key=[r["mode"] for r in phase_rows].count),
            "architecture": max({r["architecture"] for r in phase_rows}, key=[r["architecture"] for r in phase_rows].count),
            "avg_required_shaft_kW": round(sum(abs(float(r["P_shaft_required_W"])) for r in phase_rows) / len(phase_rows) / 1000.0, 3),
            "avg_delivered_shaft_kW": round(sum(abs(float(r["P_shaft_W"])) for r in phase_rows) / len(phase_rows) / 1000.0, 3),
            "avg_engine_kW": round(sum(float(r["P_engine_W"]) for r in phase_rows) / len(phase_rows) / 1000.0, 3),
            "avg_battery_kW": round(sum(float(r["P_battery_W"]) for r in phase_rows) / len(phase_rows) / 1000.0, 3),
            "avg_supercap_kW": round(sum(float(r["P_supercap_W"]) for r in phase_rows) / len(phase_rows) / 1000.0, 3),
            "fuel_burn_kg": round(float(phase_rows[-1]["fuel_burned_kg"]) - float(phase_rows[0]["fuel_burned_kg"]), 4),
            "soc_start_pct": round(float(phase_rows[0]["soc_pct"]), 3),
            "soc_end_pct": round(float(phase_rows[-1]["soc_pct"]), 3),
            "distance_km": round((float(phase_rows[-1]["distance_m"]) - float(phase_rows[0]["distance_m"])) / 1000.0, 3),
        })
    return rows


def mission_summary(records, phases):
    fuel = float(records[-1]["fuel_burned_kg"])
    return {
        "mission_name": "mission1",
        "mtow_kg": MISSION1_PRESET["mtow_kg"],
        "payload_kg": MISSION1_PRESET["payload_kg"],
        "initial_fuel_kg": MISSION1_PRESET["fuel_start_kg"],
        "battery_capacity_kWh": MISSION1_PRESET["battery_capacity_kWh"],
        "initial_soc_pct": MISSION1_PRESET["initial_soc"] * 100.0,
        "supercap_energy_kWh": MISSION1_PRESET["supercap_energy_kWh"],
        "engine_rated_kW": MISSION1_PRESET["engine_rated_W"] / 1000.0,
        "motor_rated_kW": MISSION1_PRESET["motor_rated_W"] / 1000.0,
        "wing_area_m2": MISSION1_PRESET["wing_area_m2"],
        "aspect_ratio": MISSION1_PRESET["aspect_ratio"],
        "cruise_alt_m": MISSION1_PRESET["cruise_alt_m"],
        "cruise_speed_ms": MISSION1_PRESET["cruise_speed_ms"],
        "mission_time_min": round(float(records[-1]["t_s"]) / 60.0, 3),
        "distance_km": round(float(records[-1]["distance_m"]) / 1000.0, 3),
        "fuel_burned_kg": round(fuel, 4),
        "conventional_fuel_kg": CONVENTIONAL_FUEL_KG,
        "fuel_reduction_pct": round((CONVENTIONAL_FUEL_KG - fuel) / CONVENTIONAL_FUEL_KG * 100.0, 2),
        "final_mass_kg": round(float(records[-1]["mass_kg"]), 3),
        "final_soc_pct": round(float(records[-1]["soc_pct"]), 3),
        "min_soc_pct": round(min(float(r["soc_pct"]) for r in records), 3),
        "min_stall_margin": round(min(float(r["stall_margin"]) for r in records if r["phase"] != "LANDING"), 3),
        "engine_alone_cruise_deficit_kW": round(max(float(r["P_engine_alone_deficit_W"]) for r in records if r["phase"] == "CRUISE") / 1000.0, 3),
        "violations": sum(1 for r in records if r["violation_flags"]),
        "phase_summary": phases,
    }


def write_summaries(records):
    phases = phase_summary(records)
    summary = mission_summary(records, phases)
    with open(OUT_DIR / "mission1_phase_summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=phases[0].keys())
        writer.writeheader()
        writer.writerows(phases)
    with open(OUT_DIR / "mission1_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    records = run_sim(
        mission_name="mission1",
        verbose=False,
        export_csv=True,
        export_json=True,
    )
    summary = write_summaries(records)
    plot_power_split(records)
    plot_aero_forces(records)
    plot_atmosphere_stall(records)
    plot_mass_range(records)
    plot_fuel_comparison(summary)
    print(f"Mission 1 records: {len(records)}")
    print(f"Mission time: {summary['mission_time_min']:.2f} min")
    print(f"Fuel burned: {summary['fuel_burned_kg']:.3f} kg")
    print(f"Fuel reduction vs conventional: {summary['fuel_reduction_pct']:.1f}%")
    print(f"Final SoC: {summary['final_soc_pct']:.2f}%")
    print(f"Minimum SoC: {summary['min_soc_pct']:.2f}%")
    print(f"Minimum stall margin: {summary['min_stall_margin']:.3f}x")
    print(f"Plots saved in: {OUT_DIR}")


if __name__ == "__main__":
    main()
