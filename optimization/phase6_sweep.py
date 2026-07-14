"""
Phase 6 — Optimization & Design Trade-Offs
==========================================
Parametric sweep over:
  1. Battery capacity  (20 – 80 kWh)
  2. Engine derating   (40 – 80 kW)
  3. Motor topology    (single vs twin)

Objectives (minimise / maximise):
  - Fuel burn (kg)          [minimise]
  - Total propulsion mass   [minimise]
  - Loiter endurance (min)  [maximise]

Constraints:
  - MTOW <= 1000 kg
  - Payload = 200 kg (fixed)
  - SoC in [20%, 90%] at all times
  - Battery C-rate <= 3.0 C continuous

Outputs:
  - data/outputs/sweep_results.csv
  - Trade-study PNG plots saved to data/outputs/
"""

import sys, json, math, itertools, csv
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import matplotlib
matplotlib.use("Agg")            # headless — no GUI window
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# ── subsystem models ─────────────────────────────────────────────────────────
from config import AIRCRAFT, AERO, ENGINE, BATTERY, FUEL_LHV, G, MASS
from simulation.atmosphere    import isa_density
from simulation.aerodynamics  import required_shaft_power, best_endurance_speed
from simulation.engine_model  import TurboshaftEngine
from simulation.battery_model import BatteryPack
from simulation.power_electronics import PowerElectronicsChain
from simulation.mission_profile   import build_mission_profile
from simulation.controller        import PowerSplitController

OUT = Path(__file__).parent.parent / "data" / "outputs"
OUT.mkdir(parents=True, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
PAYLOAD_KG       = 200.0
AIRFRAME_KG      = 300.0
SPEC_ENG_KW_KG   = 2.0      # kW/kg installed
SPEC_GEN_KW_KG   = 5.0      # kW/kg
SPEC_MOT_KW_KG   = 5.0      # kW/kg continuous
SPEC_BAT_WH_KG   = 200.0    # Wh/kg pack-level
BAT_SOC_WINDOW   = 0.70     # usable DoD (20-90%)
TOTAL_FUEL_KG    = 20.0     # starting fuel [conservative upper bound]
PE_MASS_KG       = 12.0     # rectifier + inverters (fixed estimate)
DT               = 1.0      # timestep [s]
MTOW_LIMIT       = 1000.0


# ─────────────────────────────────────────────────────────────────────────────
# FAST SINGLE-RUN EVALUATOR
# ─────────────────────────────────────────────────────────────────────────────
def evaluate_config(bat_kWh: float, eng_kW: float, twin_motors: bool,
                    loiter_extra_min: float = 0.0) -> dict | None:
    """
    Run a fast mission sim for one design point.

    Returns a result dict, or None if the design violates MTOW or SoC.
    """
    # ── Mass breakdown ────────────────────────────────────────────────
    m_eng  = eng_kW / SPEC_ENG_KW_KG
    m_gen  = eng_kW / SPEC_GEN_KW_KG
    m_bat  = (bat_kWh * 1000.0) / SPEC_BAT_WH_KG
    n_mot  = 2 if twin_motors else 1
    # Total motor kW = climb peak demand (~111 kW), split over n motors
    mot_kW_each = 120.0 / n_mot          # always size for 111-kW peak
    m_mot  = (mot_kW_each * n_mot) / SPEC_MOT_KW_KG
    m_prop = PAYLOAD_KG + AIRFRAME_KG + m_eng + m_gen + m_bat + m_mot + PE_MASS_KG

    # Fuel estimate: run sim to get it; bootstrap with Phase-3 value
    fuel_init = TOTAL_FUEL_KG
    m_total   = m_prop + fuel_init

    if m_total > MTOW_LIMIT:
        return None           # Hard constraint violation

    # ── Simulation ───────────────────────────────────────────────────
    eng  = TurboshaftEngine(eng_kW * 1000.0)
    bat  = BatteryPack(bat_kWh, initial_soc=0.90)
    pe   = PowerElectronicsChain()
    ctrl = PowerSplitController(eng_kW * 1000.0, bat_kWh)

    segs = build_mission_profile(
        loiter_duration_s=(20.0 + loiter_extra_min) * 60.0
    )

    W_N          = AIRCRAFT.mtow_kg * G
    fuel_remain  = fuel_init
    soc_violated = False
    crate_max    = 0.0
    t            = 0.0
    loiter_min   = 0.0
    total_shaft_J = 0.0

    for seg in segs:
        n_steps = int(seg.duration_s / DT)
        for step in range(n_steps):
            frac = step / max(n_steps - 1, 1)
            h = seg.h_start_m + (seg.h_end_m - seg.h_start_m) * frac

            P_shaft = required_shaft_power(seg.V_ms, h, W_N, seg.roc_ms, seg.eta_prop)
            P_shaft = max(P_shaft, 0.0)

            split = ctrl.compute_split(
                seg.name, P_shaft,
                pe.eta_eng_to_shaft, pe.eta_bat_to_shaft,
                bat.soc
            )
            eng_out = eng.step(split["P_engine_cmd_W"], DT)
            bat_out = bat.step(split["P_bat_elec_W"], DT)

            fuel_remain = max(0.0, fuel_remain - eng_out["fuel_burn_kg"])
            crate_max   = max(crate_max, bat_out["c_rate"])

            if bat.soc <= BATTERY.soc_min + 0.001:
                soc_violated = True   # let it finish but flag

            if seg.name == "LOITER":
                loiter_min += DT / 60.0

            P_shaft_act = (pe.engine_to_prop_shaft(eng_out["P_engine_W"]) +
                           pe.bat_to_prop_shaft(max(split["P_bat_elec_W"], 0.0)))
            total_shaft_J += P_shaft_act * DT
            t += DT

    fuel_burned = fuel_init - fuel_remain
    m_fuel_final = fuel_burned
    m_total_final = m_prop + m_fuel_final

    return {
        "bat_kWh":       bat_kWh,
        "eng_kW":        eng_kW,
        "twin_motors":   twin_motors,
        "m_engine_kg":   round(m_eng, 2),
        "m_gen_kg":      round(m_gen, 2),
        "m_bat_kg":      round(m_bat, 2),
        "m_motor_kg":    round(m_mot, 2),
        "m_pe_kg":       PE_MASS_KG,
        "m_prop_kg":     round(m_prop, 2),
        "m_fuel_kg":     round(m_fuel_final, 2),
        "mtow_kg":       round(m_total_final, 2),
        "mtow_margin_kg": round(MTOW_LIMIT - m_total_final, 2),
        "fuel_burned_kg": round(fuel_burned, 3),
        "final_soc_pct": round(bat.soc * 100.0, 2),
        "loiter_min":    round(loiter_min, 2),
        "crate_max":     round(crate_max, 3),
        "soc_violated":  soc_violated,
        "mtow_violated": m_total_final > MTOW_LIMIT,
        "shaft_energy_kWh": round(total_shaft_J / 3.6e6, 3),
    }


# ─────────────────────────────────────────────────────────────────────────────
# PARAMETRIC SWEEP
# ─────────────────────────────────────────────────────────────────────────────
BAT_RANGE = np.arange(20.0, 85.0, 5.0)    # 20, 25, 30 … 80 kWh
ENG_RANGE = np.arange(40.0, 85.0, 5.0)    # 40, 45 … 80 kW
MOTOR_OPT = [True, False]                  # Twin vs Single

print("Running parametric sweep…")
print(f"  Battery: {BAT_RANGE[0]:.0f}–{BAT_RANGE[-1]:.0f} kWh "
      f"({len(BAT_RANGE)} pts)")
print(f"  Engine:  {ENG_RANGE[0]:.0f}–{ENG_RANGE[-1]:.0f} kW "
      f"({len(ENG_RANGE)} pts)")
print(f"  Motors:  Twin + Single")
total_runs = len(BAT_RANGE) * len(ENG_RANGE) * len(MOTOR_OPT)
print(f"  Total design points: {total_runs}\n")

results = []
for bat, eng, twin in itertools.product(BAT_RANGE, ENG_RANGE, MOTOR_OPT):
    r = evaluate_config(bat, eng, twin)
    if r is not None:
        results.append(r)

print(f"  Feasible designs evaluated: {len(results)}")

# ── Write sweep CSV ──────────────────────────────────────────────────
sweep_csv = OUT / "sweep_results.csv"
if results:
    with open(sweep_csv, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=results[0].keys())
        wr.writeheader()
        wr.writerows(results)
    print(f"  Sweep CSV saved -> {sweep_csv}\n")


# ─────────────────────────────────────────────────────────────────────────────
# PARETO FRONT  (Minimise fuel burn  vs  Minimise propulsion mass)
# ─────────────────────────────────────────────────────────────────────────────
def is_pareto(costs):
    """Return boolean mask of Pareto-optimal rows (2D minimisation)."""
    n = len(costs)
    is_eff = np.ones(n, dtype=bool)
    for i in range(n):
        if not is_eff[i]:
            continue
        for j in range(n):
            if i == j or not is_eff[j]:
                continue
            if (costs[j, 0] <= costs[i, 0] and costs[j, 1] <= costs[i, 1] and
                    (costs[j, 0] < costs[i, 0] or costs[j, 1] < costs[i, 1])):
                is_eff[i] = False
                break
    return is_eff


feasible = [r for r in results if not r["soc_violated"] and not r["mtow_violated"]]
print(f"  Designs passing ALL constraints: {len(feasible)}")

if not feasible:
    print("No feasible designs found — check constraints.")
    sys.exit(1)

costs = np.array([[r["fuel_burned_kg"], r["m_prop_kg"]] for r in feasible])
pareto_mask = is_pareto(costs)
pareto_pts  = [feasible[i] for i in range(len(feasible)) if pareto_mask[i]]
pareto_pts.sort(key=lambda r: r["fuel_burned_kg"])

print(f"  Pareto-optimal designs: {len(pareto_pts)}\n")

# ─────────────────────────────────────────────────────────────────────────────
# PLOT 1 — Pareto Front: Fuel Burn vs Propulsion Mass
# ─────────────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 6))
ax.scatter(costs[:, 0], costs[:, 1],
           c="#aac8e8", s=18, alpha=0.5, label="Feasible designs")

px = [r["fuel_burned_kg"] for r in pareto_pts]
py = [r["m_prop_kg"]       for r in pareto_pts]
ax.plot(px, py, "ro-", lw=2, ms=7, label="Pareto front")

# Annotate recommended point (closest to utopia)
utopia = np.array([min(px), min(py)])
dists  = [np.linalg.norm(np.array([px[i], py[i]]) - utopia) for i in range(len(px))]
best_i = int(np.argmin(dists))
best   = pareto_pts[best_i]
ax.annotate(
    f"  Recommended\n  Bat={best['bat_kWh']:.0f} kWh | Eng={best['eng_kW']:.0f} kW",
    xy=(px[best_i], py[best_i]),
    xytext=(px[best_i] + 0.4, py[best_i] - 12),
    fontsize=9, color="darkred",
    arrowprops=dict(arrowstyle="->", color="darkred"),
)
ax.set_xlabel("Total Fuel Burned (kg)")
ax.set_ylabel("Propulsion System Mass (kg)")
ax.set_title("Pareto Front: Fuel Burn vs Propulsion Mass")
ax.legend()
ax.grid(True, alpha=0.4)
plt.tight_layout()
plt.savefig(OUT / "pareto_fuel_vs_mass.png", dpi=150)
plt.close()
print("  Plot saved: pareto_fuel_vs_mass.png")


# ─────────────────────────────────────────────────────────────────────────────
# PLOT 2 — Sensitivity: Battery Capacity vs Fuel Burn (at nominal 60 kW engine)
# ─────────────────────────────────────────────────────────────────────────────
eng_nom = 60.0
twin_data   = sorted([r for r in feasible if r["eng_kW"] == eng_nom and     r["twin_motors"]], key=lambda r: r["bat_kWh"])
single_data = sorted([r for r in feasible if r["eng_kW"] == eng_nom and not r["twin_motors"]], key=lambda r: r["bat_kWh"])

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

for ax, data, label in [(axes[0], twin_data, "Twin Motors"),
                        (axes[1], single_data, "Single Motor")]:
    if not data:
        continue
    bats  = [r["bat_kWh"]        for r in data]
    fuels = [r["fuel_burned_kg"] for r in data]
    mass  = [r["m_prop_kg"]      for r in data]
    soc_f = [r["final_soc_pct"]  for r in data]

    l1, = ax.plot(bats, fuels, "b-o", ms=5, label="Fuel burned (kg)")
    ax.set_xlabel("Battery Capacity (kWh)")
    ax.set_ylabel("Fuel Burned (kg)", color="b")
    ax.tick_params(axis="y", labelcolor="b")

    ax2 = ax.twinx()
    l2, = ax2.plot(bats, mass, "r--s", ms=5, label="Propulsion mass (kg)")
    ax2.set_ylabel("Propulsion Mass (kg)", color="r")
    ax2.tick_params(axis="y", labelcolor="r")

    ax.set_title(f"Battery Sensitivity — {label} (Engine {eng_nom:.0f} kW)")
    ax.legend(handles=[l1, l2], loc="upper left")
    ax.grid(True, alpha=0.4)

plt.tight_layout()
plt.savefig(OUT / "sensitivity_battery.png", dpi=150)
plt.close()
print("  Plot saved: sensitivity_battery.png")


# ─────────────────────────────────────────────────────────────────────────────
# PLOT 3 — Sensitivity: Engine Derating vs Fuel Burn (at nominal 55 kWh battery)
# ─────────────────────────────────────────────────────────────────────────────
bat_nom = 55.0
eng_data = sorted([r for r in feasible if abs(r["bat_kWh"] - bat_nom) < 1.0
                   and r["twin_motors"]], key=lambda r: r["eng_kW"])

fig, ax = plt.subplots(figsize=(8, 5))
if eng_data:
    engs  = [r["eng_kW"]         for r in eng_data]
    fuels = [r["fuel_burned_kg"] for r in eng_data]
    mass  = [r["m_prop_kg"]      for r in eng_data]
    crate = [r["crate_max"]      for r in eng_data]

    ax.plot(engs, fuels, "b-o", ms=6, label="Fuel burned (kg)")
    ax.set_xlabel("Engine Rating (kW)")
    ax.set_ylabel("Fuel Burned (kg)", color="b")
    ax.tick_params(axis="y", labelcolor="b")

    ax2 = ax.twinx()
    ax2.plot(engs, crate, "g--^", ms=6, label="Peak C-rate")
    ax2.set_ylabel("Peak Battery C-Rate", color="g")
    ax2.tick_params(axis="y", labelcolor="g")
    ax2.axhline(3.0, color="g", linestyle=":", alpha=0.7, label="3.0C limit")

    ax.set_title(f"Engine Derating Sensitivity (Battery {bat_nom:.0f} kWh, Twin Motors)")
    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, loc="upper left")
    ax.grid(True, alpha=0.4)
    plt.tight_layout()
    plt.savefig(OUT / "sensitivity_engine.png", dpi=150)
    plt.close()
    print("  Plot saved: sensitivity_engine.png")


# ─────────────────────────────────────────────────────────────────────────────
# RECOMMENDED CONFIGURATION (Pareto knee point)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("  RECOMMENDED OPTIMISED CONFIGURATION")
print("=" * 60)
for k, v in best.items():
    print(f"  {k:<25} {v}")
print("=" * 60)

# Save as JSON for dashboard reference
with open(OUT / "optimised_config.json", "w") as f:
    json.dump(best, f, indent=2)
print(f"\n  Recommended config saved -> {OUT}/optimised_config.json")
