"""
Hybrid simulation with series-parallel clutch and ECMS controller.

Power routing
-------------
Parallel mode  (clutch engaged, engine in BSFC sweet-spot):
    Engine drives propeller directly — 100 % engine, 0 % battery.
    No generator / motor conversion losses on the engine path.

Series mode  (clutch disengaged, or ECMS active):
    Engine -> generator -> DC bus -> motor -> propeller.
    ECMS chooses the optimal engine/battery split to minimise
    the equivalent fuel cost.  Falls back to a fixed 70/30 split
    when ecms_s is None (backward-compatible with Task 5/6).

Descent  (always series):
    Engine idle, propeller windmills through generator to recharge
    the battery at up to max charge C-rate.
"""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from sim.aircraft import Aircraft
from sim.engine import Turboshaft
from sim.mission import MissionProfile
from sim.battery import Battery
from sim.motor import ElectricMotor
from sim.clutch import Clutch
from sim.ecms import ECMS
from sim.supercap import Supercapacitor
import numpy as np

# ── Configuration ───────────────────────────────────────────
BATTERY_CAPACITY_KWH = 10.0
MOTOR_RATED_POWER_W = 50_000.0
CRUISE_ALT_M = 3000.0
CRUISE_DURATION_S = 30 * 60
LOITER_DURATION_S = 20 * 60
GENERATOR_EFFICIENCY = 0.93
TRANSIENT_THRESHOLD_W_PER_S = 2000.0
RECOVERY_EFFICIENCY = 0.5


def run_naive_hybrid(
    fault_flag: bool = False,
    ecms_s: float | None = None,
    ecms_controller = None,
    loiter_duration_s: float = LOITER_DURATION_S,
    mtow_kg: float = 1000.0,
    engine_rated_W: float = 60_000.0,
    battery_capacity_kWh: float = BATTERY_CAPACITY_KWH,
    supercap_mass_kg: float = 5.0,
    motor_rated_W: float = MOTOR_RATED_POWER_W,
    regen_efficiency: float = RECOVERY_EFFICIENCY,
) -> pd.DataFrame:
    """
    Run the hybrid simulation over a full mission.

    Parameters
    ----------
    fault_flag : Force clutch to series mode at every timestep.
    ecms_s     : If a float, use ECMS with that equivalence factor.
                 If None, fall back to the fixed 70/30 split.
    ecms_controller : Optional pre-instantiated ECMS object.
    loiter_duration_s : Allow overriding the default loiter duration.
    """
    # ── 1. Initialise components ────────────────────────────
    ac = Aircraft(mtow_kg=mtow_kg)
    eng = Turboshaft(rated_power_W=engine_rated_W)
    bat = Battery(capacity_kWh=battery_capacity_kWh)
    supercap = Supercapacitor(mass_kg=supercap_mass_kg)
    mot = ElectricMotor(rated_power_W=motor_rated_W)
    clutch = Clutch(engine_rated_kW=eng.rated_power_W / 1000.0)

    if ecms_controller is not None:
        ecms = ecms_controller
    else:
        ecms = ECMS(s=ecms_s) if ecms_s is not None else None

    # ── 2. Build mission profile ────────────────────────────
    mp = MissionProfile(
        aircraft=ac,
        cruise_alt_m=CRUISE_ALT_M,
        cruise_duration_s=CRUISE_DURATION_S,
        loiter_duration_s=loiter_duration_s,
    )
    df = mp.build()

    dt_s = 1.0

    # ── 3. Logging arrays ───────────────────────────────────
    soc_log = []
    fuel_flow_log = []
    fuel_burned_log = []
    p_eng_log = []
    p_mot_log = []
    p_bat_log = []
    p_sc_log = []
    mode_log = []

    total_fuel = 0.0
    p_req_prev = df["power_required_W"].iloc[0] if len(df) > 0 else 0.0
    total_regen_kWh = 0.0
    
    df.attrs["feasible"] = True
    df.attrs["fail_phase"] = None
    df.attrs["fail_deficit_W"] = 0.0

    # ── 4. Time-stepped loop ────────────────────────────────
    for step_idx, row in df.iterrows():
        if ecms is not None and hasattr(ecms, "sync_step"):
            ecms.sync_step(step_idx)
            
        p_req_raw = row["power_required_W"]
        
        # Supercapacitor Transient Routing
        p_req_limited = p_req_prev + np.clip(p_req_raw - p_req_prev, -TRANSIENT_THRESHOLD_W_PER_S, TRANSIENT_THRESHOLD_W_PER_S)
        p_sc_mech_req = p_req_raw - p_req_limited
        
        if p_sc_mech_req > 0:
            p_sc_elec_req = p_sc_mech_req / mot.efficiency
        else:
            p_sc_elec_req = p_sc_mech_req * mot.efficiency
            
        p_sc_elec_actual = supercap.step(p_sc_elec_req, dt_s)
        
        if p_sc_elec_actual > 0:
            p_sc_mech_actual = p_sc_elec_actual * mot.efficiency
        else:
            p_sc_mech_actual = p_sc_elec_actual / mot.efficiency
            
        # Main powertrain sees the limited request + any unfulfilled transient
        p_req = p_req_limited + (p_sc_mech_req - p_sc_mech_actual)
        p_req_prev = p_req_limited

        if p_req > 0:
            mode = clutch.select_mode(p_req, eng.rated_power_W, fault_flag)

            if mode == "parallel":
                # ── Parallel: engine 100 %, no battery ──────
                if p_req > eng.rated_power_W:
                    df.attrs["feasible"] = False
                    df.attrs["fail_phase"] = row["phase"]
                    df.attrs["fail_deficit_W"] = p_req - eng.rated_power_W
                    break
                p_eng_actual = p_req
                p_mot_mech = 0.0
                p_bat_actual = 0.0

            elif ecms is not None:
                # ── Series + ECMS ───────────────────────────
                if p_req > mot.rated_power_W:
                    df.attrs["feasible"] = False
                    df.attrs["fail_phase"] = row["phase"]
                    df.attrs["fail_deficit_W"] = p_req - mot.rated_power_W
                    break
                    
                p_eng_opt = ecms.optimal_engine_power(
                    p_req, bat, eng, mot, GENERATOR_EFFICIENCY, dt_s,
                )
                p_bus = p_req / mot.efficiency
                p_bat_needed = p_bus - p_eng_opt * GENERATOR_EFFICIENCY

                # Step battery
                p_bat_actual = bat.step(p_bat_needed, dt_s)

                # Compensate for any battery shortfall
                shortfall = p_bat_needed - p_bat_actual
                p_eng_actual = p_eng_opt
                if abs(shortfall) > 1.0:
                    p_eng_actual += shortfall / GENERATOR_EFFICIENCY
                    
                if p_eng_actual > eng.rated_power_W:
                    df.attrs["feasible"] = False
                    df.attrs["fail_phase"] = row["phase"]
                    df.attrs["fail_deficit_W"] = p_eng_actual - eng.rated_power_W
                    break

                p_mot_mech = (p_eng_actual * GENERATOR_EFFICIENCY
                              + p_bat_actual) * mot.efficiency

            else:
                # ── Series + fixed 70/30 split ──────────────
                if p_req > mot.rated_power_W:
                    df.attrs["feasible"] = False
                    df.attrs["fail_phase"] = row["phase"]
                    df.attrs["fail_deficit_W"] = p_req - mot.rated_power_W
                    break
                    
                p_mot_target = min(p_req * 0.30, mot.rated_power_W)
                p_elec_req = mot.electrical_power_in(p_mot_target)
                p_bat_actual = bat.step(p_elec_req, dt_s)
                p_mot_mech = p_bat_actual * mot.efficiency

                p_eng_share = p_req - p_mot_mech
                p_eng_actual = p_eng_share / (
                    GENERATOR_EFFICIENCY * mot.efficiency
                )
                
                if p_eng_actual > eng.rated_power_W:
                    df.attrs["feasible"] = False
                    df.attrs["fail_phase"] = row["phase"]
                    df.attrs["fail_deficit_W"] = p_eng_actual - eng.rated_power_W
                    break

        else:
            # ── Descent regen (always series) ───────────────
            mode = "series"
            p_eng_actual = 0.0
            p_mot_mech = 0.0

            p_regen_mech = min(-p_req, mot.rated_power_W)
            p_elec_gen = p_regen_mech * GENERATOR_EFFICIENCY * regen_efficiency
            p_bat_actual = bat.step(-p_elec_gen, dt_s)
            total_regen_kWh += (-p_bat_actual * dt_s) / 3600000.0

        # ── Fuel consumption ────────────────────────────────
        m_dot = eng.fuel_flow_kg_s(p_eng_actual)
        total_fuel += m_dot * dt_s

        # ── Log ─────────────────────────────────────────────
        soc_log.append(bat.soc)
        fuel_flow_log.append(m_dot)
        fuel_burned_log.append(total_fuel)
        p_eng_log.append(p_eng_actual)
        p_mot_log.append(p_mot_mech)
        p_bat_log.append(p_bat_actual)
        p_sc_log.append(p_sc_elec_actual)
        mode_log.append(mode)

    # ── 5. Attach results ───────────────────────────────────
    if len(soc_log) < len(df):
        df = df.iloc[:len(soc_log)].copy()

    df["soc"] = soc_log
    df["fuel_flow_kg_s"] = fuel_flow_log
    df["fuel_burned_kg"] = fuel_burned_log
    df["p_eng_W"] = p_eng_log
    df["p_mot_W"] = p_mot_log
    df["p_bat_W"] = p_bat_log
    df["p_sc_W"] = p_sc_log
    df["mode"] = mode_log

    df.attrs["clutch_mass_kg"] = clutch.mass_kg
    df.attrs["battery_mass_kg"] = bat.mass_kg
    df.attrs["motor_mass_kg"] = mot.mass_kg
    df.attrs["total_regen_kWh"] = total_regen_kWh
    df.attrs["supercap_mass_kg"] = supercap.mass_kg

    return df


# ================================================================
# Reporting (unchanged from Task 6 — works with both controllers)
# ================================================================

def generate_report(df: pd.DataFrame):
    """Print summary and generate fuel+SoC / mode-timeline plots."""

    total_fuel = df["fuel_burned_kg"].iloc[-1]
    final_soc = df["soc"].iloc[-1]
    min_soc = df["soc"].min()
    max_soc = df["soc"].max()

    clutch_mass = df.attrs.get("clutch_mass_kg", 0)
    battery_mass = df.attrs.get("battery_mass_kg", 0)
    motor_mass = df.attrs.get("motor_mass_kg", 0)

    n_par = int((df["mode"] == "parallel").sum())
    n_ser = int((df["mode"] == "series").sum())
    n_tot = len(df)
    soc_ok = min_soc >= 0.20 and max_soc <= 0.90

    print(f"\n{'='*62}")
    print("  HYBRID SIMULATION RESULTS")
    print(f"{'='*62}")
    print(f"  Total fuel burned : {total_fuel:.2f} kg")
    print(f"  Final SoC         : {final_soc:.4f}")
    print(f"  Min SoC           : {min_soc:.4f}  (limit 0.20)")
    print(f"  Max SoC           : {max_soc:.4f}  (limit 0.90)")
    print(f"  SoC bounds        : {'[PASS]' if soc_ok else '[FAIL]'}")
    print(f"\n  Clutch mode: parallel {n_par} ({n_par/n_tot*100:.1f}%)"
          f"  /  series {n_ser} ({n_ser/n_tot*100:.1f}%)")
    print(f"\n  Powertrain mass budget:")
    print(f"    Clutch  {clutch_mass:7.2f} kg  |  Battery {battery_mass:7.2f} kg"
          f"  |  Motor {motor_mass:7.2f} kg")
    print(f"    Total electric: {clutch_mass+battery_mass+motor_mass+df.attrs.get('supercap_mass_kg',0):.2f} kg")
    print(f"\n  Descent energy recovered : {df.attrs.get('total_regen_kWh',0):.3f} kWh")
    print(f"{'='*62}")

    report_dir = os.path.join(os.path.dirname(__file__), "..", "report")
    os.makedirs(report_dir, exist_ok=True)
    time_min = df["time_s"] / 60.0

    # ── Plot 1: Fuel + SoC ──────────────────────────────────
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    ax1.plot(time_min, df["fuel_burned_kg"], "r-", lw=2, label="Fuel")
    ax1.set_ylabel("Fuel Burned [kg]")
    ax1.set_title("Hybrid Simulation | Fuel Burn and Battery SoC")
    ax1.grid(True, alpha=0.3); ax1.legend()

    ax2.plot(time_min, df["soc"], "b-", lw=2, label="SoC")
    ax2.axhline(0.20, color="red", ls="--", alpha=0.5, label="SoC Min")
    ax2.axhline(0.90, color="green", ls="--", alpha=0.5, label="SoC Max")
    ax2.set_ylabel("SoC"); ax2.set_xlabel("Time [min]")
    ax2.set_ylim(0.1, 1.0); ax2.grid(True, alpha=0.3); ax2.legend()
    fig1.tight_layout()
    p1 = os.path.join(report_dir, "naive_hybrid_sim.png")
    fig1.savefig(p1, dpi=150, bbox_inches="tight"); plt.close(fig1)
    print(f"\n  Plot: {os.path.abspath(p1)}")

    # ── Plot 2: Mode timeline ───────────────────────────────
    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(14, 6),
                                     sharex=True, height_ratios=[1, 2])
    mode_num = df["mode"].map({"parallel": 1, "series": 0}).values
    for i in range(len(time_min) - 1):
        c = "#27ae60" if mode_num[i] == 1 else "#e67e22"
        ax3.axvspan(time_min.iloc[i], time_min.iloc[i + 1],
                    alpha=0.7, color=c, linewidth=0)
    ax3.set_yticks([0, 1]); ax3.set_yticklabels(["Series", "Parallel"])
    ax3.set_ylabel("Mode")
    ax3.set_title("Clutch Mode Timeline")
    ax3.legend(handles=[Patch(color="#27ae60", label="Parallel"),
                        Patch(color="#e67e22", label="Series")],
               loc="upper right", fontsize=9)

    PHASE_CLR = {"takeoff": "#e74c3c", "climb": "#f39c12",
                 "cruise": "#3498db", "loiter": "#2ecc71",
                 "descent": "#9b59b6"}
    for ph, clr in PHASE_CLR.items():
        m = df["phase"] == ph
        ax4.plot(time_min[m], df.loc[m, "power_required_W"]/1000,
                 color=clr, lw=1.2, label=ph.capitalize())
    lo_kW = 0.80 * 0.85 * 60; hi_kW = 0.80 * 1.15 * 60
    ax4.axhspan(lo_kW, hi_kW, alpha=0.12, color="#27ae60",
                label=f"Parallel band ({lo_kW:.0f}-{hi_kW:.0f} kW)")
    ax4.set_ylabel("Power [kW]"); ax4.set_xlabel("Time [min]")
    ax4.legend(loc="upper right", fontsize=8, ncol=2)
    ax4.grid(True, alpha=0.3)
    fig2.tight_layout()
    p2 = os.path.join(report_dir, "clutch_mode_timeline.png")
    fig2.savefig(p2, dpi=150, bbox_inches="tight"); plt.close(fig2)
    print(f"  Plot: {os.path.abspath(p2)}")

    # ── Plot 3: Power Split Stacked ──────────────────────────
    fig3, ax5 = plt.subplots(figsize=(14, 6))
    
    # We want to stack the sources that meet P_req
    # P_req = P_eng_mech + P_bat_mech + P_sc_mech
    # But for descent regen, P_req is negative, so battery and supercap charge (their output is negative)
    # Let's plot positive components
    
    # Actually p_bat_W and p_sc_W in the df are electrical power.
    # To get mechanical contributions at the propeller, we should use:
    # Series mode: P_eng_mech = P_eng * eta_gen * eta_mot
    #              P_bat_mech = P_bat * eta_mot
    # Parallel mode: P_eng_mech = P_eng
    
    # We already logged p_mot_W which is the mechanical power from the motor!
    # And we logged p_eng_W which is the actual engine shaft power.
    # In parallel mode, p_eng_W is directly the prop power.
    # In series mode, motor power is the prop power.
    # So the total mechanical power delivered to the prop is:
    # If parallel: p_eng_W. If series: p_mot_W.
    # But wait, we want to split P_mot_mech into battery, supercap, and engine (via generator).
    
    p_eng_mech = []
    p_bat_mech = []
    p_sc_mech = []
    
    for i, row in df.iterrows():
        if row["mode"] == "parallel":
            p_eng_mech.append(row["p_eng_W"] / 1000.0)
            p_bat_mech.append(0.0)
            p_sc_mech.append(0.0)
        else:
            # Series mode
            # Engine contributes P_eng * eta_gen * eta_mot
            p_e = row["p_eng_W"] * GENERATOR_EFFICIENCY * 0.94 / 1000.0  # mot.efficiency=0.94
            # Battery contributes P_bat * eta_mot
            p_b = row["p_bat_W"] * 0.94 / 1000.0
            # Supercap contributes P_sc * eta_mot
            p_s = row["p_sc_W"] * 0.94 / 1000.0
            p_eng_mech.append(p_e)
            p_bat_mech.append(p_b)
            p_sc_mech.append(p_s)
            
    ax5.stackplot(time_min, p_eng_mech, p_bat_mech, p_sc_mech,
                  labels=["Engine (Mech)", "Battery (Mech)", "Supercap (Mech)"],
                  colors=["#e74c3c", "#3498db", "#f1c40f"], alpha=0.8)
    
    # Overlay the raw power required trace
    ax5.plot(time_min, df["power_required_W"]/1000, "k--", lw=1.5, label="Power Required")
    
    ax5.set_ylabel("Power [kW]")
    ax5.set_xlabel("Time [min]")
    ax5.set_title("Powertrain Source Stacked Plot")
    ax5.legend(loc="upper left")
    ax5.grid(True, alpha=0.3)
    fig3.tight_layout()
    p3 = os.path.join(report_dir, "power_split_stacked.png")
    fig3.savefig(p3, dpi=150, bbox_inches="tight")
    plt.close(fig3)
    print(f"  Plot: {os.path.abspath(p3)}")


if __name__ == "__main__":
    df = run_naive_hybrid()
    generate_report(df)
