"""Tests for sim.mission.MissionProfile."""

import sys
import os
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import matplotlib
matplotlib.use("Agg")           # non-interactive backend for saving
import matplotlib.pyplot as plt

from sim.mission import MissionProfile


def test_mission_power_profile():
    """
    Build a full mission (3000 m cruise, 30 min cruise, 20 min loiter)
    and validate power behaviour across all phases.
    """
    mp = MissionProfile(
        cruise_alt_m=3000.0,
        cruise_duration_s=30 * 60,   # 30 min
        loiter_duration_s=20 * 60,   # 20 min
    )
    df = mp.build()

    # ── Per-phase power series ──────────────────────────────
    phase_power = {}
    for name in ["takeoff", "climb", "cruise", "loiter", "descent"]:
        phase_power[name] = df.loc[df["phase"] == name, "power_required_W"]

    # ── Print summary table ─────────────────────────────────
    print(f"\n{'='*65}")
    print("  Mission Power Profile Summary")
    print(f"{'='*65}")
    total_time = df["time_s"].iloc[-1]
    print(f"  Total mission time : {total_time:.0f} s  ({total_time/60:.1f} min)")
    print(f"  Total data points  : {len(df)}")
    print(f"  {'Phase':10s}  {'Dur[s]':>6s}  {'Mean[kW]':>9s}  "
          f"{'Max[kW]':>8s}  {'Min[kW]':>8s}")
    print(f"  {'-'*10}  {'-'*6}  {'-'*9}  {'-'*8}  {'-'*8}")
    for name in ["takeoff", "climb", "cruise", "loiter", "descent"]:
        p = phase_power[name]
        print(f"  {name:10s}  {len(p):6d}  {p.mean()/1000:9.1f}  "
              f"{p.max()/1000:8.1f}  {p.min()/1000:8.1f}")
    print(f"{'='*65}")

    # Also print key derived quantities
    V_loiter = mp.best_LD_speed(3000.0)
    CL_opt = math.sqrt(mp.ac.CD0 / mp.ac.k)
    LD_max = CL_opt / (2 * mp.ac.CD0)
    print(f"\n  Best L/D speed at 3000 m : {V_loiter:.1f} m/s ({V_loiter*3.6:.0f} km/h)")
    print(f"  CL_opt = sqrt(CD0/k)     : {CL_opt:.4f}")
    print(f"  (L/D)_max                : {LD_max:.1f}")

    # ── ASSERTION 1: Climb has the highest peak power ───────
    # During climb, T = D + W sin(gamma).  The W sin(gamma)
    # term at 5 m/s ROC adds ~68 kW on top of cruise drag
    # power, making climb the most demanding sustained phase.
    max_climb = phase_power["climb"].max()
    for name in ["takeoff", "cruise", "loiter", "descent"]:
        assert max_climb > phase_power[name].max(), (
            f"Climb max ({max_climb/1000:.1f} kW) should exceed "
            f"{name} max ({phase_power[name].max()/1000:.1f} kW)"
        )
    print(f"\n  [PASS] Climb peak power ({max_climb/1000:.1f} kW) is highest")

    # ── ASSERTION 2: Cruise and loiter power are positive ───
    assert (phase_power["cruise"] > 0).all(), "Cruise power must be positive"
    assert (phase_power["loiter"] > 0).all(), "Loiter power must be positive"
    print(f"  [PASS] Cruise & loiter power always positive")

    # ── ASSERTION 3: Descent power is negative or near-zero ─
    # With a 3 m/s sink rate at best-L/D speed, the descent
    # is slightly steeper than the equilibrium glide (where
    # T = 0), so thrust — and therefore power — should be
    # near-zero or negative, indicating a regen opportunity.
    mean_desc = phase_power["descent"].mean()
    assert mean_desc < 1000, (
        f"Mean descent power ({mean_desc:.0f} W) should be "
        f"near-zero or negative (glide / regen)"
    )
    print(f"  [PASS] Mean descent power = {mean_desc/1000:.2f} kW (near-zero/negative)")

    # ── PLOT ────────────────────────────────────────────────
    COLORS = {
        "takeoff": "#e74c3c",
        "climb":   "#f39c12",
        "cruise":  "#3498db",
        "loiter":  "#2ecc71",
        "descent": "#9b59b6",
    }

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8),
                                    sharex=True, height_ratios=[3, 1])

    # Top: power vs time
    for name, color in COLORS.items():
        mask = df["phase"] == name
        ax1.fill_between(
            df.loc[mask, "time_s"] / 60,
            df.loc[mask, "power_required_W"] / 1000,
            alpha=0.25, color=color,
        )
        ax1.plot(
            df.loc[mask, "time_s"] / 60,
            df.loc[mask, "power_required_W"] / 1000,
            color=color, linewidth=1.2, label=name.capitalize(),
        )

    ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax1.axhline(60, color="red", linewidth=0.8, linestyle=":",
                alpha=0.5, label="Engine rated (60 kW)")
    ax1.set_ylabel("Shaft Power Required [kW]", fontsize=12)
    ax1.set_title(
        "Mission Power Profile  |  1000 kg UAV, 3000 m cruise, 250 km/h",
        fontsize=13, fontweight="bold",
    )
    ax1.legend(loc="upper right", fontsize=9, ncol=2)
    ax1.grid(True, alpha=0.3)

    # Bottom: altitude vs time
    for name, color in COLORS.items():
        mask = df["phase"] == name
        ax2.plot(
            df.loc[mask, "time_s"] / 60,
            df.loc[mask, "altitude_m"],
            color=color, linewidth=1.5,
        )
    ax2.set_ylabel("Altitude [m]", fontsize=12)
    ax2.set_xlabel("Mission Time [min]", fontsize=12)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()

    report_dir = os.path.join(os.path.dirname(__file__), "..", "report")
    os.makedirs(report_dir, exist_ok=True)
    save_path = os.path.join(report_dir, "mission_power_profile.png")
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    abs_path = os.path.abspath(save_path)
    print(f"\n  Plot saved to: {abs_path}")
