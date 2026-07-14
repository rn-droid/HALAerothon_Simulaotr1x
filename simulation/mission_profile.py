"""
Mission Profile Generator.

Builds the time-stepped mission sequence (6 phases) with altitude,
airspeed, ROC, and propeller efficiency at each second.

Cross-references: MIS-01 through MIS-09 in assumption_register.md
"""

from dataclasses import dataclass, field
from typing import List
import math

from config import AIRCRAFT, AERO, MISSION, G
from simulation.aerodynamics import best_endurance_speed


@dataclass
class MissionSegment:
    """A single mission phase descriptor."""
    name: str
    duration_s: float
    h_start_m: float
    h_end_m: float
    V_ms: float          # True airspeed [m/s]
    roc_ms: float        # Rate of climb (+) or descent (-) [m/s]
    eta_prop: float      # Propeller efficiency [—]
    engine_on: bool      # Whether engine is commanded on
    regen: bool = False  # Regenerative descent flag


def build_mission_profile(
    cruise_duration_s: float = None,
    loiter_duration_s: float = None,
    cruise_alt_m: float = None,
    loiter_alt_m: float = 3000.0,
    climb_rate_ms: float = None,
    descent_rate_ms: float = None,
) -> List[MissionSegment]:
    """
    Build the 6-phase mission profile.

    All parameters are optional and default to the config values,
    enabling dashboard-driven parameter sweeps.

    Returns:
        List[MissionSegment] in mission order.
    """
    cruise_dur  = cruise_duration_s  or MISSION.cruise_duration_s
    loiter_dur  = loiter_duration_s  or MISSION.loiter_duration_s
    h_cr        = cruise_alt_m       or AIRCRAFT.cruise_alt_m
    roc         = climb_rate_ms      or MISSION.climb_rate_ms
    rod         = descent_rate_ms    or MISSION.descent_rate_ms

    h_to        = MISSION.takeoff_transition_alt_m   # 50 m
    V_cr        = AIRCRAFT.cruise_speed_ms           # 69.44 m/s
    V_cl        = 45.0   # m/s  — climb airspeed (slower than cruise)
    V_des       = 45.0   # m/s  — descent airspeed
    W_N         = AIRCRAFT.mtow_kg * G

    # Loiter at best-endurance speed
    V_lo = best_endurance_speed(loiter_alt_m, W_N)

    t_cl  = (h_cr - h_to) / roc          # Climb duration [s]
    t_des = (loiter_alt_m - 0.0) / rod   # Descent from loiter alt [s]

    prop = AERO.prop_efficiency

    segments = [
        MissionSegment(
            name="TAKEOFF",
            duration_s=30.0,
            h_start_m=0.0, h_end_m=h_to,
            V_ms=V_cl * 0.70,   # Slower during roll/rotation
            roc_ms=h_to / 30.0,  # Gain 50 m in 30 s
            eta_prop=0.60,
            engine_on=True,
        ),
        MissionSegment(
            name="CLIMB",
            duration_s=t_cl,
            h_start_m=h_to, h_end_m=h_cr,
            V_ms=V_cl,
            roc_ms=roc,
            eta_prop=prop["climb"],
            engine_on=True,
        ),
        MissionSegment(
            name="CRUISE",
            duration_s=cruise_dur,
            h_start_m=h_cr, h_end_m=h_cr,
            V_ms=V_cr,
            roc_ms=0.0,
            eta_prop=prop["cruise"],
            engine_on=True,
        ),
        MissionSegment(
            name="LOITER",
            duration_s=loiter_dur,
            h_start_m=loiter_alt_m, h_end_m=loiter_alt_m,
            V_ms=V_lo,
            roc_ms=0.0,
            eta_prop=prop["loiter"],
            engine_on=False,   # Pure EV loiter (Strategy A)
        ),
        MissionSegment(
            name="DESCENT",
            duration_s=t_des,
            h_start_m=loiter_alt_m, h_end_m=0.0,
            V_ms=V_des,
            roc_ms=-rod,
            eta_prop=0.55,     # Windmilling efficiency
            engine_on=False,
            regen=True,
        ),
        MissionSegment(
            name="LANDING",
            duration_s=30.0,
            h_start_m=0.0, h_end_m=0.0,
            V_ms=V_lo * 0.5,
            roc_ms=0.0,
            eta_prop=prop["loiter"],
            engine_on=False,
        ),
    ]

    return segments


def total_mission_time(segments: List[MissionSegment]) -> float:
    """Total mission duration [s]."""
    return sum(s.duration_s for s in segments)
