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
from simulation.atmosphere import isa_density


MISSION1_PRESET = {
    "mission_name": "mission1",
    "mtow_kg": 1000.0,
    "payload_kg": 200.0,
    "fuel_start_kg": 64.0,
    "battery_capacity_kWh": 40.0,
    "initial_soc": 0.90,
    "supercap_energy_kWh": 0.25,
    "engine_rated_W": 60_000.0,
    "bsfc_scale": 0.857,
    "motor_rated_W": 70_000.0,
    "cruise_alt_m": 3000.0,
    "loiter_alt_m": 3000.0,
    "cruise_speed_ms": 69.44,
    "climb_speed_ms": 40.0,
    "climb_rate_ms": 3.0,
    "loiter_speed_ms": 50.0,
    "loiter_bank_deg": 25.0,
    "descent_speed_ms": 45.0,
    "descent_rate_ms": 3.5,
    "wing_area_m2": 25.0,
    "aspect_ratio": 15.0,
    "oswald_efficiency": 0.80,
    "cd0": 0.025,
    "cruise_duration_s": 15.0 * 60.0,
    "loiter_duration_s": 30.0 * 60.0,
    "takeoff_enabled": True,
    "descent_enabled": True,
    "takeoff_duration_s": 3.0,
    "landing_duration_s": 52.0,
    "takeoff_ground_roll_m": 51.5,
}


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
    load_factor: float = 1.0


def _stall_limited_speed(h_m: float, W_N: float, S_ref: float) -> float:
    rho = isa_density(h_m)
    return AERO.stall_margin * math.sqrt(2.0 * W_N / (rho * S_ref * AERO.CL_max))


def build_mission_profile(
    cruise_duration_s: float = None,
    loiter_duration_s: float = None,
    cruise_alt_m: float = None,
    loiter_alt_m: float = 3000.0,
    climb_rate_ms: float = None,
    descent_rate_ms: float = None,
    cruise_speed_ms: float = None,
    takeoff_enabled: bool = True,
    descent_enabled: bool = True,
    climb_duration_s: float = None,
    climb_distance_m: float = None,
    cruise_distance_m: float = None,
    headwind_ms: float = 0.0,
    mtow_kg: float = None,
    wing_area_m2: float = None,
    aspect_ratio: float = None,
    oswald_efficiency: float = None,
    cd0: float = None,
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
    V_cr        = cruise_speed_ms or AIRCRAFT.cruise_speed_ms
    V_cl        = 45.0   # m/s  — climb airspeed (slower than cruise)
    S           = wing_area_m2 if wing_area_m2 is not None else AERO.S_ref
    W_N         = (mtow_kg or AIRCRAFT.mtow_kg) * G
    V_des       = max(45.0, _stall_limited_speed(loiter_alt_m, W_N, S))   # m/s

    # Loiter at best-endurance speed
    V_lo = best_endurance_speed(
        loiter_alt_m,
        W_N,
        S_ref=wing_area_m2,
        AR=aspect_ratio,
        e0=oswald_efficiency,
        CD0=cd0,
    )
    V_lo = max(V_lo, _stall_limited_speed(loiter_alt_m, W_N, S))
    V_land = max(V_lo * 0.58, _stall_limited_speed(0.0, W_N, S) * 1.05)

    ground_cruise_ms = max(15.0, V_cr - headwind_ms)
    ground_climb_ms = max(12.0, V_cl - headwind_ms)
    if cruise_distance_m and cruise_distance_m > 0:
        cruise_dur = cruise_distance_m / ground_cruise_ms

    if climb_duration_s and climb_duration_s > 0:
        t_cl = climb_duration_s
        h_cr = h_to + roc * t_cl
    elif climb_distance_m and climb_distance_m > 0:
        t_cl = climb_distance_m / ground_climb_ms
        h_cr = h_to + roc * t_cl
    else:
        t_cl = (h_cr - h_to) / roc          # Climb duration [s]
    h_cr = max(h_to, h_cr)

    t_des = (loiter_alt_m - 0.0) / rod   # Descent from loiter alt [s]

    prop = AERO.prop_efficiency

    segments = []

    if takeoff_enabled:
        segments.append(MissionSegment(
            name="TAKEOFF",
            duration_s=30.0,
            h_start_m=0.0, h_end_m=h_to,
            V_ms=V_cl * 0.70,   # Slower during roll/rotation
            roc_ms=h_to / 30.0,  # Gain 50 m in 30 s
            eta_prop=0.60,
            engine_on=True,
        ))
        climb_start_alt = h_to
    else:
        climb_start_alt = 0.0

    if t_cl > 1.0 and h_cr > climb_start_alt:
        segments.append(MissionSegment(
            name="CLIMB",
            duration_s=t_cl,
            h_start_m=climb_start_alt, h_end_m=h_cr,
            V_ms=V_cl,
            roc_ms=roc,
            eta_prop=prop["climb"],
            engine_on=True,
        ))

    segments.extend([
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
    ])

    if descent_enabled:
        segments.append(MissionSegment(
            name="DESCENT",
            duration_s=t_des,
            h_start_m=loiter_alt_m, h_end_m=0.0,
            V_ms=V_des,
            roc_ms=-rod,
            eta_prop=0.55,     # Windmilling efficiency
            engine_on=False,
            regen=True,
        ))

    segments.append(MissionSegment(
            name="LANDING",
            duration_s=30.0,
            h_start_m=0.0, h_end_m=0.0,
            V_ms=V_land,
            roc_ms=0.0,
            eta_prop=prop["loiter"],
            engine_on=False,
        ))

    return segments


def build_mission1_profile(**overrides) -> List[MissionSegment]:
    """Build the named Mission 1 profile from the UAV parameter brief."""
    params = dict(MISSION1_PRESET)
    params.update({k: v for k, v in overrides.items() if v is not None})
    W_N = params["mtow_kg"] * G
    eta_prop = 0.82
    takeoff_alt_m = 0.0
    climb_time_s = params["cruise_alt_m"] / params["climb_rate_ms"]
    descent_time_s = params["loiter_alt_m"] / params["descent_rate_ms"]
    loiter_load_factor = 1.0 / math.cos(math.radians(params["loiter_bank_deg"]))
    S = params["wing_area_m2"]

    takeoff_speed = max(
        1.2 * math.sqrt(2.0 * W_N / (isa_density(0.0) * S * AERO.CL_max)),
        24.8,
    )
    landing_speed = max(
        1.3 * math.sqrt(2.0 * W_N / (isa_density(0.0) * S * AERO.CL_max)),
        26.8,
    )

    return [
        MissionSegment(
            name="TAKEOFF",
            duration_s=params["takeoff_duration_s"],
            h_start_m=0.0, h_end_m=0.0,
            V_ms=takeoff_speed,
            roc_ms=0.0,
            eta_prop=0.60,
            engine_on=True,
        ),
        MissionSegment(
            name="CLIMB",
            duration_s=climb_time_s,
            h_start_m=takeoff_alt_m, h_end_m=params["cruise_alt_m"],
            V_ms=params["climb_speed_ms"],
            roc_ms=params["climb_rate_ms"],
            eta_prop=eta_prop,
            engine_on=True,
        ),
        MissionSegment(
            name="CRUISE",
            duration_s=params["cruise_duration_s"],
            h_start_m=params["cruise_alt_m"], h_end_m=params["cruise_alt_m"],
            V_ms=params["cruise_speed_ms"],
            roc_ms=0.0,
            eta_prop=eta_prop,
            engine_on=True,
        ),
        MissionSegment(
            name="LOITER",
            duration_s=params["loiter_duration_s"],
            h_start_m=params["loiter_alt_m"], h_end_m=params["loiter_alt_m"],
            V_ms=params["loiter_speed_ms"],
            roc_ms=0.0,
            eta_prop=eta_prop,
            engine_on=True,
            load_factor=loiter_load_factor,
        ),
        MissionSegment(
            name="DESCENT",
            duration_s=descent_time_s,
            h_start_m=params["loiter_alt_m"], h_end_m=0.0,
            V_ms=params["descent_speed_ms"],
            roc_ms=-params["descent_rate_ms"],
            eta_prop=eta_prop,
            engine_on=True,
            regen=True,
        ),
        MissionSegment(
            name="LANDING",
            duration_s=params["landing_duration_s"],
            h_start_m=0.0, h_end_m=0.0,
            V_ms=landing_speed,
            roc_ms=0.0,
            eta_prop=eta_prop,
            engine_on=False,
        ),
    ]


def total_mission_time(segments: List[MissionSegment]) -> float:
    """Total mission duration [s]."""
    return sum(s.duration_s for s in segments)
