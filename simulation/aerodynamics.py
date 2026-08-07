"""
Aerodynamics Model — Drag, lift, and required thrust for steady flight.

Computes propulsive power demand as a function of flight state (V, h, ROC).
Cross-references: AERO-01 through AERO-11 in assumption_register.md
"""

import math
from simulation.atmosphere import isa_density
from config import AERO, AIRCRAFT, G


def compute_drag(
    V_ms: float,
    h_m: float,
    W_N: float,
    roc_ms: float = 0.0,
    load_factor: float = 1.0,
    S_ref: float = None,
    AR: float = None,
    e0: float = None,
    CD0: float = None,
) -> dict:
    """
    Compute aerodynamic forces at a given flight state.
    
    Args:
        V_ms:  True airspeed [m/s]
        h_m:   Altitude [m]
        W_N:   Aircraft weight [N]
    
    Returns:
        dict with keys: rho, q, CL, CD, CD_i, D_N, L_D_ratio
    """
    rho = isa_density(h_m)
    q = 0.5 * rho * V_ms ** 2
    S = S_ref if S_ref is not None else AERO.S_ref
    ar = AR if AR is not None else AERO.AR
    oswald = e0 if e0 is not None else AERO.e0
    cd0 = CD0 if CD0 is not None else AERO.CD0
    k = 1.0 / (math.pi * ar * oswald)
    
    sin_gamma = max(-0.95, min(0.95, roc_ms / V_ms)) if V_ms > 1.0 else 0.0
    gamma_rad = math.asin(sin_gamma)
    lift_required_N = load_factor * W_N * math.cos(gamma_rad)

    CL = lift_required_N / (q * S) if q * S > 0 else 0.0
    CD_i = k * CL ** 2
    CD = cd0 + CD_i
    D_N = q * S * CD
    L_D = CL / CD if CD > 1e-12 else 0.0
    stall_speed_ms = math.sqrt(2.0 * W_N / (rho * S * AERO.CL_max)) if rho * S * AERO.CL_max > 0 else 0.0
    min_safe_speed_ms = AERO.stall_margin * stall_speed_ms
    
    return {
        "rho": rho,
        "q": q,
        "gamma_rad": gamma_rad,
        "gamma_deg": math.degrees(gamma_rad),
        "lift_required_N": lift_required_N,
        "CL": CL,
        "CD": CD,
        "CD_i": CD_i,
        "D_N": D_N,
        "L_D_ratio": L_D,
        "stall_speed_ms": stall_speed_ms,
        "min_safe_speed_ms": min_safe_speed_ms,
        "stall_margin": V_ms / stall_speed_ms if stall_speed_ms > 1e-9 else 0.0,
    }


def required_thrust(
    V_ms: float,
    h_m: float,
    W_N: float,
    roc_ms: float = 0.0,
    load_factor: float = 1.0,
    S_ref: float = None,
    AR: float = None,
    e0: float = None,
    CD0: float = None,
) -> float:
    """
    Required thrust for steady flight with a rate of climb.
    
    T_req = D + W * (ROC / V)
    
    Args:
        V_ms:    True airspeed [m/s]
        h_m:     Altitude [m]
        W_N:     Aircraft weight [N]
        roc_ms:  Rate of climb [m/s] (negative for descent)
    
    Returns:
        Thrust required [N]
    """
    aero = compute_drag(V_ms, h_m, W_N, roc_ms=roc_ms, load_factor=load_factor, S_ref=S_ref, AR=AR, e0=e0, CD0=CD0)
    climb_term = W_N * math.sin(aero["gamma_rad"])
    return aero["D_N"] + climb_term


def required_shaft_power(V_ms: float, h_m: float, W_N: float,
                         roc_ms: float = 0.0,
                         eta_prop: float = 0.82,
                         load_factor: float = 1.0,
                         S_ref: float = None,
                         AR: float = None,
                         e0: float = None,
                         CD0: float = None) -> float:
    """
    Required shaft power = T_req * V / eta_propeller.
    
    Returns:
        Shaft power [W]
    """
    T_req = required_thrust(
        V_ms, h_m, W_N, roc_ms,
        load_factor=load_factor,
        S_ref=S_ref, AR=AR, e0=e0, CD0=CD0,
    )
    P_thrust = T_req * V_ms
    return P_thrust / eta_prop if eta_prop > 0.01 else P_thrust


def flight_state_forces(V_ms: float, h_m: float, W_N: float,
                        roc_ms: float = 0.0,
                        eta_prop: float = 0.82,
                        load_factor: float = 1.0,
                        S_ref: float = None,
                        AR: float = None,
                        e0: float = None,
                        CD0: float = None) -> dict:
    """Return aerodynamic force balance and shaft power for one timestep."""
    aero = compute_drag(
        V_ms, h_m, W_N, roc_ms=roc_ms,
        load_factor=load_factor,
        S_ref=S_ref, AR=AR, e0=e0, CD0=CD0,
    )
    climb_force_N = W_N * math.sin(aero["gamma_rad"])
    thrust_required_N = aero["D_N"] + climb_force_N
    propulsive_power_W = thrust_required_N * V_ms
    shaft_power_W = propulsive_power_W / eta_prop if eta_prop > 0.01 else propulsive_power_W
    return {
        **aero,
        "climb_force_N": climb_force_N,
        "thrust_required_N": thrust_required_N,
        "propulsive_power_W": propulsive_power_W,
        "shaft_power_W": shaft_power_W,
        "load_factor": load_factor,
    }


def best_endurance_speed(
    h_m: float,
    W_N: float,
    S_ref: float = None,
    AR: float = None,
    e0: float = None,
    CD0: float = None,
) -> float:
    """
    Speed for minimum power (best endurance) at altitude h.
    V_be = sqrt( (2W)/(rho*S) * sqrt(k/(3*CD0)) )
    """
    rho = isa_density(h_m)
    S = S_ref if S_ref is not None else AERO.S_ref
    ar = AR if AR is not None else AERO.AR
    oswald = e0 if e0 is not None else AERO.e0
    cd0 = CD0 if CD0 is not None else AERO.CD0
    k = 1.0 / (math.pi * ar * oswald)
    inner = math.sqrt(k / (3.0 * cd0))
    return math.sqrt((2.0 * W_N / (rho * S)) * inner)
