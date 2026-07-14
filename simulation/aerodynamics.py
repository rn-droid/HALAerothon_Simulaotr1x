"""
Aerodynamics Model — Drag, lift, and required thrust for steady flight.

Computes propulsive power demand as a function of flight state (V, h, ROC).
Cross-references: AERO-01 through AERO-11 in assumption_register.md
"""

import math
from simulation.atmosphere import isa_density
from config import AERO, AIRCRAFT, G


def compute_drag(V_ms: float, h_m: float, W_N: float) -> dict:
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
    S = AERO.S_ref
    
    CL = W_N / (q * S) if q * S > 0 else 0.0
    CD_i = AERO.k * CL ** 2
    CD = AERO.CD0 + CD_i
    D_N = q * S * CD
    L_D = CL / CD if CD > 1e-12 else 0.0
    
    return {
        "rho": rho,
        "q": q,
        "CL": CL,
        "CD": CD,
        "CD_i": CD_i,
        "D_N": D_N,
        "L_D_ratio": L_D,
    }


def required_thrust(V_ms: float, h_m: float, W_N: float,
                     roc_ms: float = 0.0) -> float:
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
    aero = compute_drag(V_ms, h_m, W_N)
    climb_term = W_N * (roc_ms / V_ms) if V_ms > 1.0 else 0.0
    return aero["D_N"] + climb_term


def required_shaft_power(V_ms: float, h_m: float, W_N: float,
                         roc_ms: float = 0.0,
                         eta_prop: float = 0.82) -> float:
    """
    Required shaft power = T_req * V / eta_propeller.
    
    Returns:
        Shaft power [W]
    """
    T_req = required_thrust(V_ms, h_m, W_N, roc_ms)
    P_thrust = T_req * V_ms
    return P_thrust / eta_prop if eta_prop > 0.01 else P_thrust


def best_endurance_speed(h_m: float, W_N: float) -> float:
    """
    Speed for minimum power (best endurance) at altitude h.
    V_be = sqrt( (2W)/(rho*S) * sqrt(k/(3*CD0)) )
    """
    rho = isa_density(h_m)
    S = AERO.S_ref
    inner = math.sqrt(AERO.k / (3.0 * AERO.CD0))
    return math.sqrt((2.0 * W_N / (rho * S)) * inner)
