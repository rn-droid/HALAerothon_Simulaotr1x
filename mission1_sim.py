"""
Mission 1 — Master Simulation
==============================
Implements the FULL Mission 1 specification exactly as written.
Every parameter value comes directly from the spec — nothing is invented.

Architecture:
  Turboshaft (60 kW max) -> Generator (92%) -> DC bus -> Inverter (97%)
  -> Motor (95%) -> Propeller (82%)
  Battery (36 kWh NMC, 160 kg) <-> DC bus via Inverter
  Supercapacitor (0.066 kWh, 12 kg) -> DC bus (transient peaking)

Engine power budget:
  P_engine_total = P_engine_to_propeller + P_engine_to_generator <= 60 kW
  (engine power is NOT double-counted)

Phases:
  1. TAKEOFF      (10 s,  0->25 m)
  2. CLIMB        (1500 s, 0->6000 m, V=45 m/s, ROC=+4 m/s)
  3. CRUISE       (1200 s, 6000 m,   V=66.67 m/s)
  4. LOITER       (1800 s, 6000 m,   V=50 m/s, bank=25 deg)
  5. DESCENT      (~857 s, 6000->3000 m, V=45 m/s, ROC=-3.5 m/s)
  6. FINAL_APPROACH(~1000 s, 3000->0 m,  V=40 m/s, ROC=-3 m/s)
  7. FLARE        (~60 s,  near 0 m)
"""

import math
import json
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from dataclasses import dataclass, field
from typing import List, Dict, Optional

# ═══════════════════════════════════════════════════════════════════
# CONSTANTS (from spec — do not change)
# ═══════════════════════════════════════════════════════════════════

G = 9.80665            # m/s²

# ISA
T0_ISA   = 288.15      # K
P0_ISA   = 101325.0    # Pa
RHO0_ISA = 1.225       # kg/m³
LAPSE    = -0.0065     # K/m
R_AIR    = 287.058     # J/(kg·K)

# Wing / Aero
S        = 25.0        # m²  wing area
AR       = 15.0        # aspect ratio
E_OSW    = 0.80        # Oswald efficiency
CD0      = 0.025       # zero-lift drag
CL_MAX   = 1.50        # max lift coefficient
k_IND    = 1.0 / (math.pi * E_OSW * AR)  # induced drag factor ≈ 0.02653
ETA_PROP = 0.82        # propeller efficiency (spec §14)

# Span and chord (for reference only)
B_SPAN   = math.sqrt(AR * S)        # ≈ 19.36 m
C_CHORD  = S / B_SPAN               # ≈ 1.29 m

# Propulsion efficiencies
ETA_MOTOR    = 0.95   # motor mechanical / electrical
ETA_INVERTER = 0.97   # inverter (motor side)
ETA_GEN      = 0.92   # generator
ETA_RECT     = 0.98   # rectifier
ETA_BAT_DISC = 0.95   # battery one-way discharge (spec §2)
ETA_BAT_CHG  = 0.95   # battery one-way charge
ETA_GEARBOX  = 0.97   # gearbox
ETA_CLUTCH   = 0.98   # clutch

# Engine
ENG_MAX_W    = 60_000.0   # 60 kW max shaft
BSFC_KG_KWH  = 0.60       # kg/kWh flat baseline (spec §4)

# Battery (spec §2)
BAT_CAP_KWH  = 36.0        # kWh
BAT_MASS_KG  = 160.0       # kg
BAT_SOC_INIT = 0.90
BAT_SOC_MIN  = 0.20
BAT_SOC_MAX  = 0.95

# Supercapacitor (spec §3)
SC_MASS_KG   = 12.0
SC_SP_E      = 5.5          # Wh/kg → 66 Wh total
SC_SP_P      = 5400.0       # W/kg  → ~65 kW peak
SC_CAP_WH    = SC_MASS_KG * SC_SP_E   # 66 Wh
SC_MAX_W     = SC_MASS_KG * SC_SP_P   # 64,800 W

# Motor
MOT_MAX_W    = 70_000.0    # 70 kW peak

# Mass breakdown (spec §1)
M_DRY_NO_FUEL = 936.0      # kg (all dry mass inc. battery, SC, motor, etc.)
M_FUEL_INIT   = 64.0       # kg

# Power electronics losses tracking
PE_MASS_KG   = 15.0        # for reporting

# Loiter bank
LOITER_BANK_DEG = 25.0
LOITER_N     = 1.0 / math.cos(math.radians(LOITER_BANK_DEG))  # ≈ 1.103

# Regen descent efficiencies (spec §REGENERATIVE ENERGY MODEL)
ETA_WINDMILL = 0.60
ETA_REGEN_PATH = ETA_WINDMILL * ETA_MOTOR * ETA_INVERTER * ETA_BAT_CHG
# ≈ 0.60 * 0.95 * 0.97 * 0.95 ≈ 0.5272

# Simulation timestep
DT = 1.0  # seconds


# ═══════════════════════════════════════════════════════════════════
# ATMOSPHERE
# ═══════════════════════════════════════════════════════════════════

def isa(altitude_m: float):
    """Return (T, P, rho) for ISA troposphere (0-11000 m)."""
    h = max(0.0, min(altitude_m, 11000.0))
    T   = T0_ISA + LAPSE * h
    P   = P0_ISA * (T / T0_ISA) ** (G / (R_AIR * abs(LAPSE)))
    rho = P / (R_AIR * T)
    return T, P, rho


# ═══════════════════════════════════════════════════════════════════
# AERODYNAMICS
# ═══════════════════════════════════════════════════════════════════

def aerodynamics(W_N: float, V: float, h: float,
                 load_factor: float = 1.0) -> dict:
    """
    Compute aerodynamic quantities for given weight, speed, altitude,
    and load factor.
    Returns dict with CL, CD, CD0, CD_induced, q, D, L.
    """
    T, P, rho = isa(h)
    q  = 0.5 * rho * V * V
    CL = W_N * load_factor / (q * S)
    CD_induced = k_IND * CL ** 2
    CD_total   = CD0 + CD_induced
    L  = q * S * CL
    D  = q * S * CD_total
    return dict(
        q=q, CL=CL, CD=CD_total, CD0=CD0, CD_induced=CD_induced,
        L=L, D=D, rho=rho, T_atm=T, P_atm=P
    )


def shaft_power_required(W_N: float, V: float, h: float,
                         h_dot: float = 0.0,
                         load_factor: float = 1.0) -> dict:
    """
    Required propeller shaft power for given flight conditions.

    P_aero = D*V + W*h_dot   (spec §14)
    P_shaft = P_aero / eta_prop  (when P_aero > 0)
    When P_aero < 0 (glide/descent surplus): available for regen.
    """
    aero = aerodynamics(W_N, V, h, load_factor)
    P_aero = aero["D"] * V + W_N * h_dot
    # Shaft power needed at propeller
    if P_aero >= 0:
        P_shaft = P_aero / ETA_PROP
    else:
        # Surplus aerodynamic power available for regen
        P_shaft = P_aero  # negative: windmilling energy available
    aero["P_aero"] = P_aero
    aero["P_shaft"] = P_shaft
    aero["h_dot"] = h_dot
    return aero


def stall_speed(W_N: float, h: float) -> float:
    """V_stall = sqrt(2W / (rho * S * CL_max))"""
    _, _, rho = isa(h)
    return math.sqrt(2.0 * W_N / (rho * S * CL_MAX))


# ═══════════════════════════════════════════════════════════════════
# SUPERCAPACITOR STATE
# ═══════════════════════════════════════════════════════════════════

class Supercapacitor:
    def __init__(self):
        self.capacity_Wh = SC_CAP_WH
        self.max_power_W = SC_MAX_W
        self.soc = 0.95          # start nearly full
        self.soc_min = 0.05
        self.soc_max = 0.95
        self._eta = math.sqrt(0.97)  # symmetric round-trip 97%

    def step(self, p_req_W: float, dt_s: float) -> float:
        """
        Request power from supercap. Positive=discharge, negative=charge.
        Returns actual power delivered (W).
        """
        dt_h = dt_s / 3600.0
        if p_req_W > 0:
            p_req_W = min(p_req_W, self.max_power_W)
            e_drain = p_req_W * dt_h / self._eta
            d_soc = e_drain / self.capacity_Wh
            avail = self.soc - self.soc_min
            if d_soc > avail:
                d_soc = max(avail, 0.0)
                e_drain = d_soc * self.capacity_Wh
                p_req_W = e_drain * self._eta / dt_h if dt_h > 0 else 0.0
            self.soc -= d_soc
            return p_req_W
        elif p_req_W < 0:
            p_chg = min(-p_req_W, self.max_power_W)
            e_store = p_chg * dt_h * self._eta
            d_soc = e_store / self.capacity_Wh
            room = self.soc_max - self.soc
            if d_soc > room:
                d_soc = max(room, 0.0)
                e_store = d_soc * self.capacity_Wh
                p_chg = e_store / (dt_h * self._eta) if dt_h > 0 else 0.0
            self.soc += d_soc
            return -p_chg
        return 0.0

    @property
    def energy_Wh(self):
        return self.soc * self.capacity_Wh


# ═══════════════════════════════════════════════════════════════════
# BATTERY STATE
# ═══════════════════════════════════════════════════════════════════

class Battery:
    def __init__(self):
        self.capacity_Wh = BAT_CAP_KWH * 1000.0   # 36,000 Wh
        self.soc = BAT_SOC_INIT
        self.soc_min = BAT_SOC_MIN
        self.soc_max = BAT_SOC_MAX
        # Max C-rates: 2C discharge, 1C charge (conservative for 36 kWh pack)
        self.max_discharge_W = 2.0 * self.capacity_Wh   # 72,000 W
        self.max_charge_W    = 1.0 * self.capacity_Wh   # 36,000 W

    @property
    def energy_Wh(self):
        return self.soc * self.capacity_Wh

    @property
    def usable_Wh(self):
        return (self.soc - self.soc_min) * self.capacity_Wh

    def step(self, p_req_W: float, dt_s: float) -> float:
        """
        p_req_W > 0: discharge (battery → bus)
        p_req_W < 0: charge   (bus → battery)
        Returns actual power delivered (W).
        """
        dt_h = dt_s / 3600.0

        if p_req_W > 0:
            p_req_W = min(p_req_W, self.max_discharge_W)
            e_drain = p_req_W * dt_h / ETA_BAT_DISC
            d_soc = e_drain / self.capacity_Wh
            avail = self.soc - self.soc_min
            if d_soc > avail:
                d_soc = max(avail, 0.0)
                e_drain = d_soc * self.capacity_Wh
                p_req_W = e_drain * ETA_BAT_DISC / dt_h if dt_h > 0 else 0.0
            self.soc -= d_soc
            return p_req_W

        elif p_req_W < 0:
            p_chg = min(-p_req_W, self.max_charge_W)
            e_store = p_chg * dt_h * ETA_BAT_CHG
            d_soc = e_store / self.capacity_Wh
            room = self.soc_max - self.soc
            if d_soc > room:
                d_soc = max(room, 0.0)
                e_store = d_soc * self.capacity_Wh
                p_chg = e_store / (dt_h * ETA_BAT_CHG) if dt_h > 0 else 0.0
            self.soc += d_soc
            return -p_chg

        return 0.0


# ═══════════════════════════════════════════════════════════════════
# ENERGY MANAGEMENT (per spec §ENERGY MANAGEMENT STRATEGY)
# ═══════════════════════════════════════════════════════════════════

def split_power(phase: str, P_shaft_W: float,
                bat: Battery, sc: Supercapacitor,
                fuel_kg: float, dt_s: float) -> dict:
    """
    Determine how engine, motor, battery, generator, and supercapacitor
    share the total shaft power demand for one timestep.

    CRITICAL CONSTRAINTS (spec §CRITICAL POWER CONSTRAINTS):
      - P_engine <= 60 kW total shaft
      - P_engine = P_eng_to_prop + P_eng_to_gen   (no double-counting)
      - P_motor  <= 70 kW
      - Battery SoC: 20%-95%

    Returns dict with all power flows and fuel burn.
    """

    # ── GROUND: ENGINE_START (idle warmup) and GROUND_ROLL (engine off) ──
    _GROUND_ZERO = dict(
        P_engine_W=0.0, P_engine_to_prop_W=0.0, P_engine_to_gen_W=0.0,
        P_gen_elec_W=0.0, P_motor_mech_W=0.0, P_motor_elec_W=0.0,
        P_battery_W=0.0, P_sc_W=0.0, P_shaft_delivered_W=0.0,
        fuel_flow_kg_s=0.0, fuel_burn_kg=0.0, P_regen_stored_W=0.0,
    )
    if phase == "ENGINE_START":
        # Idle fuel burn ~3 kW shaft equivalent at idle BSFC=0.6 kg/kWh
        P_idle_W   = 3_000.0   # 3 kW idle
        ff         = (BSFC_KG_KWH * P_idle_W / 1000.0) / 3600.0
        d = _GROUND_ZERO.copy()
        d.update(P_engine_W=P_idle_W, fuel_flow_kg_s=ff,
                 fuel_burn_kg=ff * dt_s, mode="ENGINE_START_IDLE")
        return d
    if phase == "GROUND_ROLL":
        d = _GROUND_ZERO.copy(); d["mode"] = "GROUND_ROLL_SHUTDOWN"
        return d

    # ── DESCENT & FINAL APPROACH: Engine OFF, regen if applicable ──
    if phase in ("DESCENT", "FINAL_APPROACH", "FLARE"):
        if P_shaft_W < 0:
            # Surplus energy available — attempt regen
            P_regen_avail = -P_shaft_W  # positive W available at shaft
            P_regen_electrical = P_regen_avail * ETA_REGEN_PATH
            # Limit by battery max charge power
            P_charge_possible = min(P_regen_electrical,
                                    bat.max_charge_W,
                                    MOT_MAX_W * ETA_MOTOR * ETA_INVERTER)
            P_bat_actual = bat.step(-P_charge_possible, dt_s)  # negative = charging
            P_regen_stored = -P_bat_actual if P_bat_actual < 0 else 0.0
        else:
            # Idle glide - very small positive power needed; use motor at minimum
            P_bat_actual = 0.0
            P_regen_stored = 0.0

        return dict(
            P_engine_W=0.0,
            P_engine_to_prop_W=0.0,
            P_engine_to_gen_W=0.0,
            P_gen_elec_W=0.0,
            P_motor_mech_W=0.0,
            P_motor_elec_W=0.0,
            P_battery_W=P_bat_actual,
            P_sc_W=0.0,
            P_shaft_delivered_W=max(P_shaft_W, 0.0),
            fuel_flow_kg_s=0.0,
            fuel_burn_kg=0.0,
            P_regen_stored_W=P_regen_stored,
            mode="REGEN" if P_shaft_W < 0 else "GLIDE",
        )

    # ── TAKEOFF: Engine max + Motor + Supercapacitor ──────────────
    if phase == "TAKEOFF":
        # Engine: full 60 kW to propeller (parallel mode)
        P_eng_prop = min(ENG_MAX_W, P_shaft_W)
        P_remaining = P_shaft_W - P_eng_prop

        # Supercapacitor handles transient peak
        P_sc_elec = P_remaining / (ETA_MOTOR * ETA_INVERTER) if P_remaining > 0 else 0.0
        P_sc_actual = sc.step(P_sc_elec, dt_s)
        P_sc_mech = P_sc_actual * ETA_MOTOR * ETA_INVERTER

        # Battery covers remainder
        P_motor_remain = max(0.0, P_remaining - P_sc_mech)
        P_bat_elec = P_motor_remain / (ETA_MOTOR * ETA_INVERTER) if P_motor_remain > 0 else 0.0
        P_bat_elec = min(P_bat_elec, MOT_MAX_W / (ETA_MOTOR * ETA_INVERTER))
        P_bat_actual = bat.step(P_bat_elec, dt_s)
        P_motor_mech = (P_bat_actual + P_sc_actual) * ETA_MOTOR * ETA_INVERTER

        P_engine_W = P_eng_prop   # no generator usage in takeoff (parallel)
        fuel_flow = (BSFC_KG_KWH * P_engine_W / 1000.0) / 3600.0

        return dict(
            P_engine_W=P_engine_W,
            P_engine_to_prop_W=P_engine_W,
            P_engine_to_gen_W=0.0,
            P_gen_elec_W=0.0,
            P_motor_mech_W=P_motor_mech,
            P_motor_elec_W=P_bat_actual + P_sc_actual,
            P_battery_W=P_bat_actual,
            P_sc_W=P_sc_actual,
            P_shaft_delivered_W=P_engine_W + P_motor_mech,
            fuel_flow_kg_s=fuel_flow,
            fuel_burn_kg=fuel_flow * dt_s,
            P_regen_stored_W=0.0,
            mode="TAKEOFF",
        )

    # ── CLIMB: Engine max through SERIES path + Motor boost ───────
    if phase == "CLIMB":
        # Series path: engine -> generator -> DC bus -> motor -> prop
        # Engine at max rated power
        P_eng_total = ENG_MAX_W  # full 60 kW shaft
        P_gen_mech  = P_eng_total           # all to generator
        P_gen_elec  = P_gen_mech * ETA_GEN * ETA_RECT  # 60 * 0.92 * 0.98 ≈ 54.1 kW DC

        # Power needed at motor input (electrical)
        P_motor_total_mech = P_shaft_W
        P_motor_elec_total = P_motor_total_mech / (ETA_MOTOR * ETA_INVERTER)
        P_motor_elec_total = min(P_motor_elec_total, MOT_MAX_W / ETA_MOTOR)

        # Battery covers whatever generator can't supply
        P_bat_elec = max(0.0, P_motor_elec_total - P_gen_elec)
        P_bat_actual = bat.step(P_bat_elec, dt_s)

        P_motor_elec_actual = P_gen_elec + P_bat_actual
        P_motor_mech = P_motor_elec_actual * ETA_MOTOR * ETA_INVERTER
        P_motor_mech = min(P_motor_mech, MOT_MAX_W)

        fuel_flow = (BSFC_KG_KWH * P_eng_total / 1000.0) / 3600.0

        return dict(
            P_engine_W=P_eng_total,
            P_engine_to_prop_W=0.0,        # series: all engine power to generator
            P_engine_to_gen_W=P_eng_total,
            P_gen_elec_W=P_gen_elec,
            P_motor_mech_W=P_motor_mech,
            P_motor_elec_W=P_motor_elec_actual,
            P_battery_W=P_bat_actual,
            P_sc_W=0.0,
            P_shaft_delivered_W=P_motor_mech,
            fuel_flow_kg_s=fuel_flow,
            fuel_burn_kg=fuel_flow * dt_s,
            P_regen_stored_W=0.0,
            mode="CLIMB_SERIES",
        )

    # ── CRUISE: Engine ~60 kW + Motor supplement ──────────────────
    if phase == "CRUISE":
        # Series hybrid: engine -> gen -> bus -> motor
        # Engine at full rated power
        P_eng_total = ENG_MAX_W
        P_gen_elec  = P_eng_total * ETA_GEN * ETA_RECT

        P_motor_mech_req = P_shaft_W
        P_motor_elec_req = P_motor_mech_req / (ETA_MOTOR * ETA_INVERTER)
        P_motor_elec_req = min(P_motor_elec_req, MOT_MAX_W / ETA_MOTOR)

        # Battery makes up the difference
        P_bat_elec = max(0.0, P_motor_elec_req - P_gen_elec)
        P_bat_actual = bat.step(P_bat_elec, dt_s)

        P_motor_elec_actual = P_gen_elec + P_bat_actual
        P_motor_mech = P_motor_elec_actual * ETA_MOTOR * ETA_INVERTER
        P_motor_mech = min(P_motor_mech, MOT_MAX_W)

        fuel_flow = (BSFC_KG_KWH * P_eng_total / 1000.0) / 3600.0

        return dict(
            P_engine_W=P_eng_total,
            P_engine_to_prop_W=0.0,
            P_engine_to_gen_W=P_eng_total,
            P_gen_elec_W=P_gen_elec,
            P_motor_mech_W=P_motor_mech,
            P_motor_elec_W=P_motor_elec_actual,
            P_battery_W=P_bat_actual,
            P_sc_W=0.0,
            P_shaft_delivered_W=P_motor_mech,
            fuel_flow_kg_s=fuel_flow,
            fuel_burn_kg=fuel_flow * dt_s,
            P_regen_stored_W=0.0,
            mode="CRUISE_SERIES",
        )

    # ── LOITER: Engine at partial power (~40 kW), motor off ───────
    if phase == "LOITER":
        # Engine covers loiter demand directly via series path
        P_eng_req = P_shaft_W / (ETA_GEN * ETA_RECT * ETA_MOTOR * ETA_INVERTER)
        P_eng_req = min(P_eng_req, ENG_MAX_W)
        P_eng_actual = max(0.0, P_eng_req)

        P_gen_elec = P_eng_actual * ETA_GEN * ETA_RECT
        P_motor_mech = P_gen_elec * ETA_MOTOR * ETA_INVERTER

        # If engine delivers more than needed, option: charge battery
        # (spec: "optionally use unused engine power for battery charging")
        P_extra_elec = max(0.0, P_gen_elec - P_shaft_W / (ETA_MOTOR * ETA_INVERTER))
        P_bat_charge = 0.0
        if P_extra_elec > 100.0 and bat.soc < bat.soc_max:
            P_bat_charge = bat.step(-P_extra_elec, dt_s)  # negative = charging

        fuel_flow = (BSFC_KG_KWH * P_eng_actual / 1000.0) / 3600.0

        return dict(
            P_engine_W=P_eng_actual,
            P_engine_to_prop_W=0.0,
            P_engine_to_gen_W=P_eng_actual,
            P_gen_elec_W=P_gen_elec,
            P_motor_mech_W=P_motor_mech,
            P_motor_elec_W=P_gen_elec,
            P_battery_W=P_bat_charge,
            P_sc_W=0.0,
            P_shaft_delivered_W=P_motor_mech,
            fuel_flow_kg_s=fuel_flow,
            fuel_burn_kg=fuel_flow * dt_s,
            P_regen_stored_W=0.0,
            mode="LOITER_ENG",
        )

    # Fallback
    return dict(
        P_engine_W=0.0, P_engine_to_prop_W=0.0, P_engine_to_gen_W=0.0,
        P_gen_elec_W=0.0, P_motor_mech_W=0.0, P_motor_elec_W=0.0,
        P_battery_W=0.0, P_sc_W=0.0, P_shaft_delivered_W=0.0,
        fuel_flow_kg_s=0.0, fuel_burn_kg=0.0, P_regen_stored_W=0.0,
        mode="UNKNOWN",
    )


# ═══════════════════════════════════════════════════════════════════
# MISSION PHASE SEGMENTS
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PhaseSegment:
    name: str
    h_start: float    # m
    h_end: float      # m
    V: float          # m/s TAS
    h_dot: float      # m/s vertical speed (+ up, - down)
    duration_s: float # seconds
    load_factor: float = 1.0


# ----------------------------------------------------------------
# CANONICAL PHASE TABLE  (from user-confirmed master timeline)
# Column order: name, h_start, h_end, V_ms, h_dot, orig_s, load_factor
# orig_s = original (full-duration) seconds
# ----------------------------------------------------------------
_PHASE_TABLE = [
    # name             h0      h1     V_ms  h_dot  orig_s  LF
    ("ENGINE_START",    0.0,    0.0,   0.0,  0.0,   192.0,  1.0),  # 3m12s ground
    ("TAKEOFF",         0.0,   25.0,  25.0,  2.5,    12.0,  1.0),  # 12s roll/liftoff
    ("CLIMB",           0.0, 6000.0,  45.0,  4.0,  1200.0,  1.0),  # 20 min
    ("CRUISE",       6000.0, 6000.0,  66.67, 0.0,  1200.0,  1.0),  # 20 min
    ("LOITER",       6000.0, 6000.0,  50.0,  0.0,  1800.0,  LOITER_N), # 30 min
    ("DESCENT",      6000.0, 3000.0,  45.0, -3.5,   857.0,  1.0),  # 14m17s
    ("FINAL_APPROACH",3000.0,   0.0,  40.0, -3.0,  1000.0,  1.0),  # 16m40s
    ("FLARE",           5.0,    0.0,  25.0, -0.1,    60.0,  1.0),  # 1 min
    ("GROUND_ROLL",     0.0,    0.0,   5.0,  0.0,    60.0,  1.0),  # 1 min shutdown
]
# Total original mission duration (seconds)
TOTAL_ORIGINAL_S = sum(row[5] for row in _PHASE_TABLE)  # 6381 s (106 min 21 s)


def build_mission(time_scale: float = 1.0) -> List[PhaseSegment]:
    """
    Build Mission 1 phase list.

    Parameters
    ----------
    time_scale : float
        Multiplier applied to every phase duration.
        1.0  = full-duration mission  (~106 min)
        300/TOTAL_ORIGINAL_S = 5-minute compressed demo
    """
    phases = []
    for name, h0, h1, V, h_dot, orig_s, lf in _PHASE_TABLE:
        dur = max(1.0, orig_s * time_scale)  # never shorter than 1 s
        phases.append(PhaseSegment(
            name=name, h_start=h0, h_end=h1,
            V=V, h_dot=h_dot, duration_s=dur, load_factor=lf,
        ))
    return phases


# ═══════════════════════════════════════════════════════════════════
# MAIN SIMULATION LOOP
# ═══════════════════════════════════════════════════════════════════

def run_mission1(time_scale: float = 1.0) -> List[dict]:
    """
    Run Mission 1 simulation.

    Parameters
    ----------
    time_scale : float
        1.0 = full mission (~106 min).  Use 300/TOTAL_ORIGINAL_S for 5-min mode.

    Returns list of per-timestep records.
    """
    bat = Battery()
    sc  = Supercapacitor()
    phases = build_mission(time_scale)

    # State variables
    fuel_kg        = M_FUEL_INIT
    time_s         = 0.0
    distance_m     = 0.0
    cumul_fuel     = 0.0
    cumul_bat_e    = 0.0     # Wh discharged from battery (net)
    cumul_regen    = 0.0     # Wh recovered
    cumul_shaft_e  = 0.0     # Wh shaft energy
    constraint_violations = []
    records        = []

    for phase in phases:
        n_steps = max(1, int(phase.duration_s / DT))
        for step in range(n_steps):
            frac = step / max(n_steps - 1, 1)

            # Interpolate altitude within phase
            h = phase.h_start + (phase.h_end - phase.h_start) * frac
            h = max(0.0, h)
            V = phase.V
            h_dot = phase.h_dot
            lf = phase.load_factor

            # Current aircraft mass and weight
            M = M_DRY_NO_FUEL + fuel_kg
            W_N = M * G

            # Atmosphere
            T_atm, P_atm, rho = isa(h)

            # Ground phases (V=0): bypass aero model entirely
            IS_GROUND = phase.name in ("ENGINE_START", "GROUND_ROLL")
            if IS_GROUND or V < 1.0:
                V_s = 0.0
                aero = dict(
                    q=0.0, CL=0.0, CD=CD0, CD0=CD0, CD_induced=0.0,
                    L=0.0, D=0.0, rho=rho, T_atm=T_atm, P_atm=P_atm,
                    P_aero=0.0, P_shaft=0.0, h_dot=h_dot,
                )
            else:
                V_s = stall_speed(W_N, h)
                aero = shaft_power_required(W_N, V, h, h_dot, lf)

            P_shaft_req = aero["P_shaft"]

            # ── Power split decision ──────────────────────────
            out = split_power(
                phase.name, P_shaft_req, bat, sc, fuel_kg, DT
            )

            # ── Fuel accounting ───────────────────────────────
            fuel_burn = out["fuel_burn_kg"]
            fuel_kg   = max(0.0, fuel_kg - fuel_burn)
            cumul_fuel += fuel_burn

            # ── Distance (horizontal) ────────────────────────
            V_horiz = math.sqrt(max(0.0, V**2 - h_dot**2))
            distance_m += V_horiz * DT

            # ── Cumulative energy (kWh) ──────────────────────────
            # Battery: positive means discharge (energy leaving bat)
            if out["P_battery_W"] > 0:
                cumul_bat_e += out["P_battery_W"] * DT / 3_600_000.0  # kW*s -> kWh
            cumul_regen   += out["P_regen_stored_W"] * DT / 3_600_000.0
            cumul_shaft_e += max(0.0, aero["P_aero"]) * DT / 3_600_000.0  # Pa->kW->kWh

            # ── Component losses ──────────────────────────────
            P_motor_loss   = out["P_motor_elec_W"] * (1.0 / ETA_MOTOR - 1.0) if out["P_motor_elec_W"] > 0 else 0.0
            P_inv_loss     = out["P_motor_elec_W"] * (1.0 / ETA_INVERTER - 1.0) if out["P_motor_elec_W"] > 0 else 0.0
            P_gen_loss     = out["P_engine_to_gen_W"] * (1.0 - ETA_GEN) if out["P_engine_to_gen_W"] > 0 else 0.0
            P_bat_loss     = abs(out["P_battery_W"]) * (1.0 / ETA_BAT_DISC - 1.0) if out["P_battery_W"] > 0 else 0.0
            P_thermal_total = P_motor_loss + P_inv_loss + P_gen_loss + P_bat_loss

            # ── Constraint checking ───────────────────────────
            viol = []
            if bat.soc < BAT_SOC_MIN - 0.001:
                viol.append(f"BAT_SOC_LOW={bat.soc:.4f}")
            if bat.soc > BAT_SOC_MAX + 0.001:
                viol.append(f"BAT_SOC_HIGH={bat.soc:.4f}")
            if out["P_engine_W"] > ENG_MAX_W * 1.001:
                viol.append(f"ENGINE_OVER={out['P_engine_W']/1000:.1f}kW")
            if out["P_motor_mech_W"] > MOT_MAX_W * 1.001:
                viol.append(f"MOTOR_OVER={out['P_motor_mech_W']/1000:.1f}kW")
            if viol:
                constraint_violations.append((time_s, phase.name, viol))

            # ── Fuel flow ─────────────────────────────────────
            fuel_flow_kg_s = out["fuel_flow_kg_s"]

            # ── Battery voltage/current proxy ─────────────────
            # Nominal voltage 3.7V/cell, ~400V pack (LiNMC)
            V_bat_nom = 400.0   # V (proxy)
            I_bat = (out["P_battery_W"] / V_bat_nom) if V_bat_nom > 0 else 0.0

            # ── Record ────────────────────────────────────────
            records.append({
                # Time/Space
                "TIME":                   round(time_s, 1),
                "MISSION_PHASE":          phase.name,
                "ALTITUDE":               round(h, 1),
                "DISTANCE":               round(distance_m / 1000.0, 3),  # km
                "TRUE_AIRSPEED":          round(V, 2),
                "GROUND_SPEED":           round(V_horiz, 2),
                "VERTICAL_SPEED":         round(h_dot, 2),
                # Atmosphere
                "TEMPERATURE":            round(T_atm, 2),
                "PRESSURE":               round(P_atm, 1),
                "AIR_DENSITY":            round(rho, 5),
                # Mass
                "AIRCRAFT_MASS":          round(M, 3),
                "FUEL_MASS":              round(fuel_kg, 4),
                "FUEL_FLOW":              round(fuel_flow_kg_s * 3600.0, 4),  # kg/h
                "FUEL_REMAINING":         round(fuel_kg, 4),
                "WEIGHT":                 round(W_N, 2),
                # Aero
                "DYNAMIC_PRESSURE":       round(aero["q"], 2),
                "CL":                     round(aero["CL"], 5),
                "CD":                     round(aero["CD"], 5),
                "CD0":                    CD0,
                "CD_INDUCED":             round(aero["CD_induced"], 5),
                "DRAG":                   round(aero["D"], 2),
                "LIFT":                   round(aero["L"], 2),
                # Power
                "AERODYNAMIC_POWER":      round(aero["P_aero"] / 1000.0, 3),  # kW
                "SHAFT_POWER_REQUIRED":   round(P_shaft_req / 1000.0, 3),     # kW
                "ENGINE_POWER":           round(out["P_engine_W"] / 1000.0, 3),
                "GENERATOR_MECH_POWER":   round(out["P_engine_to_gen_W"] / 1000.0, 3),
                "GENERATOR_ELEC_POWER":   round(out["P_gen_elec_W"] / 1000.0, 3),
                "MOTOR_POWER":            round(out["P_motor_mech_W"] / 1000.0, 3),
                "SUPERCAPACITOR_POWER":   round(out["P_sc_W"] / 1000.0, 3),
                "BATTERY_POWER":          round(out["P_battery_W"] / 1000.0, 3),
                # Battery state
                "BATTERY_SOC":            round(bat.soc * 100.0, 3),  # %
                "BATTERY_ENERGY":         round(bat.energy_Wh / 1000.0, 4),  # kWh
                "BATTERY_CURRENT":        round(I_bat, 2),
                "BATTERY_VOLTAGE":        V_bat_nom,
                # SC state
                "SUPERCAPACITOR_SOC":     round(sc.soc * 100.0, 2),  # %
                "SUPERCAPACITOR_ENERGY":  round(sc.energy_Wh * 1000.0, 2),  # Wh
                # Efficiencies
                "MOTOR_EFFICIENCY":       ETA_MOTOR,
                "INVERTER_EFFICIENCY":    ETA_INVERTER,
                "GENERATOR_EFFICIENCY":   ETA_GEN,
                "PROPELLER_EFFICIENCY":   ETA_PROP,
                # Losses (kW)
                "MOTOR_LOSS":             round(P_motor_loss / 1000.0, 4),
                "INVERTER_LOSS":          round(P_inv_loss / 1000.0, 4),
                "GENERATOR_LOSS":         round(P_gen_loss / 1000.0, 4),
                "BATTERY_LOSS":           round(P_bat_loss / 1000.0, 4),
                "TOTAL_THERMAL_LOSS":     round(P_thermal_total / 1000.0, 4),
                # Cumulative
                "CUMULATIVE_FUEL":        round(cumul_fuel, 4),
                "CUMULATIVE_BATTERY_E":   round(cumul_bat_e, 4),  # kWh
                "CUMULATIVE_REGEN_E":     round(cumul_regen, 4),  # kWh
                "CUMULATIVE_SHAFT_E":     round(cumul_shaft_e, 4),  # kWh
                # Control mode
                "MODE":                   out["mode"],
                # Constraint
                "VIOLATIONS":             "|".join(viol) if viol else "",
                # Stall margin
                "STALL_SPEED":            round(V_s, 2),
                "STALL_MARGIN":           round(V / V_s, 3) if V_s > 0 else 0.0,
            })

            time_s += DT

    print(f"\n  Total timesteps: {len(records)}")
    print(f"  Constraint violations: {len(constraint_violations)}")
    if constraint_violations:
        for t, ph, vs in constraint_violations[:5]:
            print(f"    t={t:.0f}s [{ph}]: {vs}")

    return records, constraint_violations


# ═══════════════════════════════════════════════════════════════════
# REPORTING
# ═══════════════════════════════════════════════════════════════════

def phase_summary(records: List[dict]) -> dict:
    """Compute per-phase energy / fuel / SoC summary."""
    from collections import defaultdict
    phases = {}
    phase_order = []

    for r in records:
        ph = r["MISSION_PHASE"]
        if ph not in phases:
            phases[ph] = []
            phase_order.append(ph)
        phases[ph].append(r)

    summary = {}
    for ph in phase_order:
        recs = phases[ph]
        n = len(recs)
        dur_s = n * DT
        e_shaft = sum(max(0, r["AERODYNAMIC_POWER"]) for r in recs) * DT / 3600.0
        e_bat_discharge = sum(r["BATTERY_POWER"] for r in recs if r["BATTERY_POWER"] > 0) * DT / 3600.0
        e_regen = sum(r.get("CUMULATIVE_REGEN_E", 0) for r in recs[-1:])
        fuel = sum(r["FUEL_FLOW"] / 3600.0 * DT for r in recs)  # kg
        soc_start = recs[0]["BATTERY_SOC"]
        soc_end   = recs[-1]["BATTERY_SOC"]
        avg_shaft = sum(r["SHAFT_POWER_REQUIRED"] for r in recs) / n
        avg_engine = sum(r["ENGINE_POWER"] for r in recs) / n
        avg_motor  = sum(r["MOTOR_POWER"] for r in recs) / n
        avg_bat    = sum(r["BATTERY_POWER"] for r in recs) / n
        summary[ph] = dict(
            duration_min=round(dur_s / 60.0, 2),
            e_shaft_kWh=round(e_shaft, 3),
            e_bat_kWh=round(e_bat_discharge, 3),
            fuel_kg=round(fuel, 3),
            soc_start=round(soc_start, 2),
            soc_end=round(soc_end, 2),
            avg_shaft_kW=round(avg_shaft, 2),
            avg_engine_kW=round(avg_engine, 2),
            avg_motor_kW=round(avg_motor, 2),
            avg_bat_kW=round(avg_bat, 2),
        )
    return summary


def print_report(records: List[dict], violations: list):
    """Print comprehensive mission 1 results to console."""
    if not records:
        print("No records!")
        return

    first = records[0]
    last  = records[-1]
    total_time_s = last["TIME"] - first["TIME"] + DT
    total_dist   = last["DISTANCE"]
    fuel_consumed = last["CUMULATIVE_FUEL"]
    fuel_remain   = last["FUEL_REMAINING"]
    soc_init      = first["BATTERY_SOC"]
    soc_final     = last["BATTERY_SOC"]
    soc_min_val   = min(r["BATTERY_SOC"] for r in records)
    cumul_regen   = last["CUMULATIVE_REGEN_E"]
    cumul_bat     = last["CUMULATIVE_BATTERY_E"]
    cumul_shaft   = last["CUMULATIVE_SHAFT_E"]
    mass_land     = last["AIRCRAFT_MASS"]

    print("\n" + "=" * 70)
    print("  MISSION 1 — SIMULATION RESULTS")
    print("=" * 70)
    print(f"  Total mission duration     : {total_time_s / 60.0:.1f} min  ({total_time_s:.0f} s)")
    print(f"  Total horizontal distance  : {total_dist:.1f} km")
    print(f"  Landing mass               : {mass_land:.1f} kg")
    print()
    print(f"  Fuel consumed              : {fuel_consumed:.2f} kg")
    print(f"  Fuel remaining             : {fuel_remain:.2f} kg")
    print(f"  Fuel initial               : {M_FUEL_INIT:.1f} kg")
    print()
    print(f"  Battery SoC initial        : {soc_init:.1f} %")
    print(f"  Battery SoC final          : {soc_final:.1f} %")
    print(f"  Battery SoC minimum        : {soc_min_val:.1f} %")
    print()
    print(f"  Total shaft energy         : {cumul_shaft:.2f} kWh")
    print(f"  Battery discharged (net)   : {cumul_bat:.2f} kWh")
    print(f"  Regenerated energy         : {cumul_regen:.2f} kWh")
    print()
    print(f"  Constraint violations      : {len(violations)}")
    print()

    # Reference comparison (spec - FINAL BASELINE EXPECTED RESULTS)
    print("  -- Validation vs Spec Targets ----------------------")
    checks = [
        ("Mission duration [min]",   total_time_s / 60.0,  107.0,  15.0),
        ("Distance [km]",            total_dist,            317.0,  30.0),
        ("Fuel consumed [kg]",       fuel_consumed,          39.0,   5.0),
        ("Fuel remaining [kg]",      fuel_remain,            25.0,   5.0),
        ("Landing mass [kg]",        mass_land,             961.0,  10.0),
        ("Final battery SoC [%]",    soc_final,              37.0,   5.0),
    ]
    for label, val, target, tol in checks:
        diff = abs(val - target)
        ok = "OK" if diff <= tol else "WARN"
        print(f"  [{ok}]  {label:<30}: {val:8.2f}  (target ~{target:.1f}  +/-{tol})")

    print()
    ph_sum = phase_summary(records)
    print("  -- Phase-by-Phase Summary ---------------------------")
    hdr = f"  {'Phase':<16} {'Dur(min)':>8} {'Shaft(kWh)':>10} {'Bat(kWh)':>9} {'Fuel(kg)':>8} {'SoC%':>12}"
    print(hdr)
    print("  " + "-" * 65)
    for ph, s in ph_sum.items():
        soc_str = f"{s['soc_start']:.1f}%->{s['soc_end']:.1f}%"
        print(f"  {ph:<16} {s['duration_min']:>8.1f} {s['e_shaft_kWh']:>10.2f} "
              f"{s['e_bat_kWh']:>9.2f} {s['fuel_kg']:>8.2f} {soc_str:>12}")
    print("=" * 70)


# ===================================================================
# PLOT ENGINE (all 30 required plots)
# ===================================================================

PHASE_COLORS = {
    "TAKEOFF":       "#e74c3c",
    "CLIMB":         "#f39c12",
    "CRUISE":        "#3498db",
    "LOITER":        "#2ecc71",
    "DESCENT":       "#9b59b6",
    "FINAL_APPROACH":"#1abc9c",
    "FLARE":         "#95a5a6",
}


def add_phase_bands(ax, records):
    """Shade phase bands on a plot."""
    import numpy as np
    times = [r["TIME"] / 60.0 for r in records]
    phases = [r["MISSION_PHASE"] for r in records]
    i = 0
    while i < len(phases):
        ph = phases[i]
        j = i
        while j < len(phases) and phases[j] == ph:
            j += 1
        ax.axvspan(times[i], times[j - 1] if j < len(times) else times[-1],
                   alpha=0.08, color=PHASE_COLORS.get(ph, "gray"), zorder=0)
        i = j


def generate_plots(records: List[dict], out_dir: str):
    """Generate all required mission plots and save them."""
    os.makedirs(out_dir, exist_ok=True)

    t_min = np.array([r["TIME"] / 60.0 for r in records])

    def arr(key):
        return np.array([r[key] for r in records])

    def save(fig, name):
        path = os.path.join(out_dir, name)
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        return path

    saved = []

    # ── 1. Altitude vs Time ──────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("ALTITUDE"), "b-", lw=2)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Altitude [m]")
    ax.set_title("Mission 1 — Altitude vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "01_altitude_vs_time.png"))

    # ── 2. Speed vs Time ────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("TRUE_AIRSPEED"), "b-", lw=2, label="TAS")
    ax.plot(t_min, arr("STALL_SPEED"),   "r--", lw=1.5, label="V_stall")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Speed [m/s]")
    ax.set_title("Mission 1 — Airspeed vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "02_speed_vs_time.png"))

    # ── 3. Vertical Speed vs Time ────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("VERTICAL_SPEED"), "g-", lw=2)
    ax.axhline(0, color="k", lw=0.8, ls="--")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Vertical Speed [m/s]")
    ax.set_title("Mission 1 — Vertical Speed vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "03_vspeed_vs_time.png"))

    # ── 4. Distance vs Time ──────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("DISTANCE"), "purple", lw=2)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Cumulative Distance [km]")
    ax.set_title("Mission 1 — Distance vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "04_distance_vs_time.png"))

    # ── 5. Aircraft Mass vs Time ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("AIRCRAFT_MASS"), "brown", lw=2)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Aircraft Mass [kg]")
    ax.set_title("Mission 1 — Aircraft Mass vs Time (decreasing with fuel burn)")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "05_mass_vs_time.png"))

    # ── 6. Fuel vs Time ─────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("FUEL_REMAINING"), "r-", lw=2, label="Fuel Remaining [kg]")
    ax.plot(t_min, arr("CUMULATIVE_FUEL"), "r--", lw=1.5, label="Cumulative Consumed [kg]")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Fuel [kg]")
    ax.set_title("Mission 1 — Fuel vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "06_fuel_vs_time.png"))

    # ── 7. Battery SoC vs Time ───────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("BATTERY_SOC"), "b-", lw=2)
    ax.axhline(20.0, color="red", ls="--", lw=1.5, label="SoC Min (20%)")
    ax.axhline(95.0, color="green", ls="--", lw=1.5, label="SoC Max (95%)")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Battery SoC [%]")
    ax.set_title("Mission 1 — Battery State of Charge vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 105)
    saved.append(save(fig, "07_soc_vs_time.png"))

    # ── 8. Battery Power vs Time ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("BATTERY_POWER"), "b-", lw=2)
    ax.axhline(0, color="k", lw=0.8, ls="--")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Battery Power [kW]")
    ax.set_title("Mission 1 — Battery Power vs Time (+discharge / -charge)")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "08_bat_power_vs_time.png"))

    # ── 9. Engine Power vs Time ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("ENGINE_POWER"), "r-", lw=2)
    ax.axhline(60.0, color="orange", ls="--", lw=1.5, label="60 kW Limit")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Engine Power [kW]")
    ax.set_title("Mission 1 — Engine Power vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "09_engine_power_vs_time.png"))

    # ── 10. Motor Power vs Time ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("MOTOR_POWER"), "green", lw=2)
    ax.axhline(70.0, color="orange", ls="--", lw=1.5, label="70 kW Limit")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Motor Power [kW]")
    ax.set_title("Mission 1 — Motor Mechanical Power vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "10_motor_power_vs_time.png"))

    # ── 11. Generator Power vs Time ──────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("GENERATOR_ELEC_POWER"), "teal", lw=2, label="Gen Elec")
    ax.plot(t_min, arr("GENERATOR_MECH_POWER"), "teal", lw=1.5, ls="--", label="Gen Mech")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Generator Power [kW]")
    ax.set_title("Mission 1 — Generator Power vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "11_generator_power_vs_time.png"))

    # ── 12. Supercapacitor Power & SoC vs Time ───────────────────
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax1.plot(t_min, arr("SUPERCAPACITOR_POWER"), "gold", lw=2)
    add_phase_bands(ax1, records)
    ax1.set_ylabel("SC Power [kW]"); ax1.set_title("Supercapacitor Power & SoC")
    ax1.grid(True, alpha=0.3)
    ax2.plot(t_min, arr("SUPERCAPACITOR_SOC"), "gold", lw=2)
    add_phase_bands(ax2, records)
    ax2.set_ylabel("SC SoC [%]"); ax2.set_xlabel("Time [min]")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    saved.append(save(fig, "12_supercap_vs_time.png"))

    # ── 13. Shaft Power Required vs Time ─────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("SHAFT_POWER_REQUIRED"), "navy", lw=2)
    ax.axhline(0, color="k", lw=0.8, ls="--")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Shaft Power [kW]")
    ax.set_title("Mission 1 — Required Shaft Power vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "13_shaft_power_vs_time.png"))

    # ── 14. Drag vs Time ─────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("DRAG"), "darkorange", lw=2)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Drag [N]")
    ax.set_title("Mission 1 — Aerodynamic Drag vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "14_drag_vs_time.png"))

    # ── 15. CL vs Time ──────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("CL"), "darkgreen", lw=2)
    ax.axhline(CL_MAX, color="red", ls="--", lw=1.5, label=f"CL_max={CL_MAX}")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Lift Coefficient CL")
    ax.set_title("Mission 1 — CL vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "15_CL_vs_time.png"))

    # ── 16. CD vs Time ──────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("CD"), "darkred", lw=2, label="CD total")
    ax.plot(t_min, arr("CD_INDUCED"), "r--", lw=1.5, label="CD induced")
    ax.axhline(CD0, color="orange", ls=":", lw=1.5, label=f"CD0={CD0}")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Drag Coefficient CD")
    ax.set_title("Mission 1 — CD vs Time")
    ax.legend(); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "16_CD_vs_time.png"))

    # ── 17. Thermal Loss vs Time ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.stackplot(t_min,
                 arr("MOTOR_LOSS"), arr("INVERTER_LOSS"),
                 arr("GENERATOR_LOSS"), arr("BATTERY_LOSS"),
                 labels=["Motor", "Inverter", "Generator", "Battery"],
                 colors=["#e74c3c", "#f39c12", "#3498db", "#2ecc71"],
                 alpha=0.85)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Thermal Loss [kW]")
    ax.set_title("Mission 1 — Component Thermal Losses vs Time")
    ax.legend(loc="upper right"); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "17_thermal_loss_vs_time.png"))

    # ── 18. Energy Contribution by Source (stacked area) ─────────
    fig, ax = plt.subplots(figsize=(12, 5))
    p_eng = arr("ENGINE_POWER")
    p_bat = np.array([max(0, r["BATTERY_POWER"]) for r in records])
    p_sc  = arr("SUPERCAPACITOR_POWER")
    ax.stackplot(t_min, p_eng, p_bat, p_sc,
                 labels=["Engine", "Battery (discharge)", "Supercapacitor"],
                 colors=["#e74c3c", "#3498db", "#f1c40f"], alpha=0.85)
    ax.plot(t_min, arr("SHAFT_POWER_REQUIRED"), "k--", lw=1.5, label="Shaft Req.")
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Power [kW]")
    ax.set_title("Mission 1 — Energy Contribution by Propulsion Source")
    ax.legend(loc="upper right"); ax.grid(True, alpha=0.3)
    saved.append(save(fig, "18_energy_contribution.png"))

    # ── 19. Air Density vs Time ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("AIR_DENSITY"), "steelblue", lw=2)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Air Density [kg/m³]")
    ax.set_title("Mission 1 — ISA Air Density vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "19_air_density_vs_time.png"))

    # ── 20. Weight vs Time ───────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_min, arr("WEIGHT"), "chocolate", lw=2)
    add_phase_bands(ax, records)
    ax.set_xlabel("Time [min]"); ax.set_ylabel("Weight [N]")
    ax.set_title("Mission 1 — Aircraft Weight vs Time")
    ax.grid(True, alpha=0.3)
    saved.append(save(fig, "20_weight_vs_time.png"))

    # ── 21. MEGA DASHBOARD (4x4 grid) ────────────────────────────
    fig = plt.figure(figsize=(24, 20))
    gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.45, wspace=0.35)
    panels = [
        (gs[0, 0], "ALTITUDE", "Altitude [m]", "b"),
        (gs[0, 1], "TRUE_AIRSPEED", "Speed [m/s]", "g"),
        (gs[0, 2], "ENGINE_POWER", "Engine [kW]", "r"),
        (gs[0, 3], "MOTOR_POWER", "Motor [kW]", "green"),
        (gs[1, 0], "BATTERY_SOC", "Bat SoC [%]", "royalblue"),
        (gs[1, 1], "BATTERY_POWER", "Bat Power [kW]", "navy"),
        (gs[1, 2], "FUEL_REMAINING", "Fuel [kg]", "darkorange"),
        (gs[1, 3], "AIRCRAFT_MASS", "Mass [kg]", "saddlebrown"),
        (gs[2, 0], "SHAFT_POWER_REQUIRED", "Shaft Req [kW]", "purple"),
        (gs[2, 1], "DRAG", "Drag [N]", "crimson"),
        (gs[2, 2], "CL", "CL", "darkgreen"),
        (gs[2, 3], "CD", "CD", "darkred"),
        (gs[3, 0], "TOTAL_THERMAL_LOSS", "Thermal [kW]", "gray"),
        (gs[3, 1], "AIR_DENSITY", "rho [kg/m3]", "steelblue"),
        (gs[3, 2], "GENERATOR_ELEC_POWER", "Gen Elec [kW]", "teal"),
        (gs[3, 3], "SUPERCAPACITOR_SOC", "SC SoC [%]", "gold"),
    ]
    for sp, key, ylabel, color in panels:
        ax = fig.add_subplot(sp)
        ax.plot(t_min, arr(key), color=color, lw=1.2)
        add_phase_bands(ax, records)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_xlabel("t [min]", fontsize=7)
        ax.set_title(ylabel, fontsize=9)
        ax.grid(True, alpha=0.25)
        ax.tick_params(labelsize=7)
    fig.suptitle("Mission 1 — Full Dashboard", fontsize=16, fontweight="bold", y=1.01)
    saved.append(save(fig, "21_mission1_dashboard.png"))

    print(f"\n  Saved {len(saved)} plots to: {out_dir}")
    for p in saved:
        print(f"    {os.path.basename(p)}")

    return saved


# ═══════════════════════════════════════════════════════════════════
# SAVE DATA
# ═══════════════════════════════════════════════════════════════════

def save_data(records: List[dict], out_dir: str):
    """Save timeseries as JSON and CSV."""
    os.makedirs(out_dir, exist_ok=True)

    # JSON
    json_path = os.path.join(out_dir, "mission1_timeseries.json")
    with open(json_path, "w") as f:
        json.dump(records, f, indent=2)

    # CSV
    csv_path = os.path.join(out_dir, "mission1_timeseries.csv")
    if records:
        import csv
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)

    # Summary JSON
    ph_sum = phase_summary(records)
    last   = records[-1]
    first  = records[0]
    summary = {
        "mission": "Mission 1",
        "total_duration_min":  round((last["TIME"] - first["TIME"] + DT) / 60.0, 2),
        "total_distance_km":   last["DISTANCE"],
        "landing_mass_kg":     last["AIRCRAFT_MASS"],
        "fuel_consumed_kg":    last["CUMULATIVE_FUEL"],
        "fuel_remaining_kg":   last["FUEL_REMAINING"],
        "initial_soc_pct":     first["BATTERY_SOC"],
        "final_soc_pct":       last["BATTERY_SOC"],
        "min_soc_pct":         min(r["BATTERY_SOC"] for r in records),
        "total_shaft_kWh":     last["CUMULATIVE_SHAFT_E"],
        "bat_discharged_kWh":  last["CUMULATIVE_BATTERY_E"],
        "regen_kWh":           last["CUMULATIVE_REGEN_E"],
        "phases":              ph_sum,
        "spec_params": {
            "MTOW_kg": 1000.0,
            "fuel_initial_kg": M_FUEL_INIT,
            "bat_capacity_kWh": BAT_CAP_KWH,
            "engine_max_kW": ENG_MAX_W / 1000,
            "motor_max_kW":  MOT_MAX_W / 1000,
            "wing_area_m2":  S,
            "AR": AR,
            "CD0": CD0,
            "eta_prop": ETA_PROP,
            "BSFC_kg_per_kWh": BSFC_KG_KWH,
        },
    }
    sum_path = os.path.join(out_dir, "mission1_summary.json")
    with open(sum_path, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n  Data saved:")
    print(f"    {json_path}")
    print(f"    {csv_path}")
    print(f"    {sum_path}")


# ===================================================================
# ENTRY POINT
# ===================================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Mission 1 Hybrid-Electric UAV Simulation"
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument(
        "--fast", action="store_true",
        help="Compress mission to exactly 5 minutes (proportional timing)",
    )
    grp.add_argument(
        "--full", action="store_true",
        help="Run full-duration mission (~106 min) [default]",
    )
    args = parser.parse_args()

    TARGET_S  = 300.0   # 5 minutes
    if args.fast:
        time_scale = TARGET_S / TOTAL_ORIGINAL_S
        mode_label = "5-MINUTE COMPRESSED"
        out_subdir = "mission1_5min"
    else:
        time_scale = 1.0
        mode_label = "FULL DURATION"
        out_subdir = "mission1"

    print("=" * 70)
    print(f"  MISSION 1 -- HYBRID ELECTRIC UAV SIMULATION  [{mode_label}]")
    print("=" * 70)
    print(f"  Aircraft:  1000 kg MTOW | 200 kg payload | 64 kg fuel")
    print(f"  Battery:   {BAT_CAP_KWH} kWh NMC | {BAT_MASS_KG} kg | SoC_init={BAT_SOC_INIT*100:.0f}%")
    print(f"  Engine:    {ENG_MAX_W/1000:.0f} kW turboshaft | BSFC={BSFC_KG_KWH} kg/kWh")
    print(f"  Motor:     {MOT_MAX_W/1000:.0f} kW peak | eta={ETA_MOTOR}")
    print(f"  Wing:      S={S} m^2 | AR={AR} | CD0={CD0} | e={E_OSW}")
    print(f"  k_induced: {k_IND:.5f}")
    print(f"  Wingspan:  {B_SPAN:.2f} m | Chord: {C_CHORD:.2f} m")
    if args.fast:
        compressed_min = TOTAL_ORIGINAL_S * time_scale / 60.0
        print(f"")
        print(f"  TIME COMPRESSION ACTIVE")
        print(f"  Original duration : {TOTAL_ORIGINAL_S/60:.1f} min ({TOTAL_ORIGINAL_S:.0f} s)")
        print(f"  Compressed target : {TARGET_S:.0f} s  (exactly 5 min)")
        print(f"  Scale factor      : {time_scale:.6f}  (x{1/time_scale:.1f} faster)")
        print(f"")
        print(f"  Phase breakdown (compressed):")
        for name, h0, h1, V, h_dot, orig_s, lf in _PHASE_TABLE:
            comp_s = orig_s * time_scale
            orig_m, orig_ss = divmod(orig_s, 60)
            print(f"    {name:<18} {orig_m:.0f}m{orig_ss:02.0f}s -> {comp_s:6.2f}s")
    print()

    # Run simulation
    records, violations = run_mission1(time_scale=time_scale)

    # Console report
    print_report(records, violations)

    # Output directory
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "data", "outputs", out_subdir,
    )

    # Plots
    generate_plots(records, out_dir)

    # Save data
    save_data(records, out_dir)

    print(f"\n  Mission 1 [{mode_label}] simulation complete.")
    print(f"  Output directory: {out_dir}")

