"""
Offline Pontryagin's Minimum Principle (PMP) solver.

Finds the theoretical optimal engine/battery split by minimising the
Hamiltonian at each timestep:
    H = m_dot_f(P_eng) + lambda * (-P_bat_internal / E_batt)

A shooting method (bisection) is used to find the constant costate
lambda that results in a charge-sustaining mission (final SoC = initial SoC).
PMP freely chooses between parallel and series modes to minimise H,
unlike ECMS which was constrained to a heuristic clutch rule.
"""

import sys
import os
import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.aircraft import Aircraft
from sim.engine import Turboshaft
from sim.mission import MissionProfile
from sim.battery import Battery
from sim.motor import ElectricMotor
from sim.clutch import Clutch


class PMPController:
    def __init__(self, lam: float):
        self.lam = lam

    def minimize_H(self, p_req, eng, mot, gen_eff, bat):
        """
        Find (mode, p_eng) that minimizes Hamiltonian H for a given p_req.
        """
        E_batt_J = bat.capacity_Wh * 3600.0
        eta_dis = bat._eta_discharge
        eta_chg = bat._eta_charge

        def calc_H_series(p_eng):
            # Mechanical power to/from prop that motor must handle
            # Actually, P_req is prop demand. Motor output is P_mot_mech.
            # P_eng goes into generator. 
            # P_bat_bus + P_eng * gen_eff = P_req / mot_eff   (if P_req > 0)
            
            if p_req >= 0:
                p_bus = p_req / mot.efficiency
            else:
                # Descent regen: prop drives motor as generator
                # Motor mechanical power in = -p_req
                # Motor electrical power out = -p_req * mot.efficiency
                # So the bus sees a supply of (-p_req * mot_eff)
                # We need P_bat_bus + P_eng * gen_eff = - (-p_req * mot.efficiency)
                p_bus = p_req * mot.efficiency
            
            p_bat_bus = p_bus - p_eng * gen_eff
            
            if p_bat_bus >= 0:
                p_internal = p_bat_bus / eta_dis
            else:
                p_internal = p_bat_bus * eta_chg
                
            m_dot = eng.fuel_flow_kg_s(p_eng)
            H = m_dot - self.lam * (p_internal / E_batt_J)
            return H, p_bat_bus

        # 1. Optimize Series Mode
        # Engine bounds
        # Engine can be anything in [0, rated_power]
        # But we must respect battery C-rate limits for this timestep evaluation.
        # Actually, PMP theoretically minimizes H regardless of state bounds,
        # but to be physically realizable we restrict P_eng such that P_bat is within limits.
        
        # Max discharge internal power
        p_bat_max_dis_internal = bat.max_discharge_W / eta_dis
        # Max charge internal power
        p_bat_max_chg_internal = bat.max_charge_W * eta_chg
        
        p_bus_series = (p_req / mot.efficiency) if p_req >= 0 else (p_req * mot.efficiency)
        
        # P_bat_bus = p_bus_series - P_eng * gen_eff
        # -bat.max_charge_W <= p_bus_series - P_eng * gen_eff <= bat.max_discharge_W
        p_eng_lo = max(0.0, (p_bus_series - bat.max_discharge_W) / gen_eff)
        p_eng_hi = min(eng.rated_power_W, (p_bus_series + bat.max_charge_W) / gen_eff)
        
        if p_eng_lo > eng.rated_power_W:
            p_eng_lo = eng.rated_power_W
        if p_eng_hi < p_eng_lo:
            p_eng_hi = p_eng_lo
            
        res = minimize_scalar(
            lambda p: calc_H_series(p)[0],
            bounds=(p_eng_lo, p_eng_hi),
            method='bounded'
        )
        p_eng_series_opt = float(res.x)
        H_series, _ = calc_H_series(p_eng_series_opt)
        
        # 2. Evaluate Parallel Mode (only valid if P_req > 0 and P_req <= eng.rated_power_W)
        H_parallel = float('inf')
        if 0 < p_req <= eng.rated_power_W:
            p_eng_parallel = p_req
            p_bat_bus_parallel = 0.0
            p_internal_parallel = 0.0
            m_dot_parallel = eng.fuel_flow_kg_s(p_eng_parallel)
            H_parallel = m_dot_parallel - self.lam * (p_internal_parallel / E_batt_J)
            
        # 3. Choose best mode
        if H_parallel < H_series:
            return "parallel", p_req, 0.0
        else:
            return "series", p_eng_series_opt, p_bus_series - p_eng_series_opt * gen_eff


def run_pmp_sim(lam: float) -> pd.DataFrame:
    """Run full simulation using PMP controller with fixed lambda."""
    ac = Aircraft()
    eng = Turboshaft(rated_power_W=60_000.0)
    bat = Battery(capacity_kWh=10.0)
    mot = ElectricMotor(rated_power_W=50_000.0)
    
    mp = MissionProfile(
        aircraft=ac, cruise_alt_m=3000.0,
        cruise_duration_s=1800, loiter_duration_s=1200
    )
    df = mp.build()
    
    pmp = PMPController(lam)
    
    dt_s = 1.0
    soc_log, fuel_burned_log = [], []
    p_eng_log, p_bat_log, mode_log = [], [], []
    total_fuel = 0.0
    
    for _, row in df.iterrows():
        p_req = row["power_required_W"]
        
        mode, p_eng_actual, p_bat_needed = pmp.minimize_H(p_req, eng, mot, 0.93, bat)
        
        if mode == "parallel":
            p_bat_actual = 0.0
        else:
            p_bat_actual = bat.step(p_bat_needed, dt_s)
            # If battery hit limits and couldn't provide enough, engine must make up shortfall
            shortfall = p_bat_needed - p_bat_actual
            if abs(shortfall) > 1.0:
                p_eng_actual += shortfall / 0.93
                p_eng_actual = min(max(p_eng_actual, 0.0), eng.rated_power_W)
                
        m_dot = eng.fuel_flow_kg_s(p_eng_actual)
        total_fuel += m_dot * dt_s
        
        soc_log.append(bat.soc)
        fuel_burned_log.append(total_fuel)
        p_eng_log.append(p_eng_actual)
        p_bat_log.append(p_bat_actual)
        mode_log.append(mode)
        
    df["soc"] = soc_log
    df["fuel_burned_kg"] = fuel_burned_log
    df["p_eng_W"] = p_eng_log
    df["p_bat_W"] = p_bat_log
    df["mode"] = mode_log
    return df


def calibrate_pmp():
    """Bisection search to find charge-sustaining lambda."""
    target_soc = 0.90
    lam_lo, lam_hi = -100.0, 1000.0  # Wide initial bounds
    
    print("Finding PMP charge-sustaining lambda...")
    
    # 1. Coarse bounds check
    soc_lo = run_pmp_sim(lam_lo)["soc"].iloc[-1]
    soc_hi = run_pmp_sim(lam_hi)["soc"].iloc[-1]
    
    if (soc_lo - target_soc) * (soc_hi - target_soc) > 0:
        print("Warning: Bisection bounds do not bracket target.")
        
    # 2. Bisection
    for i in range(25):
        if lam_hi - lam_lo < 0.1:
            break
        lam_mid = (lam_lo + lam_hi) / 2.0
        soc_mid = run_pmp_sim(lam_mid)["soc"].iloc[-1]
        
        if soc_mid > target_soc:
            lam_hi = lam_mid
        else:
            lam_lo = lam_mid
            
    lam_cal = (lam_lo + lam_hi) / 2.0
    return lam_cal

if __name__ == "__main__":
    lam = calibrate_pmp()
    print(f"Optimal Lambda: {lam:.2f}")
    df = run_pmp_sim(lam)
    print(f"Final SoC: {df['soc'].iloc[-1]:.4f}")
    print(f"Fuel burned: {df['fuel_burned_kg'].iloc[-1]:.2f} kg")
