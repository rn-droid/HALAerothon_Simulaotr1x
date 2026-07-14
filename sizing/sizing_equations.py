import sys
import os
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.hybrid_sim import run_naive_hybrid
from sim.engine import Turboshaft
from sim.motor import ElectricMotor

def size_system(design_vector: list, mission_params: dict, use_adaptive: bool = False) -> dict:
    """
    design_vector = [
        engine_rated_kW, 
        battery_capacity_kWh, 
        supercap_capacity_kWh, 
        n_motors, 
        motor_kW_each, 
        ecms_s
    ]
    """
    eng_kw, bat_kwh, sc_kwh, n_mot, mot_kw, ecms_s = design_vector
    
    PAYLOAD_KG = 200.0
    AIRFRAME_KG = 300.0
    
    # Supercap specific energy is 5 Wh/kg -> 0.005 kWh/kg
    # Mass = capacity_kWh / 0.005 = capacity_kWh * 200
    sc_mass_kg = sc_kwh * 200.0
    
    mot_rated_W = mot_kw * 1000.0 * n_mot
    eng_rated_W = eng_kw * 1000.0
    
    # Generator mass (assume same specific power as motor, ~5 kW/kg)
    gen_mass_kg = eng_kw / 5.0
    
    # Engine mass (assume 2 kW/kg for small turboshaft)
    eng_mass = eng_rated_W / 2000.0
    
    # Initial guess for total mass
    total_mass = PAYLOAD_KG + AIRFRAME_KG + 500.0
    
    max_iter = 50
    tol = 0.5
    
    # print(f"--- Sizing Iteration for Design: {design_vector} ---")
    
    prev_soc_planned = None
    
    for i in range(max_iter):
        controller = None
        if use_adaptive and prev_soc_planned is not None:
            from sim.ecms import AdaptiveECMS
            controller = AdaptiveECMS(s_base=ecms_s, soc_planned=prev_soc_planned, gain=2.0)
            
        df = run_naive_hybrid(
            fault_flag=False,
            ecms_s=ecms_s if controller is None else None,
            ecms_controller=controller,
            loiter_duration_s=mission_params.get("loiter_duration_s", 1200),
            mtow_kg=total_mass,
            engine_rated_W=eng_rated_W,
            battery_capacity_kWh=bat_kwh,
            supercap_mass_kg=sc_mass_kg,
            motor_rated_W=mot_rated_W,
        )
        
        prev_soc_planned = df["soc"].values
        
        fuel_burned = df["fuel_burned_kg"].iloc[-1]
        final_soc = df["soc"].iloc[-1]
        
        clutch_mass = df.attrs["clutch_mass_kg"]
        bat_mass = df.attrs["battery_mass_kg"]
        mot_mass = df.attrs["motor_mass_kg"]
        
        new_mass = (AIRFRAME_KG + PAYLOAD_KG + fuel_burned + 
                    eng_mass + gen_mass_kg + clutch_mass + mot_mass + bat_mass + sc_mass_kg)
                    
        # print(f"  Iter {i+1}: Mass = {new_mass:.2f} kg (Prev: {total_mass:.2f} kg)")
        
        if abs(new_mass - total_mass) < tol:
            total_mass = new_mass
            break
            
        total_mass = 0.5 * total_mass + 0.5 * new_mass
        
    is_feasible = (total_mass <= 1000.0) and df.attrs.get("feasible", True)
        
    return {
        "converged_mass_kg": total_mass,
        "iterations": i + 1,
        "feasible": is_feasible,
        "final_soc": final_soc,
        "fuel_burned_kg": fuel_burned
    }

if __name__ == "__main__":
    mission = {"loiter_duration_s": 1200}
    
    # 1. Baseline: 60 kW engine, 10 kWh bat, 0.025 kWh sc (5 kg), 1 motor, 50 kW motor, s=9.0
    print("Testing Baseline Design...")
    res1 = size_system([60.0, 10.0, 0.025, 1, 50.0, 9.0], mission)
    
    # 2. Smaller Engine: 45 kW engine, 15 kWh bat, 0.05 kWh sc, 1 motor, 60 kW motor, s=9.0
    print("\nTesting Smaller Engine Design...")
    res2 = size_system([45.0, 15.0, 0.05, 1, 60.0, 9.0], mission)
    
    print("\n=========================================================================")
    print(f"{'Design':<15} | {'Mass (kg)':<10} | {'Feasible':<10} | {'Final SoC':<10} | {'Fuel (kg)':<10}")
    print("-------------------------------------------------------------------------")
    print(f"{'Baseline':<15} | {res1['converged_mass_kg']:<10.1f} | {str(res1['feasible']):<10} | {res1['final_soc']:<10.4f} | {res1['fuel_burned_kg']:<10.2f}")
    print(f"{'Small Engine':<15} | {res2['converged_mass_kg']:<10.1f} | {str(res2['feasible']):<10} | {res2['final_soc']:<10.4f} | {res2['fuel_burned_kg']:<10.2f}")
    print("=========================================================================")
