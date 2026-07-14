import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.termination import get_termination

from sizing.sizing_equations import size_system

class UAVHybridProblem(ElementwiseProblem):
    def __init__(self, mission_params):
        # design_vector = [engine_kW, battery_kWh, supercap_kWh, n_motors, motor_kW, ecms_s]
        # engine: 40-80
        # battery: 5-40
        # supercap: 0.5-5
        # n_motors: 1-4
        # motor_kW: 10-40
        # ecms_s: 4.5-18.0
        
        xl = np.array([40.0, 5.0, 0.5, 1, 10.0, 4.5])
        xu = np.array([80.0, 40.0, 5.0, 4, 40.0, 18.0])
        
        super().__init__(
            n_var=6,
            n_obj=2,   # 0: -endurance (maximize), 1: converged mass (minimize)
            n_ieq_constr=4, # mass <= 1000, soc >= 0.2, soc <= 0.9, power_fail
            xl=xl,
            xu=xu
        )
        
        self.mission_params = mission_params

    def _evaluate(self, x, out, *args, **kwargs):
        x_rounded = list(x)
        x_rounded[3] = int(round(x[3]))
        
        try:
            res = size_system(x_rounded, self.mission_params, use_adaptive=True)
            mass = res["converged_mass_kg"]
            soc = res["final_soc"]
            fuel = res["fuel_burned_kg"]
            feasible = res["feasible"]
            
            # T_nom ~ 4600s
            t_nom = 4600.0
            spare_mass = 1000.0 - mass
            
            if spare_mass > 0:
                avg_fuel_flow = fuel / t_nom
                endurance_s = t_nom + (spare_mass / avg_fuel_flow)
            else:
                endurance_s = t_nom
                
            endurance_h = endurance_s / 3600.0
            
            power_fail = 0.0 if feasible else 1.0
            
            out["F"] = [-endurance_h, mass]
            out["G"] = [
                mass - 1000.0,  # mass <= 1000 => mass - 1000 <= 0
                0.20 - soc,     # soc >= 0.20 => 0.20 - soc <= 0
                soc - 0.90,     # soc <= 0.90 => soc - 0.90 <= 0
                power_fail
            ]
        except Exception as e:
            # If simulation fails, penalize heavily
            out["F"] = [0.0, 1e6]
            out["G"] = [1e6, 1e6, 1e6, 1e6]


if __name__ == "__main__":
    mission = {"loiter_duration_s": 1200}
    problem = UAVHybridProblem(mission)
    
    algorithm = NSGA2(
        pop_size=30,
        eliminate_duplicates=True
    )
    
    termination = get_termination("n_gen", 20)
    
    print("Starting NSGA-II Optimization (this will take a minute or two)...")
    res = minimize(
        problem,
        algorithm,
        termination,
        seed=1,
        save_history=True,
        verbose=True
    )
    
    # Process results
    F = res.F
    X = res.X
    
    if F is None or X is None:
        print("No feasible solutions found!")
        sys.exit(1)
        
    endurance = -F[:, 0]
    mass = F[:, 1]
    
    # Plot Pareto front
    plt.figure(figsize=(10, 6))
    plt.scatter(endurance, mass, c='b', alpha=0.7)
    plt.xlabel("Endurance [hours]")
    plt.ylabel("Converged Mass [kg]")
    plt.title("NSGA-II Pareto Front (Endurance vs Mass)")
    plt.grid(True, alpha=0.3)
    
    report_dir = os.path.join(os.path.dirname(__file__), "..", "report")
    os.makedirs(report_dir, exist_ok=True)
    p_path = os.path.join(report_dir, "pareto_front.png")
    plt.savefig(p_path, dpi=150)
    print(f"\nPareto front saved to {os.path.abspath(p_path)}")
    
    # Sort by endurance descending
    sorted_idx = np.argsort(endurance)[::-1]
    
    print("\nTop 3 Feasible Designs (by Endurance):")
    print("="*80)
    pareto_data = []
    for i in range(len(sorted_idx)):
        idx = sorted_idx[i]
        x_des = X[idx]
        e_val = endurance[idx]
        m_val = mass[idx]
        n_mot = int(round(x_des[3]))
        
        pareto_data.append({
            "rank": i + 1,
            "endurance_h": float(e_val),
            "mass_kg": float(m_val),
            "design_vector": [float(x) for x in x_des]
        })
        
        if i < 3:
            print(f"Rank {i+1}: Endurance = {e_val:.2f} h, Mass = {m_val:.1f} kg")
            print(f"  Engine: {x_des[0]:.1f} kW | Battery: {x_des[1]:.1f} kWh | Supercap: {x_des[2]:.2f} kWh")
            print(f"  Motors: {n_mot} x {x_des[4]:.1f} kW | ECMS s: {x_des[5]:.2f}")
            print("-" * 80)

    # Save pareto front JSON
    json_path = os.path.join(report_dir, "pareto_front.json")
    import json
    with open(json_path, "w") as f:
        json.dump(pareto_data, f, indent=4)
    print(f"Saved pareto data to {os.path.abspath(json_path)}")
