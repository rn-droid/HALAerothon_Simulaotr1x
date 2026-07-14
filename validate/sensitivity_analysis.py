import sys
import os
import pandas as pd
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sizing.sizing_equations import size_system
from config import AERO, BATTERY, ENGINE

def run_sensitivity():
    # Base Design Vector from NSGA-II: [eng_kw, bat_kwh, sc_kwh, n_mot, mot_kw, ecms_s]
    base_dv = [60.0, 55.0, 0.5, 2, 60.0, 10.0]
    mission = {"loiter_duration_s": 1200}
    
    # 1. Battery Energy Density +/- 15%
    # In sizing_equations, battery mass isn't hardcoded in size_system, wait... 
    # Actually size_system gets bat_mass from df.attrs["battery_mass_kg"].
    # And the sim gets it from config.BATTERY.specific_energy_Wh_per_kg.
    # We need to temporarily monkey-patch config or pass it as an argument.
    pass

if __name__ == "__main__":
    print("Testing")
