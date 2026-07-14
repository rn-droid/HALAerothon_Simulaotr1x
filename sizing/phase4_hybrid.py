import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# --- Constants & Assumed Efficiencies ---
eta_eng_to_shaft = 0.93 * 0.98 * 0.996 * 0.98 * 0.94 # ~0.836
eta_bat_to_shaft = 0.996 * 0.98 * 0.94 # ~0.917 (wait, in Phase 3 it was 0.899? Let's check: 0.996*0.98*0.94 = 0.9175. Ah, in Phase 3 I used 0.899 but that was a slight miscalc. I'll use 0.9175 here).
# Let's use exactly 0.9175
eta_elec_to_shaft = 0.9175
eta_eng_to_shaft = 0.93 * 0.98 * eta_elec_to_shaft # = 0.836

P_eng_max = 60.0 # kW
bat_cap_kWh = 55.0 # kWh
soc_init = 0.90
soc_min = 0.20

# Mission Profile (Duration, P_shaft_req)
phases = [
    {"name": "Takeoff", "t": 30.0, "P_req": 79.3},
    {"name": "Climb", "t": 990.0, "P_req": 111.2},
    {"name": "Cruise", "t": 1800.0, "P_req": 52.9},
    {"name": "Loiter", "t": 1200.0, "P_req": 48.7},
    {"name": "Descent", "t": 1000.0, "P_req": 0.6},
    {"name": "Landing", "t": 30.0, "P_req": 9.7}
]

# Simple SFC curve
def get_sfc(p_kw):
    if p_kw <= 0: return 0
    # Base 0.375 at 60kW, increases at part load
    load = np.clip(p_kw / P_eng_max, 0.1, 1.0)
    return 0.375 * (1 + 0.3 * (1 - load)**2)

# --- Simulate Strategy Function ---
def simulate_strategy(strategy_name, engine_rule):
    t_arr = []
    soc_arr = []
    p_eng_arr = []
    p_bat_arr = []
    fuel_arr = []
    
    current_t = 0.0
    current_soc = soc_init
    current_fuel = 0.0
    
    for phase in phases:
        dt = 1.0
        steps = int(phase['t'] / dt)
        
        for _ in range(steps):
            P_req = phase['P_req']
            
            # Determine engine power based on rule
            P_eng = engine_rule(phase['name'], P_req, current_soc)
            P_eng = np.clip(P_eng, 0, P_eng_max)
            
            # Shaft power from engine
            P_eng_shaft = P_eng * eta_eng_to_shaft
            
            # Deficit or Surplus at shaft
            P_shaft_diff = P_req - P_eng_shaft
            
            if P_shaft_diff > 0:
                # Battery must discharge
                P_bat_elec = P_shaft_diff / eta_elec_to_shaft
            else:
                # Battery can charge (surplus)
                # charging efficiency = 0.95 (from bat_rt=0.95 -> assume ~0.95 charge eff)
                P_bat_elec = P_shaft_diff * eta_elec_to_shaft * 0.95 
            
            # Update SoC
            energy_kwh = P_bat_elec * (dt / 3600.0)
            current_soc = current_soc - (energy_kwh / bat_cap_kWh)
            
            # Update Fuel
            fuel_burn = P_eng * get_sfc(P_eng) * (dt / 3600.0)
            current_fuel += fuel_burn
            
            current_t += dt
            
            t_arr.append(current_t)
            soc_arr.append(current_soc)
            p_eng_arr.append(P_eng)
            p_bat_arr.append(P_bat_elec)
            fuel_arr.append(current_fuel)
            
    return pd.DataFrame({
        'Time': t_arr, 'SoC': soc_arr, 'P_eng': p_eng_arr, 
        'P_bat': p_bat_arr, 'Fuel': fuel_arr
    })

# --- Define Strategies ---

# Strategy A: Pure Peak Shaving + Electric Loiter (Baseline)
# Engine max during TO/Climb/Cruise. Engine OFF during Loiter, Descent, Landing.
def rule_A(phase_name, p_req, soc):
    if phase_name in ["Takeoff", "Climb", "Cruise"]:
        return P_eng_max
    else:
        return 0.0

# Strategy B: Engine Sustained
# Engine max during TO/Climb. Engine matches cruise/loiter demand. Engine idle in descent/landing.
def rule_B(phase_name, p_req, soc):
    if phase_name in ["Takeoff", "Climb", "Cruise"]:
        return P_eng_max
    elif phase_name in ["Loiter"]:
        # Match loiter power exactly: P_req / eta
        return min(P_eng_max, p_req / eta_eng_to_shaft)
    else:
        # Idle
        return 12.0

# Strategy C: Aggressive Recharge
# Engine always runs at 60kW, excess power recharges battery.
def rule_C(phase_name, p_req, soc):
    if soc >= 0.90:
        return min(P_eng_max, p_req / eta_eng_to_shaft)
    return P_eng_max

res_A = simulate_strategy("Strategy A", rule_A)
res_B = simulate_strategy("Strategy B", rule_B)
res_C = simulate_strategy("Strategy C", rule_C)

# --- Plotting ---
plt.style.use('ggplot')
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# 1. SoC
axs[0].plot(res_A['Time']/60, res_A['SoC']*100, label="Strat A: EV Loiter", lw=2)
axs[0].plot(res_B['Time']/60, res_B['SoC']*100, label="Strat B: Engine Sustained", lw=2)
axs[0].plot(res_C['Time']/60, res_C['SoC']*100, label="Strat C: Aggressive Recharge", lw=2)
axs[0].axhline(20, color='r', linestyle='--', label='Min SoC Limit (20%)')
axs[0].axhline(90, color='g', linestyle='--', label='Max SoC Limit (90%)')
axs[0].set_ylabel('Battery SoC (%)')
axs[0].set_title('Battery SoC Trajectory')
axs[0].legend()

# 2. Fuel Burn
axs[1].plot(res_A['Time']/60, res_A['Fuel'], label=f"Strat A: {res_A['Fuel'].iloc[-1]:.1f} kg", lw=2)
axs[1].plot(res_B['Time']/60, res_B['Fuel'], label=f"Strat B: {res_B['Fuel'].iloc[-1]:.1f} kg", lw=2)
axs[1].plot(res_C['Time']/60, res_C['Fuel'], label=f"Strat C: {res_C['Fuel'].iloc[-1]:.1f} kg", lw=2)
axs[1].set_ylabel('Fuel Burned (kg)')
axs[1].set_title('Cumulative Fuel Burn')
axs[1].legend()

# 3. Power split (Baseline Strategy A)
axs[2].plot(res_A['Time']/60, res_A['P_eng'], label='Engine Power (kW)', color='darkred')
axs[2].plot(res_A['Time']/60, res_A['P_bat'], label='Battery Power (kW) (+ Disch, - Chg)', color='darkblue')
axs[2].set_ylabel('Power (kW)')
axs[2].set_xlabel('Mission Time (minutes)')
axs[2].set_title('Power Split (Baseline Strategy A)')
axs[2].legend()

plt.tight_layout()
plt.savefig('C:/Users/Lenovo/.gemini/antigravity/brain/86bcc250-beb3-41e4-b4fd-aaaeb620244d/phase4_hybrid_strategies.png')

# Output final stats
print("Strategy A (EV Loiter) - Final SoC: {:.1f}%, Fuel: {:.1f} kg".format(res_A['SoC'].iloc[-1]*100, res_A['Fuel'].iloc[-1]))
print("Strategy B (Eng Sustained) - Final SoC: {:.1f}%, Fuel: {:.1f} kg".format(res_B['SoC'].iloc[-1]*100, res_B['Fuel'].iloc[-1]))
print("Strategy C (Aggressive Chg) - Final SoC: {:.1f}%, Fuel: {:.1f} kg".format(res_C['SoC'].iloc[-1]*100, res_C['Fuel'].iloc[-1]))

