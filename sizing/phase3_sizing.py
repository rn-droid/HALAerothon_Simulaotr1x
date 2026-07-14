import numpy as np
import pandas as pd

# Constants & Assumptions
MTOW_target = 1000.0 # kg
m_payload = 200.0 # kg
m_structure = 300.0 # kg

eta_gen = 0.93
eta_rect = 0.98
eta_bus = 0.996
eta_inv = 0.98
eta_mot = 0.94
eta_elec_to_shaft = eta_bus * eta_inv * eta_mot
eta_eng_to_shaft = eta_gen * eta_rect * eta_elec_to_shaft # ~0.8362

spec_power_eng = 2.0 # kW/kg
spec_power_gen = 5.0 # kW/kg
spec_power_mot_cont = 5.0 # kW/kg
spec_power_mot_peak = 8.0 # kW/kg
spec_energy_bat = 0.200 # kWh/kg (200 Wh/kg)
soc_window = 0.70 # 20% to 90%

# From Phase 2 (shaft power required)
t_to = 30.0; P_to_shaft = 79.3
t_cl = 990.0; P_cl_shaft = 111.2
t_cr = 1800.0; P_cr_shaft = 52.9
t_lo = 1200.0; P_lo_shaft = 48.7
t_des = 1000.0; P_des_shaft = 0.6
t_lnd = 30.0; P_lnd_shaft = 9.7

# We iterate to find a converged mass budget.
# Let's parameterize the engine rating. It must be able to sustain cruise and charge battery if needed.
# Let's say engine power P_eng_rated is a variable.
P_eng_rated = 70.0 # Initial guess

for iteration in range(10):
    print(f"--- Iteration {iteration+1} ---")
    
    # 1. Turboshaft & Generator
    m_eng = P_eng_rated / spec_power_eng
    m_gen = P_eng_rated / spec_power_gen
    
    # 2. Motors (Twin motors sized for peak shaft power)
    P_mot_peak_total = P_cl_shaft # 111.2 kW
    # Sized for continuous climb (16 mins is long, let's use continuous rating for thermal safety)
    m_mot = P_cl_shaft / spec_power_mot_cont 
    
    # 3. Battery
    # Available engine power at shaft
    P_eng_shaft_max = P_eng_rated * eta_eng_to_shaft
    
    # Battery power required during climb
    P_bat_shaft_cl = max(0, P_cl_shaft - P_eng_shaft_max)
    P_bat_elec_cl = P_bat_shaft_cl / eta_elec_to_shaft
    E_bat_cl = P_bat_elec_cl * (t_cl / 3600.0)
    
    # Battery power during takeoff
    P_bat_shaft_to = max(0, P_to_shaft - P_eng_shaft_max)
    P_bat_elec_to = P_bat_shaft_to / eta_elec_to_shaft
    E_bat_to = P_bat_elec_to * (t_to / 3600.0)
    
    # Loiter: pure electric mode for stealth/reserve? 
    # Let's say loiter is done on battery to act as reserve.
    P_bat_elec_lo = P_lo_shaft / eta_elec_to_shaft
    E_bat_lo = P_bat_elec_lo * (t_lo / 3600.0)
    
    E_bat_req = E_bat_to + E_bat_cl + E_bat_lo
    E_bat_nom = E_bat_req / soc_window
    
    m_bat = E_bat_nom / spec_energy_bat
    
    # 4. Fuel
    # Takeoff & Climb: Engine runs at max power P_eng_rated
    # Cruise: Engine runs at whatever is needed for cruise + maybe recharge
    # Actually, we don't recharge if we sized battery for full loiter.
    P_eng_cr = P_cr_shaft / eta_eng_to_shaft
    
    # SFC modeling: let's assume 0.375 kg/kWh average for simplicity
    sfc = 0.375 
    
    fuel_to = P_eng_rated * sfc * (t_to/3600.0)
    fuel_cl = P_eng_rated * sfc * (t_cl/3600.0)
    fuel_cr = P_eng_cr * sfc * (t_cr/3600.0)
    fuel_des = (P_eng_rated * 0.2) * sfc * (t_des/3600.0) # Engine at idle
    fuel_lnd = P_eng_cr * sfc * (t_lnd/3600.0)
    
    m_fuel = fuel_to + fuel_cl + fuel_cr + fuel_des + fuel_lnd
    m_fuel_tank = m_fuel * 0.1 # 10% tank overhead
    
    # 5. Mass Check
    m_total = m_payload + m_structure + m_eng + m_gen + m_mot + m_bat + m_fuel + m_fuel_tank
    
    print(f"P_eng_rated: {P_eng_rated:.1f} kW, MTOW calc: {m_total:.1f} kg")
    print(f"Mass breakdown: Eng={m_eng:.1f}, Gen={m_gen:.1f}, Mot={m_mot:.1f}, Bat={m_bat:.1f}, Fuel+Tank={m_fuel+m_fuel_tank:.1f}")
    
    # If m_total > MTOW_target, we need to reduce battery reserve or increase MTOW.
    # The requirement is 1000 kg MTOW. 
    diff = MTOW_target - m_total
    
    if abs(diff) < 1.0:
        print("Converged!")
        break
    
    # We can't change the engine size indefinitely to fix mass, because a smaller engine means larger battery for climb.
    # Let's adjust engine size to see if it finds a sweet spot.
    P_eng_rated += diff * 0.1

