import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Constants
m = 1000.0 # kg
g = 9.81
W = m * g
S = 8.0 # m^2
CD0 = 0.025
AR = 10.0
e0 = 0.80
k = 1.0 / (np.pi * AR * e0)

T0 = 288.15
p0 = 101325.0
L = -0.0065
R = 287.058

def atm(h):
    T = T0 + L * h
    p = p0 * (T / T0) ** (-g / (L * R))
    rho = p / (R * T)
    return rho

def power_req(V, h, roc=0.0):
    rho = atm(h)
    q = 0.5 * rho * V**2
    CL = W / (q * S)
    CD = CD0 + k * CL**2
    D = q * S * CD
    T_req = D + W * (roc / V)
    return T_req * V

# 1. Cruise
h_cr = 5000.0
V_cr = 69.44 # 250 km/h
eta_p_cr = 0.82
P_cr_shaft = power_req(V_cr, h_cr) / eta_p_cr

# 2. Takeoff
# Spikes to ~ 1.5 * Cruise
P_to_shaft = P_cr_shaft * 1.5
eta_p_to = 0.60
t_to = 30.0

# 3. Climb
h_cl_start = 50.0
h_cl_end = 5000.0
roc_cl = 5.0
t_cl = (h_cl_end - h_cl_start) / roc_cl
V_cl = 45.0 # m/s
eta_p_cl = 0.72
# Average power during climb (using mid-altitude)
P_cl_shaft = power_req(V_cl, (h_cl_start + h_cl_end)/2.0, roc=roc_cl) / eta_p_cl

# 4. Loiter
h_lo = 3000.0
rho_lo = atm(h_lo)
V_be = ( (2*W)/(rho_lo*S) * np.sqrt(k / (3*CD0)) )**0.5
eta_p_lo = 0.65
P_lo_shaft = power_req(V_be, h_lo) / eta_p_lo
t_lo = 1200.0
t_cr = 1800.0

# 5. Descent
h_des_start = 3000.0
h_des_end = 0.0
rod = 3.0
t_des = (h_des_start - h_des_end) / rod
V_des = 45.0 # m/s
eta_p_des = 0.55 # windmill
P_des_shaft = power_req(V_des, (h_des_start + h_des_end)/2.0, roc=-rod) 
if P_des_shaft < 0:
    P_des_shaft = P_des_shaft * eta_p_des # regenerative power available at shaft
else:
    P_des_shaft = P_des_shaft / eta_p_des

# 6. Landing
t_lnd = 30.0
P_lnd_shaft = P_lo_shaft * 0.2 # 20% of loiter power for taxi/approach

phases = [
    {"phase": "Takeoff", "duration": t_to, "power_kW": P_to_shaft/1000},
    {"phase": "Climb", "duration": t_cl, "power_kW": P_cl_shaft/1000},
    {"phase": "Cruise", "duration": t_cr, "power_kW": P_cr_shaft/1000},
    {"phase": "Loiter", "duration": t_lo, "power_kW": P_lo_shaft/1000},
    {"phase": "Descent", "duration": t_des, "power_kW": P_des_shaft/1000},
    {"phase": "Landing", "duration": t_lnd, "power_kW": P_lnd_shaft/1000}
]

df = pd.DataFrame(phases)
df['t_start'] = df['duration'].cumsum() - df['duration']
df['t_end'] = df['duration'].cumsum()
df['Energy_kWh'] = df['power_kW'] * (df['duration'] / 3600.0)

print(df)
print("\nTotal Mission Time: {:.1f} s ({:.1f} min)".format(df['duration'].sum(), df['duration'].sum()/60.0))
print("Total Energy Demand (Shaft): {:.2f} kWh".format(df['Energy_kWh'].sum()))

# Plot
plt.figure(figsize=(10, 5))
time = []
power = []
for idx, row in df.iterrows():
    time.extend([row['t_start'], row['t_end']])
    power.extend([row['power_kW'], row['power_kW']])

plt.plot(time, power, lw=2, color='b')
plt.fill_between(time, power, alpha=0.2, color='b')
plt.xlabel("Time (s)")
plt.ylabel("Shaft Power (kW)")
plt.title("Mission Power Profile")
plt.grid(True)
plt.savefig('d:/College/yuvaudhyami/Hackathon/simulation/uav_hybrid_sim/sizing/mission_power_profile.png')

