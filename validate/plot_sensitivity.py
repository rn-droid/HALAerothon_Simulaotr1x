import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Styling
plt.style.use('dark_background')
colors = ['#ff4400', '#00ff80', '#40a0d0']

# Data for Sensitivity
cases = ['-15% (Pessimistic)', 'Baseline', '+15% (Optimistic)']

# Battery Density impact on MTOW
# Baseline: 874 kg
# +15% (230 Wh/kg) -> -36 kg -> 838 kg
# -15% (170 Wh/kg) -> +48 kg -> 922 kg
bat_mtow = [922, 874, 838]

# SFC impact on MTOW (+/- 10%)
# Baseline fuel: 18.3 kg -> Total 874
# +10% fuel -> 20.1 -> 876
# -10% fuel -> 16.5 -> 872
sfc_mtow = [872, 874, 876] # Note: -10% SFC is Optimistic, +10% is Pessimistic, so we reverse it for the chart
sfc_mtow_plot = [876, 874, 872] 

# L/D impact on Cruise Power (and thus endurance)
# Baseline Power: 53 kW
# -10% L/D -> 58.3 kW
# +10% L/D -> 47.7 kW
ld_power = [58.3, 53.0, 47.7]

fig, ax1 = plt.subplots(figsize=(8, 5))
ax1.set_facecolor('#080c14')
fig.patch.set_facecolor('#04050a')

x = np.arange(len(cases))
width = 0.25

rects1 = ax1.bar(x - width, bat_mtow, width, label='Battery Energy Density (\u00b115%)', color=colors[1])
rects2 = ax1.bar(x, sfc_mtow_plot, width, label='Engine SFC (\u00b110%)', color=colors[0])

ax1.set_ylabel('MTOW (kg)', color='#ffffff')
ax1.set_title('Sensitivity Analysis: Impact on Aircraft MTOW', color='#ffffff', pad=20)
ax1.set_xticks(x)
ax1.set_xticklabels(cases)
ax1.legend(loc='upper right', frameon=False)
ax1.set_ylim(800, 950)
ax1.grid(True, axis='y', alpha=0.1)

# Add value labels
def autolabel(rects):
    for rect in rects:
        height = rect.get_height()
        ax1.annotate(f'{int(height)}',
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  
                    textcoords="offset points",
                    ha='center', va='bottom', color='#aaaaaa', fontsize=9)

autolabel(rects1)
autolabel(rects2)

plt.tight_layout()
plt.savefig('D:/College/yuvaudhyami/Hackathon/simulation/uav_hybrid_sim/report/sensitivity_mtow.png', dpi=300)

# L/D Plot
fig2, ax2 = plt.subplots(figsize=(8, 5))
ax2.set_facecolor('#080c14')
fig2.patch.set_facecolor('#04050a')

rects3 = ax2.bar(x, ld_power, 0.4, label='Aerodynamic L/D (\u00b110%)', color=colors[2])
ax2.set_ylabel('Cruise Power Required (kW)', color='#ffffff')
ax2.set_title('Sensitivity Analysis: Impact on Cruise Power', color='#ffffff', pad=20)
ax2.set_xticks(x)
ax2.set_xticklabels(['-10% L/D (Pessimistic)', 'Baseline', '+10% L/D (Optimistic)'])
ax2.set_ylim(40, 65)
ax2.grid(True, axis='y', alpha=0.1)
autolabel(rects3)

plt.tight_layout()
plt.savefig('D:/College/yuvaudhyami/Hackathon/simulation/uav_hybrid_sim/report/sensitivity_ld.png', dpi=300)

print("Sensitivity plots generated.")
