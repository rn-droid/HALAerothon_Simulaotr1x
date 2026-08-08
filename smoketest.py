import sys
sys.path.insert(0, '.')
from mission1_sim import run_mission1, TOTAL_ORIGINAL_S

recs, viol = run_mission1(time_scale=1.0)
L = recs[-1]
F = recs[0]

print("=== MISSION 1 PHYSICS ENGINE OUTPUT ===")
print("Total timesteps  :", len(recs))
print("Mission time     :", round(L["TIME"]/60, 1), "min")
print("Path distance    :", round(L["DISTANCE"], 1), "km")
print("Fuel consumed    :", round(L["CUMULATIVE_FUEL"], 2), "kg")
print("Fuel remaining   :", round(L["FUEL_REMAINING"], 2), "kg")
print("SOC initial      :", round(F["BATTERY_SOC"], 1), "%")
print("SOC final        :", round(L["BATTERY_SOC"], 1), "%")
soc_min = min(r["BATTERY_SOC"] for r in recs)
print("SOC minimum      :", round(soc_min, 1), "%")
print("Landing mass     :", round(L["AIRCRAFT_MASS"], 1), "kg")
print("Regen energy     :", round(L["CUMULATIVE_REGEN_E"], 3), "kWh")
print("Shaft energy     :", round(L["CUMULATIVE_SHAFT_E"], 2), "kWh")
print("Battery disch    :", round(L["CUMULATIVE_BATTERY_E"], 2), "kWh")
print("Violations       :", len(viol))
print("TOTAL_ORIGINAL_S :", TOTAL_ORIGINAL_S, "s =", round(TOTAL_ORIGINAL_S/60, 1), "min")
print()
print("=== FEASIBILITY CHECK ===")
checks = [
    ("Mission time ~107 min",  round(L["TIME"]/60,1), 107.0, 15.0),
    ("Distance km",            round(L["DISTANCE"],1), 317.0, 50.0),
    ("Fuel consumed kg",       round(L["CUMULATIVE_FUEL"],2), 39.0, 8.0),
    ("Fuel remaining kg",      round(L["FUEL_REMAINING"],2), 22.0, 8.0),
    ("Landing mass kg",        round(L["AIRCRAFT_MASS"],1), 959.0, 15.0),
    ("Final SOC %",            round(L["BATTERY_SOC"],1), 37.0, 15.0),
]
for label, val, tgt, tol in checks:
    ok = abs(val - tgt) <= tol
    print(f"  [{'OK  ' if ok else 'WARN'}] {label:<25}: {val:>8}  (target ~{tgt} ±{tol})")
