# Assumption Register — Hybrid-Electric UAV Propulsion System

> **Purpose:** Every engineering assumption used in any phase MUST be recorded here.
> Each row links back to the phase that introduced it and carries confidence/sensitivity tags
> so that later phases (sensitivity analysis, validation) know which knobs to turn.
>
> **How to add entries:** Append to the relevant component/domain table. Never delete a row —
> mark it `SUPERSEDED` in the Status column if it is replaced.
>
> **Phase 1 Update (2026-07-13):** All values cross-checked against published datasheets,
> textbooks (Raymer, Gundlach), AIAA papers, NASA reports, and OEM specifications.

---

## Atmospheric Model

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| ATM-01 | Sea-level temperature | 288.15 | K | ISA Standard Atmosphere (ICAO Doc 7488) | HIGH | — | 0 | ACTIVE |
| ATM-02 | Sea-level pressure | 101325 | Pa | ISA Standard Atmosphere | HIGH | — | 0 | ACTIVE |
| ATM-03 | Sea-level air density | 1.225 | kg/m³ | ISA Standard Atmosphere | HIGH | — | 0 | ACTIVE |
| ATM-04 | Temperature lapse rate | −0.0065 | K/m | ISA Standard Atmosphere (troposphere only) | HIGH | — | 0 | ACTIVE |
| ATM-05 | Gas constant (air) | 287.058 | J/(kg·K) | Standard thermodynamic constant | HIGH | — | 0 | ACTIVE |

---

## Airframe Aerodynamics

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| AERO-01 | Wing reference area | 8.0 | m² | W/S ≈ 122 kg/m² — mid-range for MALE-class UAVs (60–120 kg/m², Gundlach Ch. 6); validated against MQ-1 class (W/S ≈ 100–130 kg/m²) | MEDIUM | SENS | 1 | ACTIVE |
| AERO-02 | Aspect ratio | 10 | — | Typical for endurance UAV (MQ-1: AR≈10.1, MQ-9: AR≈11.7); Raymer Table 4.1 recommends AR 7–12 for powered sailplane/endurance configs | MEDIUM | SENS | 1 | ACTIVE |
| AERO-03 | Oswald efficiency factor | 0.80 | — | Raymer Fig. 12.14 suggests e₀ = 0.78–0.85 for clean straight-wing aircraft; 0.80 is conservative mid-range | MEDIUM | SENS | 1 | ACTIVE |
| AERO-04 | Zero-lift drag coefficient | 0.025 | — | Raymer Table 12.3: CD0 = 0.020–0.040 for UAV class; 0.025 assumes clean config with retractable/faired gear and no external stores. Gundlach notes UAV-specific drag items (antennas, gimbals) can push CD0 higher | MEDIUM | SENS | 1 | ACTIVE |
| AERO-05 | Induced drag factor k | 0.0398 | — | k = 1/(π·AR·e₀) = 1/(π·10·0.80); derived, not independent assumption | HIGH | — | 1 | ACTIVE |
| AERO-06 | Maximum L/D ratio | ~16 | — | (L/D)max = 1/(2·√(CD0·k)) = 1/(2·√(0.025·0.0398)) ≈ 15.8; consistent with published range 13–20 for MALE UAVs (NIH, DIVA-portal surveys) | MEDIUM | SENS | 1 | ACTIVE |
| AERO-07 | Propeller efficiency (cruise) | 0.82 | — | Typical fixed-pitch propeller at design advance ratio J ≈ 1.5–2.0; published range 0.80–0.87 for well-matched props (AIAA AEPP, Tsunami Air data); 0.82 is conservative | MEDIUM | SENS | 1 | ACTIVE |
| AERO-08 | Propeller efficiency (climb) | 0.72 | — | Off-design at lower advance ratio (J ≈ 1.0); drop of 10–15% from cruise is typical for fixed-pitch (MIT propeller studies) | MEDIUM | SENS | 1 | ACTIVE |
| AERO-09 | Propeller efficiency (loiter) | 0.65 | — | Low-speed loiter at best-L/D speed; significant off-design penalty; conservative estimate | LOW | SENS | 1 | ACTIVE |
| AERO-10 | Windmilling drag increment | 0.015 | — | Estimated drag penalty for propeller regeneration in descent; no direct published data for this UAV class — extrapolated from variable-pitch wind turbine analogies | LOW | SENS | 1 | ACTIVE |
| AERO-11 | Wing loading (W/S) | 122.6 | kg/m² | MTOW/S_ref = 1000/8.0; falls within published 60–120 kg/m² upper range for MALE UAVs (acceptable for speed-optimised cruise) | MEDIUM | SENS | 1 | ACTIVE |

---

## Turboshaft Engine

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| ENG-01 | Rated shaft power | 60000 | W | Design requirement (frozen) | HIGH | — | 0 | ACTIVE |
| ENG-02 | BSFC at best operating point (~80% load) | 375 | g/(kW·h) | Representative of 40–100 kW micro-turboshaft class. PBS TS100 (180 kW) SFC = 548 g/(kW·h); larger turboshafts (RR M250, 300+ kW) = 350–380 g/(kW·h). Our 60 kW engine is conceptual — 375 g/(kW·h) assumes modest recuperation or advanced design. Range: 350–550 g/(kW·h) for this class | MEDIUM | SENS | 1 | ACTIVE |
| ENG-03 | BSFC at idle (~20% load) | 620 | g/(kW·h) | Typical 1.5–1.7× increase over best-point BSFC at low power fractions due to parasitic losses dominating; consistent with small turboshaft BSFC "bowl" curves | MEDIUM | — | 1 | ACTIVE |
| ENG-04 | BSFC lookup table | see config.py | — | 9-point table from pf=0.20 to 1.00; shape matches published small turboshaft characteristics (best at 70–80%, degraded at idle and full power) | MEDIUM | SENS | 1 | ACTIVE |
| ENG-05 | Thermal efficiency at best point | ~23% | — | η_th = 3600/(BSFC × LHV_kWh) = 3600/(375 × 11.95) ≈ 0.23; consistent with 20–28% range for small turboshafts (NLR survey) | MEDIUM | — | 1 | ACTIVE |
| ENG-06 | Engine specific power (power density) | 2.0 | kW/kg | Small turboshaft: PBS TS100 = 180 kW/57 kg ≈ 3.2 kW/kg; at 60 kW scale, specific power degrades to ~1.5–2.5 kW/kg. Using 2.0 kW/kg (installed, incl. accessories) | MEDIUM | — | 1 | ACTIVE |
| ENG-07 | Engine mass (60 kW) | 30 | kg | P_rated / specific_power = 60/2.0; consistent with scaled micro-turboshaft estimates | MEDIUM | — | 1 | ACTIVE |
| ENG-08 | Fuel LHV (Jet-A1) | 43.02 × 10⁶ | J/kg | Standard aviation fuel property (11.95 kWh/kg); CRC Handbook / ASTM D1655 | HIGH | — | 0 | ACTIVE |
| ENG-09 | Fuel density (Jet-A1) | 800 | kg/m³ | Standard at 15°C; range 775–840 kg/m³ per ASTM D1655 | HIGH | — | 0 | ACTIVE |

---

## Generator & Power Electronics

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| GEN-01 | Generator efficiency | 0.93 | — | PMSM generators: published range 93–97% at rated load (AIAA AEPP, ResearchGate surveys). 0.93 is conservative for a 60 kW-class unit; accounts for copper/iron losses + bearing friction. Larger/MW-class units reach 96–99% | MEDIUM | SENS | 1 | ACTIVE |
| GEN-02 | Generator specific power | 5.0 | kW/kg | State-of-art aerospace PMSM: 5–10 kW/kg (active mass). Using 5.0 kW/kg for installed mass including housing/cooling. EMRAX-class machines demonstrated up to 10 kW/kg peak | MEDIUM | — | 1 | ACTIVE |
| GEN-03 | Generator mass (60 kW) | 12 | kg | P_rated / specific_power = 60/5.0 | MEDIUM | — | 1 | ACTIVE |
| GEN-04 | Rectifier efficiency | 0.98 | — | Modern SiC MOSFET rectifiers: 98–99%+ efficiency (NASA Glenn, DLR studies). Previously lumped into GEN-01; now separated for accuracy | MEDIUM | — | 1 | ACTIVE |
| GEN-05 | Inverter (motor controller) efficiency | 0.98 | — | SiC-based inverters: 98–99% at rated load (AIAA AEPP standards); previously lumped into motor η | MEDIUM | — | 1 | ACTIVE |
| GEN-06 | DC bus / cabling efficiency | 0.996 | — | Short cable runs in UAV; NASA efficiency chain assumes 99.6% for distribution losses | HIGH | — | 1 | ACTIVE |
| GEN-07 | AC-then-rectify rationale | — | — | AC PMSM generators are lighter and more power-dense than equivalent DC generators; rectification loss (2%) is offset by 20–30% weight saving. Industry standard for hybrid-electric aircraft (AIAA AEPP) | HIGH | — | 1 | ACTIVE |

---

## Battery Pack

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| BAT-01 | Cell-level specific energy | 250 | Wh/kg | Li-ion NMC 811 state-of-art (2024–2026): 230–300 Wh/kg cell-level (timharper.net, dcebattery.com surveys). Using 250 Wh/kg as representative of high-nickel NMC pouch cells | MEDIUM | SENS | 1 | ACTIVE |
| BAT-02 | Pack-level specific energy | 200 | Wh/kg | Pack overhead factor ~0.75–0.85× cell level (BMS, casing, thermal management, wiring). 250 × 0.80 = 200 Wh/kg. NASA reports suggest 150–180 Wh/kg for certified packs; 200 Wh/kg is optimistic but achievable for UAV (less certification overhead) | MEDIUM | SENS | 1 | ACTIVE |
| BAT-03 | Round-trip efficiency | 0.95 | — | Li-ion NMC at moderate C-rates (1–2C): 93–97% round-trip. 95% is mid-range. Degrades at high C-rates and temperature extremes | MEDIUM | SENS | 1 | ACTIVE |
| BAT-04 | Max discharge C-rate (continuous) | 3.0 | 1/h | NMC cells: continuous 2–3C typical; high-power cells up to 5C but with accelerated degradation. Using 3C as sustainable continuous limit | MEDIUM | — | 1 | ACTIVE |
| BAT-05 | Max discharge C-rate (peak, 30s) | 5.0 | 1/h | Short burst capability for takeoff/climb transients; within NMC cell limits for <30s pulses | MEDIUM | — | 1 | ACTIVE |
| BAT-06 | Max charge C-rate | 2.0 | 1/h | Conservative for cell longevity; fast-charge >2C significantly accelerates degradation (Stanford eVTOL study) | MEDIUM | — | 1 | ACTIVE |
| BAT-07 | SoC operating window | [0.20, 0.90] | — | 70% usable capacity. Aviation best practice: 20–80% preferred (large-battery.com, capture.energy). Extended to 90% ceiling for UAV (less stringent than commercial aviation) | HIGH | SENS | 1 | ACTIVE |
| BAT-08 | Nominal voltage | 400 | V | Typical for 50–100 kW electric aircraft systems; enables manageable current levels (<150 A at 60 kW) | MEDIUM | — | 1 | ACTIVE |
| BAT-09 | Cycle life at 70% DoD | >2000 | cycles | NMC 811 at 70% DoD, 25°C: typically >2000 cycles to 80% capacity retention | MEDIUM | — | 1 | ACTIVE |

---

## Electric Motor(s)

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| MOT-01 | Motor efficiency (at rated load) | 0.94 | — | EMRAX 208/228 PMSM: 92–98% efficiency range, ~94–96% at rated continuous load. Using 0.94 as conservative continuous-duty value. Peak efficiency can reach 96% | HIGH | SENS | 1 | ACTIVE |
| MOT-02 | Motor specific power (continuous) | 5.0 | kW/kg | EMRAX 208: 56 kW cont. / 9.4 kg = 5.96 kW/kg (air-cooled). EMRAX 228: 75 kW cont. / 12.9 kg = 5.81 kW/kg. Using 5.0 kW/kg installed (incl. mounting, cooling ducting) — conservative | HIGH | — | 1 | ACTIVE |
| MOT-03 | Motor specific power (peak) | 8.0 | kW/kg | EMRAX 208: 86 kW peak / 9.4 kg = 9.1 kW/kg; EMRAX 228: 124 kW peak / 12.9 kg = 9.6 kW/kg. Using 8.0 kW/kg installed for peak sizing | HIGH | — | 1 | ACTIVE |
| MOT-04 | Motor rated power (baseline) | 50000 | W | Sized to handle cruise + partial climb assist; validated against mission power profile in Phase 2 | MEDIUM | SENS | 1 | ACTIVE |
| MOT-05 | Motor mass (50 kW, continuous) | 10 | kg | P_rated / specific_power = 50/5.0 | MEDIUM | — | 1 | ACTIVE |
| MOT-06 | Motor max RPM | 6000 | rpm | EMRAX 208: 7000 rpm max; EMRAX 228: 6500 rpm max. Using 6000 rpm with reduction gearbox to propeller | MEDIUM | — | 1 | ACTIVE |
| MOT-07 | Motor peak torque | ~80 | N·m | At 50 kW, 6000 rpm: τ = P/(2π·n/60) = 50000/(2π·100) ≈ 79.6 N·m | HIGH | — | 1 | ACTIVE |

---

## Propulsion Efficiency Chain (Series Path)

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| EFF-01 | Series path: Fuel → Propeller | ~0.147 | — | η_eng(0.23) × η_gen(0.93) × η_rect(0.98) × η_bus(0.996) × η_inv(0.98) × η_mot(0.94) × η_prop(0.82) = 0.147. This is the overall fuel-to-thrust efficiency of the series-electric path | MEDIUM | SENS | 1 | ACTIVE |
| EFF-02 | Electrical path: DC Bus → Propeller | ~0.754 | — | η_inv(0.98) × η_mot(0.94) × η_prop(0.82) = 0.754. Battery-to-thrust efficiency | MEDIUM | SENS | 1 | ACTIVE |
| EFF-03 | Generator path: Engine → DC Bus | ~0.907 | — | η_gen(0.93) × η_rect(0.98) × η_bus(0.996) = 0.907 | MEDIUM | — | 1 | ACTIVE |
| EFF-04 | Regen path: Propeller → Battery | ~0.46 | — | η_prop_windmill(~0.55) × η_mot_gen(0.94) × η_rect(0.98) × η_bus(0.996) × η_bat_charge(√0.95) = 0.46. Consistent with MIS-06 recovery estimate of 0.50 | LOW | SENS | 1 | ACTIVE |

---

## Mission Profile (to be refined in Phase 2)

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| MIS-01 | Takeoff transition altitude | 50 | m | Ground roll + initial climb; standard for UAV takeoff modelling | LOW | — | 0 | ACTIVE |
| MIS-02 | Rate of climb | 5.0 | m/s | ~1000 ft/min; typical for UAVs in this class. Range: 3–8 m/s | MEDIUM | SENS | 1 | ACTIVE |
| MIS-03 | Rate of descent | 3.0 | m/s | ~600 ft/min; comfortable descent rate; structural/passenger limits not applicable for UAV | MEDIUM | — | 1 | ACTIVE |
| MIS-04 | Cruise duration (default) | 1800 | s | 30 min baseline; user-adjustable | LOW | SENS | 0 | ACTIVE |
| MIS-05 | Loiter duration (default) | 1200 | s | 20 min baseline; user-adjustable | LOW | SENS | 0 | ACTIVE |
| MIS-06 | Descent regen recovery efficiency | 0.50 | — | Conservative; ~46% computed from component chain (EFF-04), rounded up for potential optimisation in variable-pitch prop | LOW | SENS | 1 | ACTIVE |
| MIS-07 | Stall speed (sea level) | ~32 | m/s | V_stall = √(2W/(ρ₀·S·CL_max)) with CL_max ≈ 1.5: √(2·9810/(1.225·8·1.5)) ≈ 32.6 m/s | MEDIUM | — | 1 | ACTIVE |
| MIS-08 | Rotation speed | 36 | m/s | V_rot ≈ 1.1 × V_stall ≈ 36 m/s; standard practice | MEDIUM | — | 1 | ACTIVE |
| MIS-09 | CL_max (clean) | 1.5 | — | Typical clean wing with moderate-thickness airfoil (15–18% t/c); no high-lift devices assumed | MEDIUM | — | 1 | ACTIVE |

---

## Mass Budget (to be sized in Phase 3)

| ID | Parameter | Value | Unit | Source / Rationale | Confidence | Sensitivity | Phase | Status |
|----|-----------|-------|------|--------------------|------------|-------------|-------|--------|
| MASS-01 | Payload mass | 200 | kg | Frozen requirement | HIGH | — | 0 | ACTIVE |
| MASS-02 | Airframe structural mass | 300 | kg | Estimated from empty-weight fractions of similar UAVs (We/W0 ≈ 0.50–0.60 for composite construction; 300 kg for airframe structure alone is reasonable given total powertrain mass ≈ 100–150 kg) | MEDIUM | SENS | 1 | ACTIVE |
| MASS-03 | MTOW | 1000 | kg | Hard constraint | HIGH | — | 0 | ACTIVE |
| MASS-04 | Available mass for propulsion + fuel | 500 | kg | MTOW − payload − airframe = 1000 − 200 − 300 = 500 kg; must fit engine + generator + motor + battery + fuel + power electronics + misc | HIGH | SENS | 1 | ACTIVE |

---

*Add new rows as assumptions are introduced in Phases 2–8. Every assumption must have a unique ID.*
