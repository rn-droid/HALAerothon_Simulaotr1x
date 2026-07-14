# Phase 0 — Requirements Freeze & Design Constraints

> **Status:** FROZEN — Do not modify without a formal design-change review.
> **Date Frozen:** 2026-07-13
> **Author:** Hackathon Team (IIT Indore – HAL)

---

## 1. Aircraft-Level Fixed Requirements

| # | Parameter | Symbol | Value | Unit (SI) | Unit (Aviation) | Notes |
|---|-----------|--------|-------|-----------|-----------------|-------|
| R-01 | Maximum Take-Off Weight | $W_{\text{MTOW}}$ | 9810 | N | 1000 kgf | Hard constraint |
| R-02 | Maximum Take-Off Mass | $m_{\text{MTOW}}$ | 1000 | kg | — | — |
| R-03 | Payload Mass | $m_{\text{pay}}$ | 200 | kg | — | Fixed; not tradeable |
| R-04 | Cruise Speed | $V_{\text{cr}}$ | 69.44 | m/s | 250 km/h | True Airspeed (TAS) |
| R-05 | Cruise Altitude Range | $h_{\text{cr}}$ | 3000 – 10000 | m | 9843 – 32808 ft | Design point: 5000 m |
| R-06 | Prime Mover Rated Power | $P_{\text{eng,rated}}$ | 60 | kW | 80.5 hp | Turboshaft |
| R-07 | Wing Reference Area | $S_{\text{ref}}$ | 8.0 | m² | — | Estimated; refine in Phase 2 |
| R-08 | Aspect Ratio | $AR$ | 10 | — | — | Typical for endurance UAV |
| R-09 | Oswald Efficiency Factor | $e_0$ | 0.80 | — | — | Clean config |
| R-10 | Zero-Lift Drag Coefficient | $C_{D_0}$ | 0.025 | — | — | Typical fixed-wing UAV |

---

## 2. Mission Profile Definition

The reference mission consists of **six sequential phases**. Phase durations and conditions are design variables to be refined in Phase 2; the *sequence* is frozen.

```
         ┌────────────────────────────────────┐
Altitude │        ┌─── Cruise ───┐            │
   (m)   │       /                \  Loiter   │
         │      / Climb            \_______   │
         │     /                           \  │
         │    /                      Descent\ │
         │   /                               \│
         └──┤ TO │                        │ LND├──► Time
            t₀   t₁        t₂    t₃   t₄    t₅
```

| Phase | Name | Altitude (m) | Speed | Duration | Power Demand | Notes |
|-------|------|-------------|-------|----------|-------------|-------|
| 0 | Takeoff & Initial Climb | 0 → 300 | $V_{\text{TO}}$ → $V_{\text{climb}}$ | ~60 s | Peak | Battery + Engine |
| 1 | Climb | 300 → $h_{\text{cr}}$ | $V_{\text{climb}}$ ≈ 55 m/s | Variable | High | Engine primary; battery boost |
| 2 | Cruise | $h_{\text{cr}}$ | $V_{\text{cr}}$ = 69.44 m/s | Primary phase | Moderate | Engine-only preferred |
| 3 | Loiter | $h_{\text{loiter}}$ ≤ $h_{\text{cr}}$ | $V_{\text{loiter}}$ < $V_{\text{cr}}$ | 15–30 min | Low–moderate | Engine at part-load + charge |
| 4 | Descent | $h_{\text{cr}}$ → 300 | $V_{\text{desc}}$ | Variable | Minimal | Regen if feasible |
| 5 | Landing | 300 → 0 | $V_{\text{app}}$ | ~60 s | Low | Battery reserve |

---

## 3. Propulsion Architecture

```
    ┌──────────┐     ┌──────────┐     ┌──────────┐
    │  Fuel    │────►│Turboshaft│────►│   AC     │
    │  Tank    │     │  Engine  │     │Generator │
    └──────────┘     └──────────┘     └──────────┘
                                           │
                                      ┌────▼────┐
                                      │Rectifier│
                                      └────┬────┘
                                           │
          ┌────────────┐            ┌──────▼──────┐
          │  Battery   │◄──────────►│  DC Power   │
          │   Pack     │            │    Bus      │
          └────────────┘            └──────┬──────┘
                                           │
                                    ┌──────▼──────┐
                                    │   Power     │
                                    │Electronics  │
                                    │  (Inverter) │
                                    └──────┬──────┘
                                           │
                                    ┌──────▼──────┐
                                    │  Electric   │
                                    │  Motor(s)   │
                                    └──────┬──────┘
                                           │
                                    ┌──────▼──────┐
                                    │ Propeller(s)│
                                    └─────────────┘
```

**Energy flow paths:**
1. **Turboshaft path:** Fuel → Engine → Generator → Rectifier → DC Bus → Inverter → Motor → Propeller
2. **Battery path:** Battery ↔ DC Bus → Inverter → Motor → Propeller
3. **Regenerative path (descent):** Propeller → Motor → Inverter → DC Bus → Battery

---

## 4. Units Convention

All internal calculations use **SI base units**. Aviation-common units are stored as display-only aliases.

| Quantity | SI Unit | Symbol | Aviation Alias |
|----------|---------|--------|---------------|
| Mass | kilogram | kg | kg |
| Length | metre | m | ft (×3.281) |
| Time | second | s | min, hr |
| Speed | m/s | m/s | km/h (×3.6), kts (×1.944) |
| Force | newton | N | kgf (÷9.81) |
| Power | watt | W | kW (÷1000), hp (÷745.7) |
| Energy | joule | J | Wh (÷3600), kWh (÷3.6e6) |
| Altitude | metre | m | ft (×3.281) |
| Temperature | kelvin | K | °C (−273.15) |
| Pressure | pascal | Pa | atm (÷101325) |
| Fuel flow | kg/s | kg/s | g/s (×1000), kg/hr (×3600) |
| Specific energy | J/kg | J/kg | Wh/kg (÷3600) |
| Angle | radian | rad | degree (×57.296) |

---

## 5. Notation Standard

| Convention | Example | Usage |
|-----------|---------|-------|
| Subscript for component | $P_{\text{eng}}$, $P_{\text{bat}}$, $P_{\text{mot}}$ | Power of engine, battery, motor |
| Subscript for phase | $P_{\text{cr}}$, $P_{\text{TO}}$ | Power during cruise, takeoff |
| Subscript "rated" | $P_{\text{eng,rated}}$ | Maximum continuous rating |
| Greek η for efficiency | $\eta_{\text{gen}}$, $\eta_{\text{mot}}$ | Component efficiency |
| Dot notation for rates | $\dot{m}_f$ | Fuel mass flow rate |
| Overline for averages | $\bar{\eta}_{\text{sys}}$ | Mission-average system efficiency |
| Δ for changes | $\Delta h$, $\Delta t$ | Altitude change, time step |

---

## 6. Key Physical Constants

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| Gravitational acceleration | $g$ | 9.80665 | m/s² |
| Sea-level air density | $\rho_0$ | 1.225 | kg/m³ |
| Sea-level temperature | $T_0$ | 288.15 | K |
| Temperature lapse rate | $\lambda$ | 0.0065 | K/m |
| Specific heat ratio (air) | $\gamma$ | 1.4 | — |
| Gas constant (air) | $R_{\text{air}}$ | 287.058 | J/(kg·K) |
| Jet-A1 specific energy (LHV) | $h_f$ | 43.02 × 10⁶ | J/kg |
| Jet-A1 density | $\rho_f$ | 800 | kg/m³ |

---

## 7. Confidence & Sensitivity Tagging

Every assumption in subsequent phases must be tagged with:

| Tag | Meaning |
|-----|---------|
| **HIGH** confidence | Well-established value from textbook / datasheet / published test data |
| **MEDIUM** confidence | Derived from analogous systems or engineering judgement |
| **LOW** confidence | Rough estimate; must be refined or validated |
| **SENS** (Sensitivity flag) | Parameter significantly affects outputs; include in sensitivity sweep |

---

*This document is the single source of truth for all frozen requirements. All subsequent phases inherit these values via the central `config.py` constants module.*
