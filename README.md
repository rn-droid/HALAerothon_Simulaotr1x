# Hybrid-Electric UAV Propulsion System — Digital Engineering Platform

A high-fidelity, interactive engineering dashboard for a **hybrid-electric fixed-wing UAV propulsion system**. This is a **system-level digital twin** that models the complete energy and power management of a hybrid-electric propulsion architecture using real engineering equations, research-backed assumptions, and a time-stepped simulation engine.

**Developed for the IIT Indore – HAL Hackathon.**

---

## Aircraft Configuration

| Parameter | Value |
|-----------|-------|
| MTOW | 1000 kg |
| Payload | 200 kg |
| Cruise Speed | 250 km/h (69.44 m/s) |
| Cruise Altitude | 3–10 km (design point: 5 km) |
| Prime Mover | 60 kW Turboshaft Engine |

### Propulsion Architecture

```
Fuel Tank → Turboshaft → AC Generator → Rectifier → DC Bus → Inverter → Motor(s) → Propeller(s)
                                                      ↑↓
                                                  Battery Pack
```

---

## Project Structure

```
uav_hybrid_sim/
├── config.py               # Central constants — single source of truth
│
├── sim/                    # [Legacy] Original simulation modules
│   ├── aircraft.py         #   ISA atmosphere + aerodynamic model
│   ├── engine.py           #   Turboshaft with BSFC table
│   ├── battery.py          #   Li-ion battery with SoC tracking
│   ├── motor.py            #   PM electric motor model
│   ├── mission.py          #   Mission profile builder (1 s steps)
│   ├── hybrid_sim.py       #   Power routing simulation loop
│   ├── ecms.py             #   ECMS power management controller
│   ├── pmp.py              #   Pontryagin's Minimum Principle benchmark
│   ├── clutch.py           #   Series-parallel clutch model
│   └── supercap.py         #   Supercapacitor transient buffer
│
├── simulation/             # [New] Refactored simulation engine
│
├── sizing/                 # Component sizing & mass convergence
│   └── sizing_equations.py #   Iterative mass-consistent sizing loop
│
├── optimization/           # [New] Multi-objective design exploration
│
├── dashboard/              # [New] Interactive engineering dashboard
│   └── app.py              #   Dashboard application (to be rebuilt)
│
├── data/                   # Input data, outputs, and cache
│   ├── inputs/             #   Static input tables (BSFC, propeller maps)
│   ├── outputs/            #   Simulation result CSVs
│   └── cache/              #   Cached optimization runs
│
├── docs/                   # Engineering documentation
│   ├── phase0_requirements.md    # Frozen design constraints & units
│   ├── assumption_register.md    # Master assumption register
│   ├── methodology.md            # Physics formulations summary
│   └── requirements_traceability.md
│
├── tests/                  # Unit and integration tests
├── validate/               # Model validation scripts
├── report/                 # Generated plots and reports
└── requirements.txt        # Python dependencies
```

---

## Engineering Workflow (8 Phases)

| Phase | Title | Status |
|-------|-------|--------|
| **0** | Requirements Freeze & Assumption Framework | ✅ Complete |
| **1** | Research & Engineering Assumptions | ⬜ Pending |
| **2** | Mission Profile & Power Demand | ⬜ Pending |
| **3** | Component Sizing | ⬜ Pending |
| **4** | Hybrid Power-Sharing Strategies | ⬜ Pending |
| **5** | Time-Stepped Simulation Engine | ⬜ Pending |
| **6** | Design Optimization | ⬜ Pending |
| **7** | Interactive Dashboard | ⬜ Pending |
| **8** | Validation & Sensitivity Analysis | ⬜ Pending |

---

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run the baseline simulation (legacy)
python -m sim.hybrid_sim

# Launch the dashboard (Phase 7+)
# python -m dashboard.app
```

---

## Units Convention

All internal calculations use **SI base units**. See [phase0_requirements.md](docs/phase0_requirements.md) for the full units table and conversion helpers available in `config.py`.

---

## License

Academic / Hackathon use. IIT Indore – HAL.
