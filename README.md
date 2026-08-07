# Aerothon Hybrid-Electric UAV Propulsion Simulator

Aerothon is a digital engineering platform for a fixed-wing hybrid-electric UAV propulsion system. The project models a 1000 kg class UAV with a 60 kW turboshaft prime mover, battery-assisted electric propulsion, supercapacitor peak shaving, regenerative descent, mission-level power management, sizing, validation, optimization, and dashboard visualization.

The codebase was developed for the IIT Indore - HAL Aerothon / hackathon workflow. It is not a visual mockup: the outputs are generated from time-stepped physics and control logic implemented in Python.

---

## 1. What This Project Does

The simulator answers the main design question:

> Can a hybrid-electric UAV satisfy the mission, MTOW, payload, power, fuel, and battery constraints while improving fuel use versus a conventional turboshaft baseline?

It does this by combining:

- ISA atmosphere and altitude-dependent density.
- Fixed-wing lift, drag, induced drag, thrust, and shaft-power calculations.
- Mission phases: takeoff, climb, cruise, loiter, descent, landing.
- Turboshaft fuel burn using a BSFC part-load map.
- Generator, rectifier, DC bus, inverter, and motor efficiency chain.
- Battery SoC integration with C-rate and SoC constraint checks.
- Supercapacitor peak-power support for takeoff and climb transients.
- Series, parallel, series-parallel, and regenerative operating modes.
- Adaptive ECMS-style power split logic.
- Mission 1 report generation and phase summaries.
- Parametric sweep and NSGA-II design optimization.
- Flask and Dash dashboards for interactive analysis.

---

## 2. Headline Mission 1 Result

The generated Mission 1 outputs in `data/outputs/mission1_summary.json` currently report:

| Metric | Value |
|---|---:|
| MTOW | 1000.0 kg |
| Payload | 200.0 kg |
| Engine rating | 60.0 kW |
| Motor rating | 70.0 kW |
| Battery capacity | 40.0 kWh |
| Initial SoC | 90.0 % |
| Supercapacitor energy | 0.25 kWh |
| Cruise altitude | 3000 m |
| Cruise speed | 69.44 m/s |
| Mission time | 76.85 min |
| Ground distance | 232.53 km |
| Hybrid fuel burned | 20.64 kg |
| Conventional comparison fuel | 25.68 kg |
| Fuel reduction | 19.62 % |
| Final SoC | 47.11 % |
| Minimum SoC | 37.86 % |
| Minimum stall margin | 1.20 x |
| Constraint-flagged timesteps | 0 |

Mission 1 phase behavior:

| Phase | Duration min | Architecture | Main mode | Avg shaft kW | Avg engine kW | Avg battery kW | Avg supercap kW | Fuel kg | SoC start -> end |
|---|---:|---|---|---:|---:|---:|---:|---:|---:|
| Takeoff | 0.05 | Series-parallel | Peak assist | 123.00 | 60.00 | 32.57 | 42.94 | 0.011 | 90.00 -> 89.93 |
| Climb | 16.67 | Series-parallel | Peak assist | 67.50 | 58.04 | 11.99 | 7.96 | 5.386 | 89.92 -> 81.39 |
| Cruise | 15.00 | Parallel | Parallel assist | 119.89 | 60.00 | 67.89 | 0.00 | 5.072 | 81.34 -> 37.86 |
| Loiter | 30.00 | Parallel | Charge sustain | 49.77 | 60.00 | -7.19 | 0.00 | 10.150 | 37.86 -> 46.62 |
| Descent | 14.28 | Series regen | Regen descent | 1.69 | 0.00 | -0.85 | 0.30 | 0.000 | 46.62 -> 47.11 |
| Landing | 0.87 | Idle | Landing idle | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 | 47.11 -> 47.11 |

Positive battery power means discharge. Negative battery power means charging.

---

## 3. Aircraft and Propulsion Configuration

### Frozen aircraft-level requirements

| Parameter | Value |
|---|---:|
| Maximum takeoff mass | 1000 kg |
| Payload | 200 kg |
| Cruise speed | 250 km/h = 69.44 m/s |
| Cruise altitude range | 3000-10000 m |
| Default design cruise altitude | 5000 m |
| Prime mover | 60 kW turboshaft |
| Fuel | Jet-A1 |

### Baseline series-hybrid architecture

```text
Fuel tank
   -> Turboshaft engine
   -> AC generator
   -> Rectifier
   -> DC bus
   -> Inverter
   -> Electric motor(s)
   -> Propeller(s)

Battery pack <-> DC bus
Supercapacitor -> DC bus / peak shaft assist logic
Propeller -> motor-generator -> DC bus -> battery during regenerative descent
```

### Series-parallel control architecture

The newer controller also models a clutch-selected series-parallel architecture:

- Series mode: engine power passes through generator and electric drivetrain.
- Parallel mode: engine and motor power are coupled mechanically to the shaft.
- Series-parallel peak mode: takeoff and climb use engine, battery, and supercapacitor support.
- Regen mode: descent opens the clutch and treats the windmilling propeller as a generator source.
- Idle mode: landing can command zero propulsion power.

---

## 4. Physics and Engineering Logic

All internal calculations use SI units. Constants and assumptions are centralized in `config.py` and documented in `docs/assumption_register.md`.

### 4.1 Atmosphere

Implemented in `simulation/atmosphere.py`.

The simulator uses the ISA troposphere model:

```text
T(h) = T0 + L h
p(h) = p0 * (T(h) / T0)^(-g / (L R))
rho(h) = p(h) / (R T(h))
```

Where:

- `T0 = 288.15 K`
- `p0 = 101325 Pa`
- `L = -0.0065 K/m`
- `R = 287.058 J/(kg K)`
- `g = 9.80665 m/s^2`

This makes lift, drag, stall speed, and power demand change with altitude.

### 4.2 Aerodynamics

Implemented in `simulation/aerodynamics.py`.

The aerodynamic model uses a parabolic drag polar:

```text
q = 0.5 rho V^2
CL = L_required / (q S)
k = 1 / (pi AR e0)
CD_i = k CL^2
CD = CD0 + CD_i
D = q S CD
L/D = CL / CD
```

For non-level flight, the model resolves flight-path angle from rate of climb:

```text
sin(gamma) = ROC / V
L_required = n W cos(gamma)
T_required = D + W sin(gamma)
P_propulsive = T_required V
P_shaft = P_propulsive / eta_prop
```

Stall checks are also computed:

```text
V_stall = sqrt(2 W / (rho S CL_max))
V_min_safe = stall_margin * V_stall
```

The code records stall speed, safe speed, dynamic pressure, CL, CD, induced drag, thrust required, and delivered shaft power at each timestep.

### 4.3 Mission profile

Implemented in `simulation/mission_profile.py`.

The default mission builder creates a six-phase mission:

1. Takeoff and initial climb.
2. Climb to cruise altitude.
3. Cruise.
4. Loiter.
5. Regenerative descent.
6. Landing.

The named `mission1` preset uses:

| Parameter | Value |
|---|---:|
| MTOW | 1000 kg |
| Payload | 200 kg |
| Fuel start | 64 kg |
| Battery | 40 kWh |
| Initial SoC | 0.90 |
| Supercap | 0.25 kWh |
| Engine | 60 kW |
| Motor | 70 kW |
| Cruise altitude | 3000 m |
| Cruise speed | 69.44 m/s |
| Climb speed | 40 m/s |
| Climb rate | 3 m/s |
| Loiter speed | 50 m/s |
| Loiter bank | 25 deg |
| Descent speed | 45 m/s |
| Descent rate | 3.5 m/s |
| Wing area | 25 m^2 |
| Aspect ratio | 15 |
| CD0 | 0.025 |
| Cruise duration | 15 min |
| Loiter duration | 30 min |

The general mission builder supports user-adjustable cruise time, loiter time, cruise altitude, loiter altitude, climb/descent rates, distance-based cruise/climb, headwind, wing area, aspect ratio, Oswald efficiency, and CD0.

### 4.4 Engine and fuel burn

Implemented in `simulation/engine_model.py`.

The turboshaft is represented by a rated-power limit and a BSFC lookup table:

| Power fraction | 0.20 | 0.30 | 0.40 | 0.50 | 0.60 | 0.70 | 0.80 | 0.90 | 1.00 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| BSFC g/kWh | 620 | 520 | 460 | 420 | 395 | 380 | 375 | 380 | 395 |

At every timestep:

```text
P_engine = clamp(P_commanded, 0, P_rated)
power_fraction = P_engine / P_rated
BSFC = interpolate(power_fraction)
fuel_kg = (BSFC / 1000) * P_engine_kW * dt_hr
```

The model also estimates thermal efficiency using Jet-A1 lower heating value.

### 4.5 Electrical power chain

Implemented in `simulation/power_electronics.py`.

Default component efficiencies:

| Component | Efficiency |
|---|---:|
| Generator | 0.93 |
| Rectifier | 0.98 |
| DC bus / cabling | 0.996 |
| Inverter | 0.98 |
| Motor | 0.94 |

Derived chain efficiencies:

```text
eta_engine_to_dc = eta_generator * eta_rectifier * eta_bus
eta_dc_to_shaft = eta_inverter * eta_motor
eta_engine_to_shaft = eta_engine_to_dc * eta_dc_to_shaft
eta_battery_to_shaft = eta_bus * eta_inverter * eta_motor
```

The simulator distinguishes between:

- Series electrical path from engine to propeller shaft.
- Parallel mechanical path with clutch efficiency.
- Battery-to-shaft electric path.
- Regenerative shaft-to-battery path.

### 4.6 Battery model

Implemented in `simulation/battery_model.py`.

The battery is a Li-ion NMC pack with:

| Parameter | Value |
|---|---:|
| Default capacity | 55 kWh |
| Mission 1 capacity | 40 kWh |
| Pack specific energy | 200 Wh/kg |
| Round-trip efficiency | 0.95 |
| Continuous discharge limit | 3C |
| Peak discharge limit | 5C |
| Charge limit | 2C |
| Minimum SoC | 0.20 |
| Nominal voltage | 400 V |

The sign convention is:

```text
P_battery > 0  => discharge
P_battery < 0  => charge
```

Efficiency is split symmetrically:

```text
eta_discharge = sqrt(0.95)
eta_charge = sqrt(0.95)
```

The model performs energy integration:

```text
discharge internal energy = P_terminal / eta_discharge * dt
charge internal energy = P_terminal * eta_charge * dt
SoC_new = SoC_old - energy_internal / capacity
```

It clamps power if C-rate limits are exceeded and flags violations for SoC and charge/discharge limits.

### 4.7 Power split controller

Implemented in `simulation/controller.py`.

The controller is an adaptive series-parallel power manager with ECMS-style SoC feedback. It receives:

- Flight phase.
- Required shaft power.
- Engine-to-shaft efficiency.
- Battery-to-shaft efficiency.
- Current battery SoC.

It returns:

- Engine command.
- Battery electrical power.
- Supercapacitor power.
- Engine shaft contribution.
- Battery shaft contribution.
- Operating mode.
- Architecture.
- Clutch state.
- Adaptive equivalence factor.

The equivalence factor increases when SoC is below the target and decreases when SoC is high:

```text
s = base_equivalence * phase_bias * fuzzy_soc_gain
```

Control logic by phase:

- Takeoff and climb: engine commanded near efficient/high power; supercapacitor handles transient peak demand; battery fills remaining deficit.
- Cruise: parallel mode is preferred when SoC is above the minimum reserve; battery assists if engine shaft power is insufficient.
- Loiter: parallel mode can run the engine and charge the battery if there is spare shaft power.
- Descent: engine off; bounded regenerative recovery charges battery and supercapacitor.
- Landing: propulsion can be idle.

This is why Mission 1 uses supercap support in takeoff/climb, strong battery assist in cruise, charge sustain during loiter, and regen during descent.

### 4.8 Time-stepped simulation loop

Implemented in `simulation/run_simulation.py`.

The simulator runs at `dt = 1 s`. For every timestep it:

1. Interpolates phase altitude.
2. Computes airspeed, ground speed, and rate of climb.
3. Updates current aircraft mass from cumulative fuel burn.
4. Computes aerodynamic force balance and required shaft power.
5. Applies special Mission 1 takeoff and landing logic.
6. Converts descent into bounded regenerative shaft power when enabled.
7. Calls the power split controller.
8. Steps the engine fuel-burn model.
9. Steps the battery SoC and C-rate model.
10. Computes delivered shaft power from engine, battery, and supercap.
11. Flags power deficits, stall-margin issues, and battery violations.
12. Records the complete time-series row.

The output rows include more than only power and SoC. They also include atmosphere, aerodynamic coefficients, thrust, drag, CL/CD, stall margin, mass, fuel remaining, BSFC, torque proxy, motor RPM proxy, ECMS factor, architecture, clutch state, and violation flags.

---

## 5. Repository Structure

```text
Aerothon/
|-- config.py                         # Central constants and assumptions
|-- requirements.txt                  # Python dependencies
|-- README.md                         # Project overview and run guide
|
|-- simulation/                       # Newer physics-based simulation stack
|   |-- atmosphere.py                 # ISA atmosphere
|   |-- aerodynamics.py               # Drag, thrust, power, stall checks
|   |-- battery_model.py              # Battery SoC/C-rate model
|   |-- engine_model.py               # Turboshaft BSFC/fuel model
|   |-- power_electronics.py          # Generator/bus/inverter/motor chain
|   |-- mission_profile.py            # Default mission and Mission 1 preset
|   |-- controller.py                 # Adaptive series-parallel controller
|   |-- run_simulation.py             # Main time-stepped simulator
|   `-- mission1_analysis.py          # Mission 1 plots and summaries
|
|-- sim/                              # Legacy / research simulation stack
|   |-- aircraft.py                   # Earlier atmosphere/aero model
|   |-- engine.py                     # Earlier turboshaft model
|   |-- battery.py                    # Earlier battery model
|   |-- motor.py                      # Electric motor model
|   |-- mission.py                    # Earlier mission builder
|   |-- hybrid_sim.py                 # Naive hybrid mission simulation
|   |-- ecms.py                       # Adaptive ECMS controller
|   |-- pmp.py                        # PMP benchmark
|   |-- clutch.py                     # Series-parallel clutch model
|   `-- supercap.py                   # Supercapacitor model
|
|-- sizing/                           # Mass and component sizing
|   |-- sizing_equations.py           # Iterative mass-consistent sizing loop
|   |-- phase2_calc.py
|   |-- phase3_sizing.py
|   `-- phase4_hybrid.py
|
|-- optimization/                     # Grid/parametric trade studies
|   `-- phase6_sweep.py               # Battery/engine/motor sweep
|
|-- optimize/                         # NSGA-II optimizer
|   `-- nsga2_runner.py               # Pymoo multi-objective optimizer
|
|-- dashboard/
|   |-- server.py                     # Flask dashboard and REST API
|   |-- index.html                    # Single-page dashboard UI
|   `-- app.py                        # Legacy Dash/Plotly dashboard
|
|-- validate/                         # Validation and comparison scripts
|   |-- baseline_compare.py
|   |-- full_validation.py
|   |-- compare_ecms_pmp.py
|   |-- calibrate_ecms.py
|   |-- sensitivity_analysis.py
|   |-- plot_sensitivity.py
|   |-- test_adaptive_ecms.py
|   `-- test_power_caps.py
|
|-- tests/                            # Pytest unit/integration tests
|   |-- test_aircraft.py
|   |-- test_clutch.py
|   |-- test_components.py
|   `-- test_mission.py
|
|-- docs/
|   |-- phase0_requirements.md        # Frozen requirements and units
|   |-- assumption_register.md        # Assumptions and source rationale
|   |-- methodology.md                # Physics/methodology notes
|   `-- requirements_traceability.md
|
|-- data/
|   |-- README.md
|   `-- outputs/                      # Generated CSV, JSON, and PNG outputs
|
`-- report/                           # Legacy reports, validation plots, Pareto data
```

Note: `__pycache__/` files are generated Python bytecode and are not part of the model definition.

---

## 6. Installation

Use Python 3.10+.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Dependencies:

- `numpy`
- `scipy`
- `matplotlib`
- `pandas`
- `pymoo`
- `dash`
- `plotly`
- `pytest`
- `flask`

---

## 7. How To Run

### Run the main Mission 1 simulation

```bash
python -m simulation.run_simulation
```

This writes:

- `data/outputs/mission_timeseries.csv`
- `data/outputs/mission_timeseries.json`

It also prints a validation report comparing key outputs with checkpoint values.

### Generate Mission 1 engineering plots and summaries

```bash
python -m simulation.mission1_analysis
```

This writes:

- `data/outputs/mission1_summary.json`
- `data/outputs/mission1_phase_summary.csv`
- `data/outputs/mission1_power_split.png`
- `data/outputs/mission1_aero_forces.png`
- `data/outputs/mission1_atmosphere_stall.png`
- `data/outputs/mission1_mass_range.png`
- `data/outputs/mission1_fuel_comparison.png`

### Start the Flask dashboard

```bash
python -m dashboard.server
```

Then open:

```text
http://127.0.0.1:5050
```

Useful Flask API endpoints:

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Dashboard UI |
| `/api/data/timeseries` | GET | Precomputed mission time-series |
| `/api/data/sweep` | GET | Parametric sweep results |
| `/api/data/optimised` | GET | Saved optimized configuration |
| `/api/simulate` | POST | Rerun mission with edited parameters |
| `/api/mission-optimize` | POST | Compact sweep around dashboard parameters |

### Run the legacy Dash dashboard

```bash
python -m dashboard.app
```

This dashboard reads legacy report data such as `report/pareto_front.json` and can rerun legacy design-vector simulations.

### Run the parametric sweep

```bash
python -m optimization.phase6_sweep
```

The sweep evaluates battery capacity, engine rating, and motor topology against:

- Fuel burn.
- Propulsion mass.
- Loiter endurance.
- MTOW feasibility.
- SoC feasibility.
- Battery C-rate feasibility.

Outputs are saved to `data/outputs/`, including `sweep_results.csv`, Pareto plots, and sensitivity plots.

### Run NSGA-II optimization

```bash
python -m optimize.nsga2_runner
```

Decision vector:

```text
[engine_kW, battery_kWh, supercap_kWh, n_motors, motor_kW_each, ecms_s]
```

Objectives:

- Maximize endurance.
- Minimize converged mass.

Constraints:

- Mass <= 1000 kg.
- Final SoC within the allowed operating window.
- No power-cap failure.

Outputs:

- `report/pareto_front.png`
- `report/pareto_front.json`

### Run tests

```bash
pytest
```

The tests cover component behavior, mission power profile logic, aircraft model behavior, and clutch logic.

---

## 8. Output Files

### Main simulation outputs

| File | Description |
|---|---|
| `data/outputs/mission_timeseries.csv` | Full per-second simulation table |
| `data/outputs/mission_timeseries.json` | Same time-series in JSON form |
| `data/outputs/mission1_summary.json` | Mission 1 headline summary |
| `data/outputs/mission1_phase_summary.csv` | Phase-level Mission 1 metrics |

### Mission 1 plot outputs

| File | Description |
|---|---|
| `mission1_power_split.png` | Engine, battery, supercap, regen, SoC, and fuel trends |
| `mission1_aero_forces.png` | Lift, drag, thrust, CL, CD, induced drag, dynamic pressure |
| `mission1_atmosphere_stall.png` | Altitude, airspeed, ground speed, density, stall margin |
| `mission1_mass_range.png` | Mass reduction from fuel burn and distance progression |
| `mission1_fuel_comparison.png` | Hybrid fuel burn vs conventional comparison |

### Optimization outputs

| File | Description |
|---|---|
| `data/outputs/sweep_results.csv` | Parametric sweep table |
| `data/outputs/pareto_fuel_vs_mass.png` | Fuel-vs-mass Pareto front |
| `data/outputs/sensitivity_battery.png` | Battery capacity sensitivity |
| `data/outputs/sensitivity_engine.png` | Engine rating sensitivity |
| `data/outputs/optimised_config.json` | Recommended optimized config |
| `report/pareto_front.json` | NSGA-II Pareto solution data |
| `report/pareto_front.png` | NSGA-II Pareto front plot |

---

## 9. Validation Logic

The project uses multiple validation layers:

### Built-in simulation checkpoints

`simulation/run_simulation.py` compares the time-stepped simulation against hand-calculated Phase 2/3/4 checkpoints:

| Checkpoint | Reference |
|---|---:|
| Cruise shaft power | 52.9 kW |
| Climb shaft power | 111.2 kW |
| Loiter shaft power | 48.7 kW |
| Total fuel | 17.6 kg |
| Final SoC | 20.9 % |
| Total shaft energy | 74.16 kWh |

The current Mission 1 preset intentionally differs from some default checkpoint assumptions, so use the printed validation report as a consistency warning system rather than a certification statement.

### Unit and integration tests

The `tests/` suite validates:

- Battery SoC integration against hand calculations.
- SoC minimum and maximum clamping.
- Battery and motor mass scaling.
- Motor electrical/mechanical power conversion.
- Mission phase power behavior.
- Descent near-zero/regen opportunity behavior.
- Clutch mode behavior.
- Aircraft model behavior.

### Validation scripts

The `validate/` folder contains:

- Baseline conventional comparison.
- Adaptive ECMS validation.
- ECMS vs PMP comparison.
- ECMS calibration.
- Sensitivity analysis.
- Full design validation and report export.

---

## 10. Important Model Assumptions

The main assumptions are recorded in `docs/assumption_register.md`. Important defaults include:

| Domain | Assumption |
|---|---|
| Atmosphere | ISA troposphere, 0-11 km |
| Fuel | Jet-A1, LHV = 43.02 MJ/kg |
| Aerodynamics | Parabolic drag polar, CD = CD0 + k CL^2 |
| Default wing | S = 8.0 m^2, AR = 10, e0 = 0.80, CD0 = 0.025 |
| Mission 1 wing | S = 25.0 m^2, AR = 15, e0 = 0.80, CD0 = 0.025 |
| Battery | NMC pack, 200 Wh/kg, 95% round-trip efficiency |
| Engine | 60 kW turboshaft with BSFC interpolation |
| Generator | 93% efficient |
| Inverter and rectifier | 98% each |
| Motor | 94% efficient, 5 kW/kg continuous |
| Supercap | Used for short-duration peak shaving |
| SoC reserve | Minimum 20% |
| Simulation step | 1 second |

For detailed source rationale and sensitivity tags, read:

- `docs/phase0_requirements.md`
- `docs/assumption_register.md`
- `docs/methodology.md`
- `docs/requirements_traceability.md`

---

## 11. Interpreting the Results

### Why the engine alone is not enough in some phases

The engine is rated at 60 kW, but cruise and takeoff shaft demand can exceed the shaft power available after drivetrain losses or during high-demand mission settings. The controller therefore uses:

- Battery assist for sustained deficits.
- Supercapacitor assist for sharp transients.
- Parallel clutching when the mechanical path is more efficient.

### Why loiter can recharge the battery

In Mission 1, loiter shaft demand is lower than the engine's commanded operating point. The controller uses the extra available power to charge the battery, moving SoC upward before descent and landing.

### Why descent fuel burn is zero

In descent, the engine is commanded off. The windmilling propeller is modeled as a bounded recovery source, so descent contributes a small amount of electrical recharge instead of fuel burn.

### Why stall margin matters

At every timestep, the simulator computes stall speed from current mass, density, wing area, and CL_max. If the commanded airspeed falls below the required safe margin, the row receives a stall-margin violation flag.

---

## 12. Current Development Status

The repository contains two simulation generations:

- `simulation/`: newer, structured, physics-rich stack used by Mission 1, the Flask dashboard, and current output files.
- `sim/`: older research stack used by legacy tests, sizing, PMP/ECMS comparison, NSGA-II optimization, and the Dash dashboard.

Both are useful. For new mission analysis, prefer `simulation.run_simulation` and `simulation.mission1_analysis`. For legacy optimizer and ECMS/PMP comparison work, use the `sim/`, `sizing/`, `validate/`, and `optimize/` paths.

---

## 13. License / Use

Academic and hackathon use for IIT Indore - HAL Aerothon work. Engineering assumptions are documented for transparency, but this repository is not a certified aircraft design tool.
