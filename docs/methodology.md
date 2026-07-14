# UAV Hybrid Powertrain Methodology

This document summarises the physics formulations and algorithmic assumptions utilised across the simulation, component sizing, and optimization modules of the UAV hybrid powertrain model.

## 1. Aerodynamics & Environment (`sim/aircraft.py`)

- **Standard Atmosphere**: The environment is modelled using the ISA Standard Atmosphere for the troposphere (0–11 km). The temperature lapse rate is $\lambda = -0.0065$ K/m. Air density $\rho$ is calculated using the barometric formula combined with the ideal gas law:
  $$ \rho(h) = \rho_0 \left( \frac{T_0 + \lambda h}{T_0} \right)^{\frac{g}{R|\lambda|} - 1} $$
- **Drag Polar**: The aerodynamic drag assumes a parabolic polar for a fixed-wing aircraft:
  $$ C_D = C_{D0} + k C_L^2 $$
  where $C_{D0}$ is the zero-lift drag coefficient and $k$ is the induced drag factor.
- **Level Flight Assumption**: For standard segments, lift is assumed to equal weight ($L = W$), yielding:
  $$ C_L = \frac{W}{\frac{1}{2}\rho V^2 S} $$
- **Shaft Power Requirement**: Thrust $T$ is balanced against drag $D$ and the climb weight component:
  $$ T = D + W \sin \gamma $$
  Required shaft power is determined by the propeller efficiency ($\eta_{prop}$):
  $$ P_{shaft} = \frac{T \cdot V}{\eta_{prop}} $$
- **Descent Regeneration**: During descent, regeneration is enabled by driving the motor as a generator. This induces extra drag on the airframe modeled as a windmilling drag coefficient increment ($\Delta C_{D, windmill} = 0.015$).

## 2. Propulsion Components

### Turboshaft Engine (`sim/engine.py`)
- **BSFC Model**: The turboshaft's Brake Specific Fuel Consumption (BSFC) is modeled via a lookup table, reflecting generic small turboshaft characteristics. Efficiency peaks around 70-80% of rated power, degrading significantly at idle and slightly at max power.
- **Fuel Flow**: Fuel flow $\dot{m}_{fuel}$ (kg/s) is linearly interpolated from the BSFC table based on the engine's instantaneous power fraction $P_{actual}/P_{rated}$. 

### Electric Storage (`sim/battery.py`, `sim/supercap.py`)
- **Li-Ion Battery**: Modeled using a Coulomb-counting State of Charge (SoC) tracker. 
  - **Specific Energy**: Assumed at 220 Wh/kg.
  - **Efficiency**: A round-trip efficiency of 95% is split symmetrically ($\eta_{chg} = \eta_{dis} = \sqrt{0.95}$).
  - **Limits**: Hard SoC bounds of [0.20, 0.90] are enforced alongside continuous C-rate limits (4C discharge, 2C charge).
- **Supercapacitor**: Incorporated to handle high-frequency power transients (> 2000 W/s), shielding the battery from degradation. 
  - **Parameters**: 5 Wh/kg specific energy, 4000 W/kg specific power, 97% round-trip efficiency.

### Mechanical Clutch (`sim/clutch.py`)
- **Fail-Safe Logic**: The series-parallel clutch is designed such that if the clutch actuator fails, it defaults to the **disengaged** state (Series mode). This ensures that the electric motor can still receive power from either the battery or the engine (via the generator) to maintain flight, avoiding a catastrophic single-point mechanical failure.

## 3. Power Management Algorithms

### Adaptive ECMS (`sim/ecms.py`)
- **Base Formulation**: The Equivalent Consumption Minimization Strategy (ECMS) determines the instantaneous optimal power split by minimizing the equivalent fuel cost:
  $$ J = \dot{m}_{fuel}(P_{eng}) + s \cdot \frac{P_{bat}}{Q_{LHV}} $$
  where $s$ is the equivalence factor and $Q_{LHV}$ is the fuel's lower heating value (43 MJ/kg).
- **Adaptive Feedback Rule**: To prevent SoC drift during unpredicted mission deviations (e.g., a loiter hold extended by 50%), the equivalence factor $s$ is continuously adjusted via proportional feedback against a planned reference SoC trajectory:
  $$ s_{adj} = s_{base} \left( 1 + K_{gain} \cdot (SoC_{planned} - SoC_{actual}) \right) $$

### Pontryagin's Minimum Principle (PMP) (`sim/pmp.py`)
- **Formulation**: Serves as a theoretical offline benchmark. It minimizes the Hamiltonian:
  $$ H = \dot{m}_{fuel}(P_{eng}) + \lambda \cdot \left( \frac{-P_{bat, internal}}{E_{batt}} \right) $$
- **Shooting Method**: A bisection search solves for the constant costate variable $\lambda$ that strictly enforces the charge-sustaining boundary condition ($SoC_{final} = SoC_{initial}$). PMP is unconstrained in its mode selection, freely picking between Series and Parallel at each timestep to find the absolute minimum possible fuel consumption.

## 4. Mass-Consistent Sizing Loop (`sizing/sizing_equations.py`)
- **Iterative Convergence**: Aircraft total mass ($M_{TOW}$) is not statically assumed. It is solved iteratively by running the full mission simulation.
- **Process**:
  1. Guess initial $M_{TOW}$.
  2. Compute mission power requirements.
  3. Run the hybrid powertrain simulation (ECMS + battery + engine + clutch).
  4. Size components based on required peak power and capacity.
  5. Sum component masses (airframe + payload + powertrain + fuel).
  6. Compare new mass to previous mass. If $\Delta M > 0.5$ kg, apply under-relaxation ($\alpha = 0.5$) and repeat.
- **Constraints**: Aborts if the required power exceeds installed power limits (e.g., initial climb power exceeds motor+engine rating).

## 5. Multi-Objective Optimization (`optimize/nsga2_runner.py`)
- **Algorithm**: Non-dominated Sorting Genetic Algorithm II (NSGA-II) via the `pymoo` framework.
- **Decision Variables**: 
  - Engine rated power (40–80 kW)
  - Battery capacity (5–40 kWh)
  - Supercapacitor mass (0.5–5 kg)
  - Motor count (1–4)
  - Motor rated power (10–40 kW each)
  - Base ECMS equivalence factor $s$ (0.5–2.0)
- **Objectives**: 
  1. **Maximize Endurance** (Minimize negative endurance)
  2. **Minimize Total Converged Mass**
- **Constraints**:
  - Converged mass $\leq$ 1000 kg (Strict MTOW limit).
  - Feasibility flag = True (Ensures power caps are strictly enforced and SoC stays within safe 0.20-0.90 bounds).
