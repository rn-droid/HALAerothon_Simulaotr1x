"""
Central configuration — Single source of truth for all frozen constants.

Every module imports from here instead of defining local magic numbers.
Organised by domain; cross-referenced to the Assumption Register IDs in
docs/assumption_register.md.

Units: SI throughout (see docs/phase0_requirements.md §4).
"""

from dataclasses import dataclass, field
from typing import Dict

# ════════════════════════════════════════════════════════════════
# 1. PHYSICAL CONSTANTS
# ════════════════════════════════════════════════════════════════

G = 9.80665              # gravitational acceleration    [m/s²]
R_AIR = 287.058          # specific gas constant (air)   [J/(kg·K)]
GAMMA_AIR = 1.4          # specific heat ratio (air)     [—]


# ════════════════════════════════════════════════════════════════
# 2. ISA STANDARD ATMOSPHERE (Troposphere, 0–11 km)
# ════════════════════════════════════════════════════════════════

T0_ISA = 288.15          # sea-level temperature         [K]        ATM-01
P0_ISA = 101_325.0       # sea-level pressure            [Pa]       ATM-02
RHO0_ISA = 1.225         # sea-level density             [kg/m³]    ATM-03
LAPSE_RATE = -0.0065     # temperature lapse rate        [K/m]      ATM-04


# ════════════════════════════════════════════════════════════════
# 3. FUEL PROPERTIES (Jet-A1)
# ════════════════════════════════════════════════════════════════

FUEL_LHV = 43.02e6       # lower heating value           [J/kg]     ENG-04
FUEL_DENSITY = 800.0     # density                       [kg/m³]    ENG-05


# ════════════════════════════════════════════════════════════════
# 4. AIRCRAFT-LEVEL REQUIREMENTS (FROZEN)
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class AircraftRequirements:
    """Immutable aircraft-level design constraints."""
    mtow_kg: float = 1000.0                 # R-02
    payload_kg: float = 200.0               # R-03
    cruise_speed_ms: float = 69.44          # R-04  (250 km/h)
    cruise_alt_m: float = 5000.0            # R-05  design point
    cruise_alt_min_m: float = 3000.0        # R-05  lower bound
    cruise_alt_max_m: float = 10000.0       # R-05  upper bound
    engine_rated_power_W: float = 60_000.0  # R-06


AIRCRAFT = AircraftRequirements()


# ════════════════════════════════════════════════════════════════
# 5. AIRFRAME AERODYNAMICS
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class AeroConfig:
    """Aerodynamic parameters (assumption-register IDs in comments)."""
    S_ref: float = 8.0                # wing reference area       [m²]    AERO-01
    AR: float = 10.0                  # aspect ratio              [—]     AERO-02
    e0: float = 0.80                  # Oswald efficiency         [—]     AERO-03
    CD0: float = 0.025                # zero-lift drag coeff      [—]     AERO-04
    windmilling_CD: float = 0.015     # descent regen drag inc.   [—]     AERO-09

    @property
    def k(self) -> float:
        """Induced drag factor k = 1 / (π · AR · e₀)."""  # AERO-05
        import math
        return 1.0 / (math.pi * self.AR * self.e0)

    @property
    def prop_efficiency(self) -> Dict[str, float]:
        """Propeller efficiency per flight regime."""
        return {
            "cruise": 0.82,           # AERO-06
            "climb":  0.72,           # AERO-07
            "loiter": 0.65,           # AERO-08
        }


AERO = AeroConfig()


# ════════════════════════════════════════════════════════════════
# 6. TURBOSHAFT ENGINE
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class EngineConfig:
    """Turboshaft engine parameters."""
    rated_power_W: float = 60_000.0         # ENG-01
    # BSFC lookup table: (power_fraction, BSFC [g/(kW·h)])
    bsfc_table_pf: tuple = (0.20, 0.30, 0.40, 0.50,
                            0.60, 0.70, 0.80, 0.90, 1.00)
    bsfc_table_values: tuple = (620, 520, 460, 420,
                                395, 380, 375, 380, 395)   # ENG-02, ENG-03
    specific_power_kW_per_kg: float = 2.0   # mass sizing


ENGINE = EngineConfig()


# ════════════════════════════════════════════════════════════════
# 7. GENERATOR & POWER ELECTRONICS
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class GeneratorConfig:
    """Generator (+ rectifier) parameters."""
    efficiency: float = 0.93                # GEN-01
    specific_power_kW_per_kg: float = 5.0   # same class as motor


GENERATOR = GeneratorConfig()

@dataclass(frozen=True)
class PowerElectronicsConfig:
    """Power-electronics parameters (Phase 1 separation)."""
    rectifier_efficiency: float = 0.98   # GEN-04 — SiC MOSFET rectifier
    inverter_efficiency: float = 0.98    # GEN-05 — SiC MOSFET inverter
    bus_efficiency: float = 0.996        # GEN-06 — DC bus / cabling


PE = PowerElectronicsConfig()


# ════════════════════════════════════════════════════════════════
# 8. BATTERY PACK
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class BatteryConfig:
    """Li-ion NMC battery pack parameters (Phase 1/3 values)."""
    default_capacity_kWh: float = 55.0        # Phase 3 sized pack
    specific_energy_Wh_per_kg: float = 200.0  # BAT-02 pack-level
    round_trip_efficiency: float = 0.95       # BAT-03
    max_discharge_C: float = 3.0              # BAT-04 continuous
    max_discharge_C_peak: float = 5.0         # BAT-05 peak (30s)
    max_charge_C: float = 2.0                 # BAT-06
    soc_min: float = 0.20                     # BAT-07
    soc_max: float = 0.90                     # BAT-07
    nominal_voltage_V: float = 400.0          # BAT-08


BATTERY = BatteryConfig()


# ════════════════════════════════════════════════════════════════
# 9. ELECTRIC MOTOR(S)
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class MotorConfig:
    """PM electric motor parameters (Phase 3 twin-motor sizing)."""
    total_rated_power_W: float = 120_000.0     # 2 x 60 kW (Phase 3)
    num_motors: int = 2                         # Twin for redundancy
    efficiency: float = 0.94                    # MOT-01
    specific_power_kW_per_kg: float = 5.0       # MOT-02 continuous
    max_rpm: int = 6000                         # MOT-06


MOTOR = MotorConfig()


# ════════════════════════════════════════════════════════════════
# 10. MISSION PROFILE DEFAULTS
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class MissionDefaults:
    """Default mission segment parameters (user-adjustable via dashboard)."""
    takeoff_transition_alt_m: float = 50.0      # MIS-01
    climb_rate_ms: float = 5.0                  # MIS-02
    descent_rate_ms: float = 3.0                # MIS-03
    cruise_duration_s: int = 1800               # MIS-04  (30 min)
    loiter_duration_s: int = 1200               # MIS-05  (20 min)
    regen_recovery_efficiency: float = 0.50     # MIS-06


MISSION = MissionDefaults()


# ════════════════════════════════════════════════════════════════
# 11. MASS BUDGET FIXED ITEMS
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class MassBudget:
    """Fixed mass items (not sized by the optimiser)."""
    payload_kg: float = 200.0                # MASS-01
    airframe_kg: float = 300.0               # MASS-02
    mtow_limit_kg: float = 1000.0            # MASS-03


MASS = MassBudget()


# ════════════════════════════════════════════════════════════════
# 12. SIMULATION PARAMETERS
# ════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class SimulationParams:
    """Numerical simulation settings."""
    dt_s: float = 1.0                   # time step [s]
    transient_threshold_W_per_s: float = 2000.0  # supercap routing threshold


SIM = SimulationParams()


# ════════════════════════════════════════════════════════════════
# 13. UNIT CONVERSION HELPERS
# ════════════════════════════════════════════════════════════════

def ms_to_kmh(v: float) -> float:
    """m/s → km/h."""
    return v * 3.6

def kmh_to_ms(v: float) -> float:
    """km/h → m/s."""
    return v / 3.6

def m_to_ft(h: float) -> float:
    """metres → feet."""
    return h * 3.28084

def ft_to_m(h: float) -> float:
    """feet → metres."""
    return h / 3.28084

def W_to_kW(p: float) -> float:
    """watts → kilowatts."""
    return p / 1000.0

def W_to_hp(p: float) -> float:
    """watts → horsepower."""
    return p / 745.7

def J_to_Wh(e: float) -> float:
    """joules → watt-hours."""
    return e / 3600.0

def J_to_kWh(e: float) -> float:
    """joules → kilowatt-hours."""
    return e / 3.6e6
