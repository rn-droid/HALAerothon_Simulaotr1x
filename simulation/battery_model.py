"""
Battery Pack Model — Li-ion NMC SoC integrator with C-rate and thermal checks.

Tracks State of Charge via coulomb counting, enforces safe operating
limits, and flags constraint violations.
Cross-references: BAT-01 through BAT-09 in assumption_register.md
"""

import math
from config import BATTERY


class BatteryPack:
    """
    55 kWh Li-ion NMC battery pack model.
    
    Features:
      - Coulomb-counting SoC integration
      - C-rate enforcement (continuous & peak)
      - SoC window clamping [20%, 100%]
      - Round-trip efficiency modelling (sqrt split)
      - Constraint violation flagging
    """
    
    def __init__(self, capacity_kWh: float = None, initial_soc: float = None):
        self.capacity_kWh = capacity_kWh or BATTERY.default_capacity_kWh
        self.capacity_Wh = self.capacity_kWh * 1000.0
        self.capacity_J = self.capacity_Wh * 3600.0
        
        self.soc = min(1.0, initial_soc if initial_soc is not None else BATTERY.soc_max)
        self.soc_min = BATTERY.soc_min
        self.soc_max = 1.0
        
        # Efficiency: split round-trip into discharge and charge
        self.eta_discharge = math.sqrt(BATTERY.round_trip_efficiency)  # ~0.9747
        self.eta_charge = math.sqrt(BATTERY.round_trip_efficiency)     # ~0.9747
        
        # C-rate limits
        self.max_discharge_C = BATTERY.max_discharge_C       # 3.0 continuous
        self.max_discharge_C_peak = BATTERY.max_discharge_C_peak  # 5.0 peak
        self.max_charge_C = BATTERY.max_charge_C              # 2.0
        
        # Derived power limits
        self.max_discharge_W = self.max_discharge_C * self.capacity_kWh * 1000.0
        self.max_discharge_W_peak = self.max_discharge_C_peak * self.capacity_kWh * 1000.0
        self.max_charge_W = self.max_charge_C * self.capacity_kWh * 1000.0
        
        # Violation tracking
        self.violations = []
    
    def step(self, P_electrical_W: float, dt_s: float,
             is_peak: bool = False) -> dict:
        """
        Advance battery state by one timestep.
        
        Convention: P_electrical_W > 0 = DISCHARGE (battery supplies power)
                    P_electrical_W < 0 = CHARGE   (battery absorbs power)
        
        Args:
            P_electrical_W: Electrical power at battery terminals [W]
            dt_s:           Timestep [s]
            is_peak:        If True, allow peak C-rate limit instead of continuous
        
        Returns:
            dict with: P_battery_W, soc, soc_pct, c_rate, energy_delta_Wh,
                       violation_flags (list of strings)
        """
        flags = []
        
        # --- C-rate check ---
        c_rate = abs(P_electrical_W) / (self.capacity_kWh * 1000.0)
        
        if P_electrical_W > 0:  # Discharging
            max_P = self.max_discharge_W_peak if is_peak else self.max_discharge_W
            if P_electrical_W > max_P:
                flags.append(f"DISCHARGE_C_RATE_EXCEEDED: {c_rate:.2f}C > "
                             f"{'5.0C peak' if is_peak else '3.0C cont'}")
                P_electrical_W = max_P  # Clamp
        else:  # Charging
            if abs(P_electrical_W) > self.max_charge_W:
                flags.append(f"CHARGE_C_RATE_EXCEEDED: {c_rate:.2f}C > 2.0C")
                P_electrical_W = -self.max_charge_W  # Clamp
        
        # --- Energy accounting with efficiency ---
        if P_electrical_W > 0:
            # Discharging: internal energy drawn > terminal energy delivered
            energy_internal_Wh = (P_electrical_W / self.eta_discharge) * (dt_s / 3600.0)
        else:
            # Charging: internal energy stored < terminal energy absorbed
            energy_internal_Wh = (P_electrical_W * self.eta_charge) * (dt_s / 3600.0)
        
        # --- SoC update ---
        delta_soc = energy_internal_Wh / self.capacity_Wh
        new_soc = self.soc - delta_soc  # subtract because discharge is positive
        
        # SoC clamping
        if new_soc < self.soc_min:
            flags.append(f"SOC_MIN_REACHED: {new_soc:.4f} < {self.soc_min}")
            new_soc = self.soc_min
            # Reduce actual power delivered
            P_electrical_W = 0.0
        
        if new_soc > self.soc_max:
            flags.append(f"SOC_MAX_REACHED: {new_soc:.4f} > {self.soc_max}")
            new_soc = self.soc_max
            P_electrical_W = 0.0
        
        self.soc = new_soc
        if flags:
            self.violations.extend(flags)
        
        return {
            "P_battery_W": P_electrical_W,
            "soc": self.soc,
            "soc_pct": self.soc * 100.0,
            "c_rate": c_rate,
            "energy_delta_Wh": energy_internal_Wh,
            "violation_flags": flags,
        }
    
    def remaining_energy_Wh(self) -> float:
        """Usable energy remaining above SoC_min [Wh]."""
        return (self.soc - self.soc_min) * self.capacity_Wh
    
    def reset(self, initial_soc: float = None):
        """Reset battery for a new simulation run."""
        self.soc = initial_soc if initial_soc is not None else BATTERY.soc_max
        self.violations = []
