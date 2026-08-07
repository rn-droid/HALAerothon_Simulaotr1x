"""
Turboshaft Engine Model — SFC-based fuel consumption with part-load curve.

Models a 60 kW turboshaft with a realistic BSFC bowl curve interpolated
from the config lookup table.
Cross-references: ENG-01 through ENG-09 in assumption_register.md
"""

import numpy as np
from config import ENGINE


class TurboshaftEngine:
    """
    60 kW-class turboshaft engine model.
    
    Features:
      - BSFC lookup table with interpolation for part-load efficiency
      - Fuel burn integration
      - Load tracking and thermal proxy
    """
    
    def __init__(self, rated_power_W: float = None, bsfc_scale: float = 1.0):
        self.rated_power_W = rated_power_W or ENGINE.rated_power_W
        self.rated_power_kW = self.rated_power_W / 1000.0
        self.bsfc_scale = bsfc_scale
        
        # Build BSFC interpolation arrays
        self._pf = np.array(ENGINE.bsfc_table_pf, dtype=float)
        self._bsfc = np.array(ENGINE.bsfc_table_values, dtype=float)  # g/(kW*h)
        
        # State
        self.is_running = False
        self.current_power_W = 0.0
        self.cumulative_fuel_kg = 0.0
    
    def get_bsfc(self, power_fraction: float) -> float:
        """
        Interpolated BSFC [g/(kW*h)] at a given power fraction (0..1).
        Returns 0 if engine is off.
        """
        if power_fraction <= 0.0:
            return 0.0
        pf = np.clip(power_fraction, self._pf[0], self._pf[-1])
        return float(np.interp(pf, self._pf, self._bsfc)) * self.bsfc_scale
    
    def get_thermal_efficiency(self, power_fraction: float) -> float:
        """
        Thermal efficiency at a given load.
        eta_th = 3600 / (BSFC_g_per_kWh * LHV_kWh_per_kg)
        LHV Jet-A1 = 43.02 MJ/kg = 11.95 kWh/kg
        """
        bsfc = self.get_bsfc(power_fraction)
        if bsfc <= 0:
            return 0.0
        return 3600.0 / (bsfc * 11.95)
    
    def step(self, commanded_power_W: float, dt_s: float) -> dict:
        """
        Advance engine state by one timestep.
        
        Args:
            commanded_power_W: Mechanical shaft power requested [W]
            dt_s: Timestep [s]
        
        Returns:
            dict with: P_engine_W, power_fraction, bsfc, fuel_burn_kg,
                        thermal_efficiency, is_running
        """
        P = np.clip(commanded_power_W, 0.0, self.rated_power_W)
        self.current_power_W = P
        
        if P < 100.0:  # Below 100W = effectively off
            self.is_running = False
            return {
                "P_engine_W": 0.0,
                "power_fraction": 0.0,
                "bsfc_g_kWh": 0.0,
                "fuel_burn_kg": 0.0,
                "thermal_efficiency": 0.0,
                "is_running": False,
            }
        
        self.is_running = True
        pf = P / self.rated_power_W
        bsfc = self.get_bsfc(pf)
        
        # Fuel burn: BSFC [g/(kW*h)] * Power [kW] * dt [h]
        P_kW = P / 1000.0
        dt_h = dt_s / 3600.0
        fuel_kg = (bsfc / 1000.0) * P_kW * dt_h  # g->kg
        self.cumulative_fuel_kg += fuel_kg
        
        eta_th = self.get_thermal_efficiency(pf)
        
        return {
            "P_engine_W": P,
            "power_fraction": pf,
            "bsfc_g_kWh": bsfc,
            "fuel_burn_kg": fuel_kg,
            "thermal_efficiency": eta_th,
            "is_running": True,
        }
    
    def reset(self):
        """Reset engine state for a new simulation run."""
        self.is_running = False
        self.current_power_W = 0.0
        self.cumulative_fuel_kg = 0.0
