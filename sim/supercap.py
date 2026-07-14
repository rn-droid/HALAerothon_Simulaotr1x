"""
Supercapacitor model for handling high-frequency power transients.
"""

import math

class Supercapacitor:
    def __init__(
        self,
        mass_kg: float = 5.0,
        specific_energy_Wh_per_kg: float = 5.0,
        specific_power_W_per_kg: float = 4000.0,
        round_trip_efficiency: float = 0.97,
        soc_min: float = 0.05,
        soc_max: float = 0.95,
    ):
        self.mass_kg = mass_kg
        self.capacity_Wh = mass_kg * specific_energy_Wh_per_kg
        self.max_power_W = mass_kg * specific_power_W_per_kg
        
        self.soc_min = soc_min
        self.soc_max = soc_max
        self.soc = self.soc_max
        
        self._eta_discharge = math.sqrt(round_trip_efficiency)
        self._eta_charge = math.sqrt(round_trip_efficiency)

    def step(self, power_W: float, dt_s: float) -> float:
        """
        Advance one timestep.
        power_W > 0: discharge to bus
        power_W < 0: charge from bus
        """
        dt_h = dt_s / 3600.0
        
        if power_W > 0:
            power_W = min(power_W, self.max_power_W)
            energy_drain_Wh = power_W * dt_h / self._eta_discharge
            delta_soc = energy_drain_Wh / self.capacity_Wh
            
            available_soc = self.soc - self.soc_min
            if delta_soc > available_soc:
                delta_soc = max(available_soc, 0.0)
                energy_drain_Wh = delta_soc * self.capacity_Wh
                power_W = energy_drain_Wh * self._eta_discharge / dt_h if dt_h > 0 else 0.0
                
            self.soc -= delta_soc
            return power_W
            
        elif power_W < 0:
            charge_power = min(-power_W, self.max_power_W)
            energy_stored_Wh = charge_power * dt_h * self._eta_charge
            delta_soc = energy_stored_Wh / self.capacity_Wh
            
            headroom_soc = self.soc_max - self.soc
            if delta_soc > headroom_soc:
                delta_soc = max(headroom_soc, 0.0)
                energy_stored_Wh = delta_soc * self.capacity_Wh
                charge_power = energy_stored_Wh / (dt_h * self._eta_charge) if dt_h > 0 else 0.0
                
            self.soc += delta_soc
            return -charge_power
            
        return 0.0
