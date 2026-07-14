"""
Power Electronics Chain Model.

Models the efficiency losses through:
  Engine → Generator → Rectifier → DC Bus → Inverter → Motor → Shaft

Cross-references: GEN-01 through GEN-07, MOT-01 through MOT-07
"""

from config import GENERATOR, PE, MOTOR


class PowerElectronicsChain:
    """
    Lumped efficiency chain for the series hybrid-electric drivetrain.

    Engine shaft → Generator (AC PMSM) → Rectifier (SiC) →
    DC Bus → Inverter (SiC) → Motor (PMSM) → Propeller shaft

    All efficiencies are from the Phase 1 assumption register.
    """

    def __init__(self):
        self.eta_gen    = GENERATOR.efficiency          # 0.93
        self.eta_rect   = PE.rectifier_efficiency       # 0.98
        self.eta_bus    = PE.bus_efficiency             # 0.996
        self.eta_inv    = PE.inverter_efficiency        # 0.98
        self.eta_mot    = MOTOR.efficiency              # 0.94

        # Pre-computed compound efficiencies (Phase 1 EFF entries)
        self.eta_eng_to_dc   = self.eta_gen * self.eta_rect * self.eta_bus   # 0.907
        self.eta_dc_to_shaft = self.eta_inv * self.eta_mot                   # 0.921
        self.eta_eng_to_shaft = self.eta_eng_to_dc * self.eta_dc_to_shaft    # 0.836

        # Battery-to-shaft (bus → inverter → motor)
        self.eta_bat_to_shaft = self.eta_bus * self.eta_inv * self.eta_mot   # 0.917

    def engine_shaft_to_dc_bus(self, P_engine_W: float) -> float:
        """Engine mechanical power → DC bus electrical power [W]."""
        return P_engine_W * self.eta_eng_to_dc

    def dc_bus_to_prop_shaft(self, P_dc_W: float) -> float:
        """DC bus power → propeller shaft power [W]."""
        return P_dc_W * self.eta_dc_to_shaft

    def engine_to_prop_shaft(self, P_engine_W: float) -> float:
        """Engine mechanical power → propeller shaft power [W] (full chain)."""
        return P_engine_W * self.eta_eng_to_shaft

    def shaft_demand_to_dc_bus(self, P_shaft_W: float) -> float:
        """Required shaft power → required DC bus power [W]."""
        return P_shaft_W / self.eta_dc_to_shaft if self.eta_dc_to_shaft > 0 else 0.0

    def bat_to_prop_shaft(self, P_bat_elec_W: float) -> float:
        """Battery electrical power → propeller shaft power [W]."""
        return P_bat_elec_W * self.eta_bat_to_shaft

    def prop_shaft_to_bat(self, P_shaft_regen_W: float) -> float:
        """
        Regenerative shaft power → battery charge power [W].
        Uses reverse of motor→shaft chain, then battery charge efficiency.
        """
        return P_shaft_regen_W * self.eta_mot * self.eta_inv * self.eta_bus * 0.9747

    def system_efficiency(self, P_fuel_W: float, P_shaft_W: float) -> float:
        """Overall fuel→shaft efficiency at current operating point."""
        return (P_shaft_W / P_fuel_W) if P_fuel_W > 1.0 else 0.0

    def motor_torque_proxy(self, P_shaft_W: float, rpm: float = 4500.0) -> float:
        """
        Proxy torque at motor shaft: τ = P / ω  [N·m].
        Uses a default cruise RPM of 4500 if not provided.
        """
        omega = (rpm / 60.0) * 2.0 * 3.14159
        return (P_shaft_W / omega) if omega > 0 else 0.0
