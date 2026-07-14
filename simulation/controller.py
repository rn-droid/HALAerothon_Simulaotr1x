"""
Power-Split Controller — Phase 4 Strategy A State Machine.

Implements the "Peak-Shaving + EV Loiter" hybridization strategy:
  - Engine runs at max rated power during TAKEOFF, CLIMB, CRUISE
  - Engine OFF during LOITER, DESCENT, LANDING (pure EV)
  - Battery covers all deficit above engine output
  - SoC safety limits enforced as hard constraints

Cross-references: Phase 4 control logic table, EFF-01 through EFF-04
"""

from config import ENGINE, BATTERY


class PowerSplitController:
    """
    Rule-based power-split controller (Strategy A: Peak-Shaving + EV Loiter).

    At each timestep it receives the shaft power demand and current SoC,
    and returns commanded engine power and battery power.
    """

    ENGINE_ON_PHASES  = {"TAKEOFF", "CLIMB", "CRUISE"}
    ENGINE_OFF_PHASES = {"LOITER", "DESCENT", "LANDING"}

    def __init__(self,
                 engine_rated_W: float = None,
                 bat_capacity_kWh: float = None):
        self.engine_rated_W   = engine_rated_W  or ENGINE.rated_power_W
        self.bat_capacity_kWh = bat_capacity_kWh or BATTERY.default_capacity_kWh
        self.soc_min          = BATTERY.soc_min
        self.soc_max          = BATTERY.soc_max

    def compute_split(
        self,
        phase_name: str,
        P_shaft_demand_W: float,
        eta_eng_to_shaft: float,
        eta_bat_to_shaft: float,
        current_soc: float,
    ) -> dict:
        """
        Compute commanded engine and battery powers for one timestep.

        Args:
            phase_name:         Current mission phase (e.g. "CLIMB")
            P_shaft_demand_W:   Shaft power needed at propeller [W]
            eta_eng_to_shaft:   Engine→shaft efficiency (product of chain)
            eta_bat_to_shaft:   Battery→shaft efficiency
            current_soc:        Current battery SoC [0..1]

        Returns:
            dict with:
              P_engine_cmd_W    — commanded engine mechanical power [W]
              P_bat_elec_W      — battery electrical power (+discharge/-charge) [W]
              P_engine_shaft_W  — actual shaft contribution from engine [W]
              P_bat_shaft_W     — actual shaft contribution from battery [W]
              mode              — operating mode string
        """
        # ── Engine decision ──────────────────────────────────────────
        if phase_name in self.ENGINE_ON_PHASES:
            P_eng_cmd = self.engine_rated_W
        else:
            P_eng_cmd = 0.0

        P_eng_shaft = P_eng_cmd * eta_eng_to_shaft  # shaft power from engine

        # ── Battery covers deficit (or captures surplus during regen) ─
        P_shaft_deficit = P_shaft_demand_W - P_eng_shaft

        if P_shaft_deficit > 0.0:
            # Battery discharges to cover deficit
            P_bat_elec = P_shaft_deficit / eta_bat_to_shaft
            mode = "ENGINE+BAT_BOOST" if P_eng_cmd > 0 else "PURE_ELECTRIC"
        elif P_shaft_deficit < 0.0 and phase_name == "DESCENT":
            # Regenerative braking: windmilling motor charges battery
            P_bat_elec = P_shaft_deficit * eta_bat_to_shaft  # negative = charge
            mode = "REGEN"
        else:
            # Engine provides more than enough (cruise with engine at max)
            # Slight surplus recharges battery if SoC allows
            if current_soc < self.soc_max and P_shaft_deficit < 0.0:
                P_bat_elec = P_shaft_deficit * eta_bat_to_shaft * 0.95
                mode = "ENGINE_RECHARGE"
            else:
                P_bat_elec = 0.0
                mode = "ENGINE_ONLY"

        # SoC floor — if battery is depleted, cannot discharge
        if current_soc <= self.soc_min and P_bat_elec > 0.0:
            P_bat_elec = 0.0
            mode = "BATTERY_DEPLETED"

        P_bat_shaft = P_bat_elec * eta_bat_to_shaft if P_bat_elec > 0 else 0.0

        return {
            "P_engine_cmd_W":   P_eng_cmd,
            "P_bat_elec_W":     P_bat_elec,
            "P_engine_shaft_W": P_eng_shaft,
            "P_bat_shaft_W":    P_bat_shaft,
            "mode":             mode,
        }
