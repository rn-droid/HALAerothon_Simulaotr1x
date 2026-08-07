"""
Adaptive series-parallel power-split controller for the UAV dashboard.

Architectures modelled:
  Series: engine -> generator -> DC bus -> motor -> propeller.
  Parallel: engine and motor both couple mechanically to the propeller shaft.
  Series-parallel: clutch selects the efficient path for the current phase.

The controller combines fuzzy ECMS-style SoC feedback with peak shaving:
the supercapacitor handles short high-power deficits, the battery handles
continuous electric power, and descent uses the windmilling propeller as a
generator when recoverable shaft power is available.
"""

from config import BATTERY, ENGINE


class PowerSplitController:
    """Series-parallel hybrid controller with adaptive ECMS and peak shaving."""

    def __init__(
        self,
        engine_rated_W: float = None,
        bat_capacity_kWh: float = None,
        supercap_energy_kWh: float = 1.5,
        motor_rated_W: float = 120_000.0,
    ):
        self.engine_rated_W = engine_rated_W or ENGINE.rated_power_W
        self.bat_capacity_kWh = bat_capacity_kWh or BATTERY.default_capacity_kWh
        self.supercap_energy_kWh = max(0.05, supercap_energy_kWh)
        self.motor_rated_W = motor_rated_W
        self.soc_min = BATTERY.soc_min
        self.soc_max = 1.0
        self.soc_target = 0.62
        self.base_equivalence = 2.4
        self.eta_parallel = 0.96
        self.supercap_max_W = min(80_000.0, self.supercap_energy_kWh * 1000.0 * 320.0)
        self.prev_demand_W = 0.0

    def _bsfc(self, power_fraction: float) -> float:
        if power_fraction <= 0.0:
            return 0.0
        table_pf = ENGINE.bsfc_table_pf
        table_bsfc = ENGINE.bsfc_table_values
        pf = max(table_pf[0], min(table_pf[-1], power_fraction))
        for idx in range(1, len(table_pf)):
            if pf <= table_pf[idx]:
                lo_pf, hi_pf = table_pf[idx - 1], table_pf[idx]
                lo_bsfc, hi_bsfc = table_bsfc[idx - 1], table_bsfc[idx]
                frac = (pf - lo_pf) / max(hi_pf - lo_pf, 1e-9)
                return lo_bsfc + frac * (hi_bsfc - lo_bsfc)
        return table_bsfc[-1]

    def _equivalence_factor(self, current_soc: float, phase_name: str) -> float:
        phase_bias = {
            "TAKEOFF": 0.85,
            "CLIMB": 1.05,
            "CRUISE": 1.20,
            "LOITER": 1.45,
            "DESCENT": 0.70,
            "LANDING": 1.10,
        }.get(phase_name, 1.0)
        soc_error = self.soc_target - current_soc
        if soc_error > 0.12:
            fuzzy_gain = 1.75
        elif soc_error > 0.04:
            fuzzy_gain = 1.30
        elif soc_error < -0.18:
            fuzzy_gain = 0.62
        elif soc_error < -0.08:
            fuzzy_gain = 0.82
        else:
            fuzzy_gain = 1.0
        return max(0.6, self.base_equivalence * phase_bias * fuzzy_gain)

    def _series_engine_command(
        self,
        phase_name: str,
        P_shaft_demand_W: float,
        eta_eng_to_shaft: float,
        current_soc: float,
        ecms_s: float,
    ) -> float:
        efficient_engine_W = 0.72 * self.engine_rated_W
        demand_engine_W = P_shaft_demand_W / max(eta_eng_to_shaft, 1e-6)

        if phase_name in {"TAKEOFF", "CLIMB"}:
            return min(self.engine_rated_W, max(efficient_engine_W, demand_engine_W * 0.72))
        if phase_name == "LOITER":
            if current_soc > 0.45:
                return 0.0
            return min(self.engine_rated_W, max(0.38 * self.engine_rated_W, demand_engine_W * 0.55))
        if current_soc < 0.50:
            return min(self.engine_rated_W, max(efficient_engine_W, demand_engine_W * 1.08))
        if current_soc > 0.82 and ecms_s < self.base_equivalence:
            return min(self.engine_rated_W, demand_engine_W * 0.65)
        return min(self.engine_rated_W, max(0.45 * self.engine_rated_W, demand_engine_W * 0.92))

    def compute_split(
        self,
        phase_name: str,
        P_shaft_demand_W: float,
        eta_eng_to_shaft: float,
        eta_bat_to_shaft: float,
        current_soc: float,
    ) -> dict:
        """Return engine, battery, supercap, and clutch commands."""
        ecms_s = self._equivalence_factor(current_soc, phase_name)
        transient_W = max(0.0, P_shaft_demand_W - self.prev_demand_W)
        self.prev_demand_W = P_shaft_demand_W

        if phase_name == "LANDING" and P_shaft_demand_W <= 100.0:
            return {
                "P_engine_cmd_W": 0.0,
                "P_bat_elec_W": 0.0,
                "P_supercap_W": 0.0,
                "P_engine_shaft_W": 0.0,
                "P_bat_shaft_W": 0.0,
                "mode": "LANDING_IDLE",
                "architecture": "IDLE",
                "clutch_state": "OPEN",
                "ecms_s": ecms_s,
                "soc_target": self.soc_target,
            }

        if P_shaft_demand_W < 0.0 or phase_name == "DESCENT":
            regen_shaft_W = abs(min(P_shaft_demand_W, 0.0))
            charge_W = -regen_shaft_W * eta_bat_to_shaft * 0.55 if current_soc < 0.995 else 0.0
            return {
                "P_engine_cmd_W": 0.0,
                "P_bat_elec_W": charge_W,
                "P_supercap_W": min(regen_shaft_W * 0.18, self.supercap_max_W),
                "P_engine_shaft_W": 0.0,
                "P_bat_shaft_W": 0.0,
                "mode": "REGEN_DESCENT",
                "architecture": "SERIES_REGEN",
                "clutch_state": "OPEN",
                "ecms_s": ecms_s,
                "soc_target": self.soc_target,
            }

        use_parallel = phase_name in {"CRUISE", "LOITER"} and current_soc > self.soc_min + 0.02

        if use_parallel:
            P_eng_cmd = self.engine_rated_W if phase_name == "LOITER" else min(self.engine_rated_W, P_shaft_demand_W / self.eta_parallel)
            P_eng_shaft = P_eng_cmd * self.eta_parallel
            P_deficit = P_shaft_demand_W - P_eng_shaft
            if P_deficit > 0.0:
                P_bat_elec = min(P_deficit / eta_bat_to_shaft, self.motor_rated_W / max(eta_bat_to_shaft, 1e-6))
                mode = "PARALLEL_ASSIST"
            elif phase_name == "LOITER" and current_soc < 0.90:
                P_bat_elec = P_deficit * eta_bat_to_shaft
                mode = "PARALLEL_CHARGE_LOITER"
            else:
                P_bat_elec = 0.0
                mode = "PARALLEL_CRUISE"
            return {
                "P_engine_cmd_W": P_eng_cmd,
                "P_bat_elec_W": P_bat_elec,
                "P_supercap_W": 0.0,
                "P_engine_shaft_W": P_eng_shaft,
                "P_bat_shaft_W": max(P_bat_elec, 0.0) * eta_bat_to_shaft,
                "mode": mode,
                "architecture": "PARALLEL",
                "clutch_state": "LOCKED",
                "ecms_s": ecms_s,
                "soc_target": self.soc_target,
            }

        P_eng_cmd = self._series_engine_command(
            phase_name, P_shaft_demand_W, eta_eng_to_shaft, current_soc, ecms_s
        )
        P_eng_shaft = P_eng_cmd * eta_eng_to_shaft
        P_deficit = P_shaft_demand_W - P_eng_shaft

        P_supercap_W = 0.0
        if P_deficit > 0.0 and phase_name in {"TAKEOFF", "CLIMB"}:
            peak_need_W = max(P_deficit * 0.42, transient_W * 0.55)
            P_supercap_W = min(self.supercap_max_W, max(0.0, peak_need_W))
            P_deficit -= P_supercap_W

        if P_deficit > 0.0:
            P_bat_elec = P_deficit / eta_bat_to_shaft
        elif P_deficit < 0.0 and current_soc < 0.96:
            P_bat_elec = P_deficit * eta_bat_to_shaft * min(0.85, ecms_s / 3.5)
        else:
            P_bat_elec = 0.0

        if current_soc <= self.soc_min and P_bat_elec > 0.0:
            P_bat_elec = 0.0
            mode = "BATTERY_PROTECT"
        elif phase_name in {"TAKEOFF", "CLIMB"}:
            mode = "SERIES_PARALLEL_PEAK"
        elif P_eng_cmd <= 0.0:
            mode = "EV_LOITER"
        elif P_bat_elec < -100.0:
            mode = "SERIES_CHARGE_SUSTAIN"
        else:
            mode = "SERIES_ECMS"

        return {
            "P_engine_cmd_W": P_eng_cmd,
            "P_bat_elec_W": P_bat_elec,
            "P_supercap_W": P_supercap_W,
            "P_engine_shaft_W": P_eng_shaft,
            "P_bat_shaft_W": max(P_bat_elec, 0.0) * eta_bat_to_shaft,
            "mode": mode,
            "architecture": "SERIES_PARALLEL" if phase_name in {"TAKEOFF", "CLIMB"} else "SERIES",
            "clutch_state": "MODULATING" if phase_name in {"TAKEOFF", "CLIMB"} else "OPEN",
            "ecms_s": ecms_s,
            "soc_target": self.soc_target,
        }
