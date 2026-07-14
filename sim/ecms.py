"""
Equivalent Consumption Minimisation Strategy (ECMS).

At each timestep the ECMS chooses the engine / motor power split
that minimises a composite cost function:

    J  =  m_dot_fuel(P_eng)  +  s * P_bat / Q_LHV        [kg/s]

where
    m_dot_fuel : actual fuel flow for engine shaft power P_eng
    s          : equivalence factor  (dimensionless, > 0)
    P_bat      : electrical power drawn from (+) or fed to (-) battery
    Q_LHV      : fuel lower heating value  [J/kg]  (43 MJ/kg for Jet-A)

Physically, the second term converts battery power into a "virtual
fuel flow", penalising discharge and rewarding charge.  The scalar s
is calibrated off-line so that the net SoC change over a full mission
is approximately zero (charge-sustaining).

Series-mode power balance
    P_req = (P_eng * eta_gen + P_bat) * eta_mot
    =>  P_bat = P_req / eta_mot  -  P_eng * eta_gen

The optimisation is 1-D over P_eng, bounded by:
    - Engine >= 0
    - Battery discharge <= C-rate and SoC-floor limits
    - Battery charge    <= C-rate and SoC-ceiling limits
"""

from scipy.optimize import minimize_scalar


class ECMS:
    """Single-parameter ECMS controller."""

    def __init__(self, s: float = 2.0, fuel_LHV_J_per_kg: float = 43.0e6):
        """
        Parameters
        ----------
        s                 : Equivalence factor (calibrate for charge-sustaining).
        fuel_LHV_J_per_kg : Lower heating value of fuel [J/kg].
        """
        self.s = s
        self.fuel_LHV = fuel_LHV_J_per_kg

    # ─────────────────────────────────────────────────────────
    def optimal_engine_power(self, p_req_W, bat, eng, mot, gen_eff, dt_s):
        """
        Return optimal engine shaft power [W] for series mode.

        Called only when clutch_mode == 'series' and p_req_W > 0.
        """
        eta_m = mot.efficiency
        eta_g = gen_eff

        # Total electrical power needed at the DC bus
        p_bus = p_req_W / eta_m

        # ── Battery power bounds ────────────────────────────
        dt_h = dt_s / 3600.0

        # Max discharge (positive P_bat)
        avail_soc = max(bat.soc - bat.soc_min, 0.0)
        if dt_h > 0 and avail_soc > 0:
            p_dis_soc = (avail_soc * bat.capacity_Wh
                         * bat._eta_discharge / dt_h)
        else:
            p_dis_soc = 0.0
        p_bat_max_dis = min(bat.max_discharge_W, p_dis_soc)

        # Max charge (positive magnitude)
        headroom = max(bat.soc_max - bat.soc, 0.0)
        if dt_h > 0 and headroom > 0:
            p_chg_soc = (headroom * bat.capacity_Wh
                         / (dt_h * bat._eta_charge))
        else:
            p_chg_soc = 0.0
        p_bat_max_chg = min(bat.max_charge_W, p_chg_soc)

        # ── Engine bounds from battery constraints ──────────
        # P_bat = p_bus - P_eng * eta_g
        # -p_bat_max_chg <= P_bat <= p_bat_max_dis
        p_eng_lo = max(0.0, (p_bus - p_bat_max_dis) / eta_g)
        p_eng_hi = max(p_eng_lo, (p_bus + p_bat_max_chg) / eta_g)

        # ── Engine physical bounds ──────────────────────────
        p_eng_hi = min(p_eng_hi, eng.rated_power_W)
        p_eng_lo = min(p_eng_lo, eng.rated_power_W)

        if p_eng_hi - p_eng_lo < 1.0:          # degenerate
            return p_eng_lo

        # ── Minimise ECMS cost ──────────────────────────────
        s_val = self.s
        Q = self.fuel_LHV

        def cost(p_eng):
            m_dot = eng.fuel_flow_kg_s(p_eng)
            p_bat = p_bus - p_eng * eta_g
            return m_dot + s_val * p_bat / Q

        res = minimize_scalar(
            cost,
            bounds=(p_eng_lo, p_eng_hi),
            method="bounded",
            options={"xatol": 10.0},            # 10 W tolerance
        )
        return float(res.x)

class AdaptiveECMS(ECMS):
    """
    Adaptive ECMS that adjusts the equivalence factor s based on SoC tracking.
    """
    def __init__(self, s_base: float, soc_planned, gain: float = 2.0, threshold: float = 0.05, fuel_LHV_J_per_kg: float = 43.0e6):
        super().__init__(s_base, fuel_LHV_J_per_kg)
        self.s_base = s_base
        self.soc_planned = soc_planned
        self.gain = gain
        self.threshold = threshold
        self.current_step = 0

    def sync_step(self, step_idx: int):
        self.current_step = step_idx

    def optimal_engine_power(self, p_req_W, bat, eng, mot, gen_eff, dt_s):
        # Determine planned SoC at this time step
        # Make sure not to exceed the planned array if the mission is extended!
        idx = min(self.current_step, len(self.soc_planned) - 1)
        soc_ref = self.soc_planned[idx]
        
        # Calculate deviation
        soc_deviation = soc_ref - bat.soc
        
        # Apply proportional feedback if threshold is exceeded
        if abs(soc_deviation) > self.threshold:
            self.s = self.s_base * (1.0 + self.gain * soc_deviation)
        else:
            self.s = self.s_base
            
        # Call base optimal_engine_power
        return super().optimal_engine_power(p_req_W, bat, eng, mot, gen_eff, dt_s)
