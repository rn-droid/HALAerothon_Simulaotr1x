"""
Mission profile builder for a hybrid-electric UAV.

Constructs a 1-second-timestep time-series of required shaft power
across five mission phases:

  Takeoff  ->  Climb  ->  Cruise  ->  Loiter  ->  Descent

Physics decisions documented inline.
"""

import math
import pandas as pd
from sim.aircraft import Aircraft


# ── Map mission phase labels to Aircraft.prop_eff keys ──────
# The Aircraft class defines propeller efficiencies for three
# regimes: 'cruise', 'climb', 'loiter'.  We map the two extra
# mission phases to the closest aerodynamic regime:
#   takeoff ≈ climb  (steep ascent, moderate speed)
#   descent ≈ loiter (low speed, shallow glide)
_PROP_PHASE = {
    "takeoff": "climb",
    "climb":   "climb",
    "cruise":  "cruise",
    "loiter":  "loiter",
    "descent": "loiter",
}


class MissionProfile:
    """Builds a full-mission power-required time-series (1 s steps)."""

    def __init__(
        self,
        aircraft: Aircraft | None = None,
        cruise_alt_m: float = 3000.0,
        cruise_duration_s: int = 1800,
        loiter_duration_s: int = 1200,
        cruise_speed_kmh: float = 250.0,
        climb_rate_ms: float = 5.0,
        descent_rate_ms: float = 3.0,
    ):
        """
        Parameters
        ----------
        aircraft          : Aircraft model (defaults to standard 1000 kg UAV).
        cruise_alt_m      : Target cruise altitude [m].
        cruise_duration_s : Time spent cruising [s].
        loiter_duration_s : Time spent loitering [s].
        cruise_speed_kmh  : Cruise true airspeed [km/h].
        climb_rate_ms     : Constant rate-of-climb [m/s].
        descent_rate_ms   : Constant sink rate during descent [m/s].
        """
        self.ac = aircraft or Aircraft()
        self.cruise_alt = cruise_alt_m
        self.cruise_dur = cruise_duration_s
        self.loiter_dur = loiter_duration_s
        self.V_cruise = cruise_speed_kmh / 3.6          # convert to m/s
        self.climb_rate = climb_rate_ms
        self.descent_rate = descent_rate_ms
        self.W = self.ac.mtow_kg * self.ac.G            # weight [N]
        self.takeoff_transition_alt = 50.0               # end of takeoff [m]

    # ── Helper: best-L/D speed ──────────────────────────────
    def best_LD_speed(self, altitude_m: float, phase: str = "cruise") -> float:
        """
        Speed for maximum lift-to-drag ratio at a given altitude.

        Derivation
        ----------
        Minimise  CD / CL  =  (CD0 + k CL^2) / CL
        d/dCL  =>  -CD0/CL^2 + k = 0
                    CL_opt = sqrt(CD0 / k)

        At CL_opt, CD_opt = 2 CD0  and  (L/D)_max = CL_opt / CD_opt.
        The corresponding airspeed (from L = W = 0.5 rho V^2 S CL):

            V_opt = sqrt( 2 W / (rho S CL_opt) )
        """
        cd0 = self.ac.CD0
        if phase == "descent":
            cd0 += getattr(self.ac, "windmilling_drag_coefficient", 0.0)
            
        CL_opt = math.sqrt(cd0 / self.ac.k)
        rho = self.ac.air_density(max(altitude_m, 0.0))
        return math.sqrt(2.0 * self.W / (rho * self.ac.S * CL_opt))

    # ── Helper: build one data row ──────────────────────────
    def _row(self, t: int, phase: str, alt: float, vel: float, gamma: float) -> dict:
        """Compute power and return a record dict for one timestep."""
        P = self.ac.shaft_power_required(
            self.W, vel, max(alt, 0.0), gamma, phase
        )
        return {
            "time_s":           t,
            "phase":            phase,
            "altitude_m":       alt,
            "velocity_ms":      vel,
            "power_required_W": P,
            "climb_angle_rad":  gamma,
        }

    # ── Main builder ────────────────────────────────────────
    def build(self) -> pd.DataFrame:
        """
        Construct the mission time-series.

        Returns
        -------
        pd.DataFrame
            Columns: time_s, phase, altitude_m, velocity_ms,
                     power_required_W, climb_angle_rad
        """
        rows: list[dict] = []
        t = 0

        # ────────────────────────────────────────────────────
        # PHASE 1 — TAKEOFF  (ground -> 50 m)
        # ────────────────────────────────────────────────────
        # Model: speed ramps linearly from V_rotate to V_cruise
        # while altitude ramps linearly to 50 m.
        #
        # V_rotate ~ 1.1 x V_stall.  With CL_max ~ 1.5:
        #   V_stall = sqrt(2W / (rho0 S CL_max))
        #           = sqrt(2 x 9807 / (1.225 x 10 x 1.5))
        #           ~ 32.7 m/s
        #   V_rotate ~ 36 m/s
        #
        # At low speed, CL is high -> high induced drag ->
        # high power demand.  This naturally captures the
        # elevated takeoff power without needing a separate
        # thrust/acceleration model.
        V_rotate = 36.0                                  # m/s
        takeoff_roc = 2.5                                # m/s vertical
        takeoff_dur = int(self.takeoff_transition_alt / takeoff_roc)  # 20 s

        for i in range(takeoff_dur):
            frac = i / takeoff_dur
            alt = frac * self.takeoff_transition_alt
            vel = V_rotate + frac * (self.V_cruise - V_rotate)
            gamma = math.asin(takeoff_roc / vel)
            rows.append(self._row(t, "takeoff", alt, vel, gamma))
            t += 1

        # ────────────────────────────────────────────────────
        # PHASE 2 — CLIMB  (50 m -> cruise altitude)
        # ────────────────────────────────────────────────────
        # Constant TAS = V_cruise, constant ROC = climb_rate.
        #   gamma = arcsin(ROC / V_cruise)
        #   duration = (cruise_alt - 50) / ROC
        #
        # Power = (D + W sin gamma) V / eta_climb
        # The W sin gamma term dominates — this is the most
        # power-demanding sustained phase.
        climb_range = self.cruise_alt - self.takeoff_transition_alt
        climb_dur = int(climb_range / self.climb_rate)
        gamma_climb = math.asin(self.climb_rate / self.V_cruise)

        for i in range(climb_dur):
            alt = self.takeoff_transition_alt + i * self.climb_rate
            rows.append(self._row(t, "climb", alt, self.V_cruise, gamma_climb))
            t += 1

        # ────────────────────────────────────────────────────
        # PHASE 3 — CRUISE  (level flight at V_cruise)
        # ────────────────────────────────────────────────────
        # gamma = 0, constant altitude.
        # Power = D V / eta_cruise
        for _ in range(self.cruise_dur):
            rows.append(self._row(t, "cruise", self.cruise_alt, self.V_cruise, 0.0))
            t += 1

        # ────────────────────────────────────────────────────
        # PHASE 4 — LOITER  (level flight at best-L/D speed)
        # ────────────────────────────────────────────────────
        # Flying at CL_opt = sqrt(CD0/k) minimises drag for
        # a given weight, maximising endurance on remaining
        # fuel.  The loiter speed is lower than cruise speed
        # because CL_opt > CL_cruise.
        V_loiter = self.best_LD_speed(self.cruise_alt)
        for _ in range(self.loiter_dur):
            rows.append(self._row(t, "loiter", self.cruise_alt, V_loiter, 0.0))
            t += 1

        # ────────────────────────────────────────────────────
        # PHASE 5 — DESCENT  (glide back to ground)
        # ────────────────────────────────────────────────────
        # Calculate windmilling drag and descend at equilibrium glide
        alt = self.cruise_alt
        while alt > 0:
            V_desc = self.best_LD_speed(alt, "descent")
            
            # Aerodynamic drag of airframe
            D_clean = self.ac.drag_force(self.W, V_desc, alt, "descent")
            
            # Extra drag from windmilling propeller
            rho = self.ac.air_density(alt)
            q = 0.5 * rho * V_desc**2
            D_wind = q * self.ac.S * getattr(self.ac, "windmilling_drag_coefficient", 0.0)
            
            # Equilibrium glide slope: T_total = 0 => W*sin(gamma) = - (D_clean + D_wind)
            sin_gamma = -(D_clean + D_wind) / self.W
            gamma = math.asin(max(-1.0, min(1.0, sin_gamma)))
            
            step_sink = -V_desc * sin_gamma
            if step_sink <= 0:
                step_sink = 3.0 # Fallback
            
            # Ensure we don't overshoot ground
            step_sink = min(step_sink, alt)
            
            # Mechanical power extracted by propeller from airstream
            P_aero = -D_wind * V_desc
            
            # Shaft power to generator is reduced by recovery_efficiency (from hybrid_sim config)
            # We'll assume recovery_efficiency = 0.5 if not explicitly configured here,
            # or just leave it to hybrid_sim.py? Wait, hybrid_sim.py already applies RECOVERY_EFFICIENCY!
            # Yes, hybrid_sim.py: p_elec_gen = p_regen_mech * GENERATOR_EFFICIENCY * RECOVERY_EFFICIENCY
            # So `power_required_W` should just be P_aero!
            P_req = P_aero
            
            row_dict = self._row(t, "descent", alt, V_desc, gamma)
            row_dict["power_required_W"] = P_req
            rows.append(row_dict)
            
            alt -= step_sink
            t += 1

        return pd.DataFrame(rows)
