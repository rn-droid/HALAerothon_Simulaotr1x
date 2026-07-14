"""
Series-parallel mechanical clutch model.

In a series-parallel hybrid, the clutch selects between:

  Parallel mode
    Engine shaft is mechanically coupled directly to the propeller.
    No generator/motor conversion losses on the engine path.
    Selected when the engine is operating near its best-BSFC point.

  Series mode
    Engine drives a generator; electrical power is routed through
    the motor to the propeller.  generator_eta * motor_eta conversion
    losses apply to the engine path.  Selected when power demand is
    far from the engine's efficient zone.

Fail-safe:  any fault_flag forces series mode.  The generator/motor
electrical path provides full speed-torque controllability and is
the safer default if a mechanical coupling fault is detected.

Mass model:
    m_clutch = 2 kg (base)  +  0.02 * engine_rated_kW  (power-scaled)
"""


class Clutch:
    """Series-parallel clutch with BSFC-aware mode selection."""

    def __init__(
        self,
        engine_rated_kW: float,
        optimal_pf: float = 0.80,
        band_pct: float = 0.15,
    ):
        """
        Parameters
        ----------
        engine_rated_kW : Engine rated power [kW], used for mass calc.
        optimal_pf      : Power fraction at minimum BSFC (default 0.80,
                          corresponding to 375 g/kWh in the reference
                          turboshaft table).
        band_pct        : Half-width of the parallel-selection band as
                          a fraction of optimal_pf (default 0.15 = +/-15%).
                          Resulting band: [0.68, 0.92] for default values.
        """
        self.engine_rated_kW = engine_rated_kW
        self.mass_kg = 2.0 + 0.02 * engine_rated_kW

        self.optimal_pf = optimal_pf
        self.band_pct = band_pct
        self.pf_low = optimal_pf * (1.0 - band_pct)     # 0.68
        self.pf_high = optimal_pf * (1.0 + band_pct)    # 0.92

    # ─────────────────────────────────────────────────────────
    def select_mode(
        self,
        power_required_W: float,
        engine_rated_W: float,
        fault_flag: bool = False,
    ) -> str:
        """
        Choose 'parallel' or 'series' coupling.

        Returns 'parallel' when BOTH:
          1. fault_flag is False
          2. power_required / engine_rated falls within
             [pf_low, pf_high]  (engine near best BSFC)

        Otherwise returns 'series'.
        """
        if fault_flag:
            return "series"

        pf = power_required_W / engine_rated_W
        if self.pf_low <= pf <= self.pf_high:
            return "parallel"
        return "series"
