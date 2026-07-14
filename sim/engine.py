"""
Turboshaft engine model with BSFC lookup table.

Physics
-------
BSFC (Brake Specific Fuel Consumption) is the mass of fuel burned
per unit of shaft energy delivered:

    BSFC  [g / (kW h)]

The BSFC of a small turboshaft has a characteristic "bowl" shape:
  - Worst at low power  (high parasitic losses relative to output)
  - Best near 70-85 % rated  (design point)
  - Slightly worse at 100 %  (thermal / aero limits)

Fuel mass-flow from BSFC:
    fuel_flow [g/h]  = BSFC [g/(kW h)]  x  P [kW]
    fuel_flow [kg/s] = BSFC  x  P_kW / (1000  x  3600)
                     = BSFC  x  P_kW / 3 600 000
"""

import numpy as np


class Turboshaft:
    """Simple turboshaft with tabulated BSFC."""

    def __init__(self, rated_power_W: float = 60_000):
        self.rated_power_W = rated_power_W

        # ── BSFC lookup table ───────────────────────────────
        # Power fraction (P / P_rated) vs BSFC [g/(kW h)]
        self._pf = np.array([0.20, 0.30, 0.40, 0.50,
                             0.60, 0.70, 0.80, 0.90, 1.00])
        self._bsfc = np.array([620, 520, 460, 420,
                               395, 380, 375, 380, 395], dtype=float)

        # Idle power = lowest tabulated fraction
        self._idle_fraction = float(self._pf[0])

    # ─────────────────────────────────────────────────────────
    def bsfc(self, power_fraction: float) -> float:
        """
        Interpolated BSFC [g/(kW h)] at a given power fraction.

        Uses linear interpolation over the table.  Values outside
        [0.2, 1.0] are clamped to the nearest edge (np.interp
        default behaviour).
        """
        return float(np.interp(power_fraction, self._pf, self._bsfc))

    # ─────────────────────────────────────────────────────────
    def fuel_flow_kg_s(self, power_W: float) -> float:
        """
        Instantaneous fuel flow [kg/s] for a given shaft output.

        Handles three regimes:
          P <= 0           : engine at idle (minimum-tabulated fraction)
          0 < P <= P_rated : normal operation, BSFC interpolated
          P > P_rated      : BSFC clamped at rated-power value;
                             fuel flow still scales with actual P.
                             (Represents an oversized engine for
                             the non-hybrid baseline comparison.)
        """
        if power_W <= 0:
            # Engine at idle — still burns fuel
            eff_power_W = self._idle_fraction * self.rated_power_W
        else:
            eff_power_W = power_W

        pf = eff_power_W / self.rated_power_W
        bsfc_val = self.bsfc(pf)                        # g/(kW h)
        power_kW = eff_power_W / 1000.0
        return bsfc_val * power_kW / 3_600_000           # kg/s
