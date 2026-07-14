"""
Li-ion battery pack model with SoC tracking.

Physics
-------
The battery stores energy E = SoC * capacity.

Discharge (power_W > 0  --  battery delivers power to bus):
    Internal energy drained per step:
        dE = P_delivered * dt / eta_discharge
    The one-way discharge efficiency means the battery must
    drain MORE internally than what it delivers externally.

Charge (power_W < 0  --  bus pushes power into battery):
    Internal energy stored per step:
        dE = P_absorbed * dt * eta_charge
    Charge efficiency means LESS is stored than what arrives.

Round-trip efficiency:
    eta_rt = eta_discharge * eta_charge
    Split symmetrically:  eta_discharge = eta_charge = sqrt(eta_rt)

C-rate limits:
    Max discharge power = max_C * capacity_Wh   [W]
    (a 4C rate on a 10 kWh pack = 40 kW peak discharge)

SoC is hard-clamped to [soc_min, soc_max].  If a requested
power would violate the floor/ceiling, the actual delivered/
absorbed power is reduced accordingly.
"""

import math


class Battery:
    """Li-ion battery pack with coulomb-counting SoC model."""

    def __init__(
        self,
        capacity_kWh: float,
        specific_energy_Wh_per_kg: float = 220.0,
        max_discharge_C: float = 4.0,
        max_charge_C: float = 2.0,
        round_trip_efficiency: float = 0.95,
        soc_min: float = 0.20,
        soc_max: float = 0.90,
    ):
        self.capacity_kWh = capacity_kWh
        self.capacity_Wh = capacity_kWh * 1000.0
        self.specific_energy = specific_energy_Wh_per_kg
        self.max_discharge_C = max_discharge_C
        self.max_charge_C = max_charge_C
        self.soc_min = soc_min
        self.soc_max = soc_max

        # ── Derived quantities ──────────────────────────────
        self.mass_kg = self.capacity_Wh / self.specific_energy
        self.max_discharge_W = self.max_discharge_C * self.capacity_Wh
        self.max_charge_W = self.max_charge_C * self.capacity_Wh

        # Symmetric efficiency split
        self._eta_discharge = math.sqrt(round_trip_efficiency)
        self._eta_charge = math.sqrt(round_trip_efficiency)
        self.round_trip_efficiency = round_trip_efficiency

        # State — initialise at top of safe window
        self.soc = self.soc_max

    # ─────────────────────────────────────────────────────────
    @property
    def usable_energy_Wh(self) -> float:
        """Energy available between soc_min and current SoC."""
        return (self.soc - self.soc_min) * self.capacity_Wh

    # ─────────────────────────────────────────────────────────
    def step(self, power_W: float, dt_s: float) -> float:
        """
        Advance one timestep.

        Parameters
        ----------
        power_W : Requested power.
                  >0 = discharge (battery delivers to bus).
                  <0 = charge   (bus pushes into battery).
        dt_s    : Timestep duration [s].

        Returns
        -------
        float : Actual power delivered (>0) or absorbed (<0) [W].
                May be less than requested if C-rate or SoC
                limits are hit.
        """
        dt_h = dt_s / 3600.0

        if power_W > 0:
            # ── Discharge ───────────────────────────────────
            power_W = min(power_W, self.max_discharge_W)

            # Internal energy drained (more than delivered)
            energy_drain_Wh = power_W * dt_h / self._eta_discharge
            delta_soc = energy_drain_Wh / self.capacity_Wh

            # Enforce SoC floor
            available_soc = self.soc - self.soc_min
            if delta_soc > available_soc:
                delta_soc = max(available_soc, 0.0)
                energy_drain_Wh = delta_soc * self.capacity_Wh
                power_W = energy_drain_Wh * self._eta_discharge / dt_h if dt_h > 0 else 0.0

            self.soc -= delta_soc
            return power_W

        elif power_W < 0:
            # ── Charge ──────────────────────────────────────
            charge_power = min(-power_W, self.max_charge_W)

            # Internal energy stored (less than absorbed)
            energy_stored_Wh = charge_power * dt_h * self._eta_charge
            delta_soc = energy_stored_Wh / self.capacity_Wh

            # Enforce SoC ceiling
            headroom_soc = self.soc_max - self.soc
            if delta_soc > headroom_soc:
                delta_soc = max(headroom_soc, 0.0)
                energy_stored_Wh = delta_soc * self.capacity_Wh
                charge_power = energy_stored_Wh / (dt_h * self._eta_charge) if dt_h > 0 else 0.0

            self.soc += delta_soc
            return -charge_power

        return 0.0
