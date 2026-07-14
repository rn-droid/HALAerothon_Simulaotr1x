"""
Aviation-grade permanent-magnet electric motor / generator model.

Physics
-------
Motor mode  (mechanical power out):
    P_elec_in = P_mech_out / eta_motor

Generator mode would use generator efficiency (0.93 per ref data),
but this class models the motor only.  Generator losses can be
applied externally or via a separate wrapper.

Mass:
    mass_kg = P_rated [kW] / power_density [kW/kg]
"""


class ElectricMotor:
    """PM electric motor with fixed efficiency."""

    def __init__(
        self,
        rated_power_W: float,
        power_density_kW_per_kg: float = 5.0,
        efficiency: float = 0.94,
    ):
        """
        Parameters
        ----------
        rated_power_W          : Rated continuous mechanical output [W].
        power_density_kW_per_kg: Gravimetric power density [kW/kg].
        efficiency             : Motor electrical-to-mechanical efficiency.
        """
        self.rated_power_W = rated_power_W
        self.power_density = power_density_kW_per_kg
        self.efficiency = efficiency
        self.mass_kg = (rated_power_W / 1000.0) / power_density_kW_per_kg

    def electrical_power_in(self, mechanical_power_W: float) -> float:
        """
        Electrical power the motor draws to deliver a given
        mechanical (shaft) output.

        P_elec = P_mech / eta

        Returns a value >= P_mech (motor always consumes more
        electrical power than it delivers mechanically).
        """
        return mechanical_power_W / self.efficiency
