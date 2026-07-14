"""
Atmosphere Model — ISA Standard Atmosphere (Troposphere, 0-11 km)

Provides air density, temperature, and pressure as functions of altitude.
Cross-references: ATM-01 through ATM-05 in assumption_register.md
"""

import math
from config import T0_ISA, P0_ISA, LAPSE_RATE, R_AIR, G


def isa_temperature(h_m: float) -> float:
    """ISA temperature at altitude h [m]. Returns [K]."""
    return T0_ISA + LAPSE_RATE * h_m


def isa_pressure(h_m: float) -> float:
    """ISA pressure at altitude h [m]. Returns [Pa]."""
    T = isa_temperature(h_m)
    return P0_ISA * (T / T0_ISA) ** (-G / (LAPSE_RATE * R_AIR))


def isa_density(h_m: float) -> float:
    """ISA air density at altitude h [m]. Returns [kg/m^3]."""
    T = isa_temperature(h_m)
    p = isa_pressure(h_m)
    return p / (R_AIR * T)
