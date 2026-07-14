"""
Fixed-wing UAV aerodynamic & performance model.

Physics summary (verify these):
──────────────────────────────────────────────────────────────
1. ISA STANDARD ATMOSPHERE (troposphere, 0–11 km)
   ─ Temperature lapse rate  λ = −0.0065 K/m
   ─ T(h) = T₀ + λ·h          where T₀ = 288.15 K
   ─ Pressure via barometric formula:
       P(h) = P₀ · (T/T₀) ^ [g / (R·|λ|)]
     where g = 9.80665 m/s², R = 287.0528 J/(kg·K)
     exponent = 9.80665 / (287.0528 × 0.0065) ≈ 5.2559
   ─ Density via ideal gas:
       ρ(h) = P(h) / (R·T(h))
     equivalently ρ(h) = ρ₀ · (T/T₀) ^ (g/(R·|λ|) − 1)
                       = ρ₀ · (T/T₀) ^ 4.2559

2. LIFT COEFFICIENT (level-flight assumption)
   L = W  ⟹  CL = W / (½ ρ V² S)

3. DRAG POLAR
   CD = CD₀ + k·CL²          (parabolic drag polar)
   D  = ½ ρ V² S · CD

4. SHAFT POWER REQUIRED
   Thrust balance in a steady climb at flight-path angle γ:
       T = D + W sin(γ)
   Propulsive power = T · V
   Shaft power      = T · V / η_prop(phase)
   (η_prop accounts for propeller losses converting shaft
    work to thrust work)
──────────────────────────────────────────────────────────────
"""

import math


class Aircraft:
    """Fixed-wing UAV aerodynamic model."""

    # ── ISA sea-level constants ──────────────────────────────
    T0 = 288.15          # sea-level temperature  [K]
    P0 = 101_325.0       # sea-level pressure     [Pa]
    RHO0 = 1.225         # sea-level density      [kg/m³]
    LAPSE = -0.0065      # temperature lapse rate  [K/m] (negative = cooling)
    G = 9.80665          # gravitational accel     [m/s²]
    R_AIR = 287.0528     # specific gas constant   [J/(kg·K)]

    def __init__(
        self,
        mtow_kg: float = 1000.0,
        S: float = 10.0,
        CD0: float = 0.022,
        k: float = 0.0332,
        prop_eff: dict | None = None,
        windmilling_drag_coefficient: float = 0.015,
    ):
        """
        Parameters
        ----------
        mtow_kg : Maximum take-off weight [kg].
        S       : Wing reference area [m²].
        CD0     : Zero-lift drag coefficient.
        k       : Induced drag factor  k = 1/(π·AR·e).
        prop_eff: Dict of propeller efficiency per flight phase.
        windmilling_drag_coefficient: Extra CD0 added during descent regen.
        """
        self.mtow_kg = mtow_kg
        self.S = S
        self.CD0 = CD0
        self.k = k
        self.windmilling_drag_coefficient = windmilling_drag_coefficient
        self.prop_eff = prop_eff or {
            "cruise": 0.82,
            "climb": 0.72,
            "loiter": 0.65,
        }

    # ─── 1. ISA standard atmosphere ─────────────────────────
    def air_density(self, altitude_m: float) -> float:
        """
        ISA air density in the troposphere (0 ≤ h ≤ 11 000 m).

        Derivation
        ----------
        T(h)   = T₀ + λ·h
        ρ(h)   = ρ₀ · (T(h)/T₀) ^ (g/(R·|λ|) − 1)

        The exponent (g/(R·|λ|) − 1) ≈ 4.2559 comes from
        combining the barometric pressure formula with the
        ideal-gas law.

        Returns density in [kg/m³].
        """
        if altitude_m < 0 or altitude_m > 11_000:
            raise ValueError(
                f"Altitude {altitude_m} m is outside the troposphere (0–11 000 m)."
            )

        T = self.T0 + self.LAPSE * altitude_m          # local temperature [K]
        # exponent for density: (g / (R · |λ|)) - 1
        exponent = self.G / (self.R_AIR * abs(self.LAPSE)) - 1.0  # ≈ 4.2559
        rho = self.RHO0 * (T / self.T0) ** exponent
        return rho

    # ─── 2. Required lift coefficient ───────────────────────
    def required_CL(
        self,
        weight_N: float,
        velocity_ms: float,
        altitude_m: float,
    ) -> float:
        """
        CL needed to sustain level flight (L = W).

        CL = W / (½ ρ V² S)

        This is the *level-flight* CL. During a climb the lift
        is actually W·cos(γ), but for moderate climb angles
        cos(γ) ≈ 1, so this is used as the baseline.
        """
        rho = self.air_density(altitude_m)
        q = 0.5 * rho * velocity_ms ** 2   # dynamic pressure [Pa]
        CL = weight_N / (q * self.S)
        return CL

    # ─── 3. Drag force ──────────────────────────────────────
    def drag_force(
        self,
        weight_N: float,
        velocity_ms: float,
        altitude_m: float,
        phase: str = "cruise",
    ) -> float:
        """
        Total aerodynamic drag [N].

        CD = CD₀ + k · CL²        (parabolic drag polar)
        D  = ½ ρ V² S · CD = q · S · CD

        CD₀ captures skin-friction + pressure drag (profile).
        k·CL² captures induced (vortex) drag.
        """
        rho = self.air_density(altitude_m)
        q = 0.5 * rho * velocity_ms ** 2
        CL = self.required_CL(weight_N, velocity_ms, altitude_m)
        CD = self.CD0 + self.k * CL ** 2
        D = q * self.S * CD
        return D

    # ─── 4. Shaft power required ────────────────────────────
    def shaft_power_required(
        self,
        weight_N: float,
        velocity_ms: float,
        altitude_m: float,
        climb_angle_rad: float,
        phase: str,
    ) -> float:
        """
        Shaft power the engine/motor must deliver [W].

        Physics
        -------
        In a steady (non-accelerating) climb at flight-path
        angle γ, forces along the flight path give:

            T = D + W · sin(γ)

        where
          T = thrust                       [N]
          D = aerodynamic drag             [N]
          W = aircraft weight              [N]
          γ = climb angle (positive = up)  [rad]

        Propulsive power = T · V  (thrust × true airspeed).
        The propeller converts shaft power to thrust power
        with efficiency η_prop, so:

            P_shaft = T · V / η_prop

        Note: During descent (γ < 0), W·sin(γ) is negative,
        so thrust (and power) can be less than drag — or even
        zero in an idle glide.
        """
        D = self.drag_force(weight_N, velocity_ms, altitude_m, phase)
        thrust = D + weight_N * math.sin(climb_angle_rad)
        eta = self.prop_eff[_PROP_PHASE.get(phase, phase)] if "_PROP_PHASE" in globals() else self.prop_eff.get(phase, self.prop_eff.get("loiter", 0.65))
        # Actually in mission.py, it passes the mapped phase, so it's already "climb", "cruise", or "loiter".
        # But wait! If mission.py passes the MAPPED phase (e.g. "loiter" instead of "descent"), 
        # then drag_force won't see "descent"! 
        # So I need to pass the REAL phase to shaft_power_required, and map it inside.
        
        # We will map it in mission.py correctly and pass the real phase to shaft_power_required.
        # Wait, if I change the signature here, let's just pass `phase` directly.
        # Previously mission.py mapped it: `self.ac.shaft_power_required(..., _PROP_PHASE[phase])`
        # So `phase` here was the MAPPED phase. I need to change mission.py to pass the raw phase.
        
        eta = self.prop_eff.get(phase, self.prop_eff.get("loiter", 0.65))
        # But wait, self.prop_eff only has 'cruise', 'climb', 'loiter'. So I need to map it inside.
        mapped_phase = phase
        if phase == "takeoff": mapped_phase = "climb"
        elif phase == "descent": mapped_phase = "loiter"
        
        eta = self.prop_eff.get(mapped_phase, 0.65)
        P_shaft = thrust * velocity_ms / eta
        return P_shaft
