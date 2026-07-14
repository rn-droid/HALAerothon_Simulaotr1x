"""
simulation/ — Refactored high-fidelity subsystem models (Phase 5).

Modules:
  atmosphere      — ISA standard atmosphere
  aerodynamics    — Drag polar, thrust, shaft power demand
  engine_model    — Turboshaft with BSFC interpolation
  battery_model   — Li-ion NMC SoC integrator with C-rate checks
  power_electronics — Generator→rectifier→bus→inverter→motor chain
  mission_profile — 6-phase mission segment builder
  controller      — Phase 4 Strategy A power-split state machine
  run_simulation  — Main orchestrator; exports CSV + JSON
"""
