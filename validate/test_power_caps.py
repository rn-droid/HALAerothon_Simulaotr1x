import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.hybrid_sim import run_naive_hybrid

def test_undersized_powertrain():
    # Use a highly undersized powertrain (engine 10 kW, motor 10 kW)
    # The climb phase needs ~115 kW for a 1000 kg MTOW aircraft.
    df = run_naive_hybrid(
        fault_flag=False,
        ecms_s=4.0,
        loiter_duration_s=0,
        mtow_kg=1000.0,
        engine_rated_W=10_000.0,
        battery_capacity_kWh=5.0,
        supercap_mass_kg=1.0,
        motor_rated_W=10_000.0
    )
    
    assert df.attrs.get("feasible") == False, "Mission should fail due to power caps."
    print("Test passed: Undersized powertrain correctly reported infeasibility.")
    print(f"Failed phase: {df.attrs.get('fail_phase')}")
    print(f"Deficit: {df.attrs.get('fail_deficit_W'):.1f} W")

if __name__ == "__main__":
    test_undersized_powertrain()
