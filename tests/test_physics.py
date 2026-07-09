"""Acceptance + physics gut-checks for the AVM solver."""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from avm_physics.beam import standard_endmill
from avm_physics.cutting import CutGeometry, MATERIALS, cut_forces
from avm_physics.solver import solve_levels, Limits

D, Z, RPM = 0.375, 3, 6000.0
FZ0, AE0 = 88.0/(RPM*Z), 0.020
GEOM = CutGeometry(D, Z, 38.0)
BEAM = standard_endmill(D, 2.197, 1.75)
LEVELS = [(k+1, 0.937 - k*0.09375) for k in range(9)]

def test_cube_law_stickout_uniform():
    # pure cube law: same section throughout -> exactly 8x for 2x length
    c1 = standard_endmill(0.5, 1.0, 1.0).tip_deflection_per_lbf(0.0)
    c2 = standard_endmill(0.5, 2.0, 2.0).tip_deflection_per_lbf(0.0)
    assert 7.8 < c2/c1 < 8.2

def test_cube_law_stickout_stepped():
    # added length is fat shank: d^4 blunts the cube -> ~3.2x, not 8x
    c1 = standard_endmill(0.5, 1.0, 1.0).tip_deflection_per_lbf(0.0)
    c2 = standard_endmill(0.5, 2.0, 1.0).tip_deflection_per_lbf(0.0)
    assert 2.8 < c2/c1 < 3.7

def test_d4_diameter():
    c_half  = standard_endmill(0.5,  1.5, 1.0).tip_deflection_per_lbf(0.0)
    c_three = standard_endmill(0.75, 1.5, 1.0).tip_deflection_per_lbf(0.0)
    assert 4.0 < c_half/c_three < 6.5   # (0.75/0.5)^4 ~ 5x stiffer

def test_load_height_reduces_tip_deflection():
    b = standard_endmill(0.375, 2.0, 1.75)
    assert b.tip_deflection_per_lbf(0.5) < b.tip_deflection_per_lbf(0.05)

def test_thinning_h_avg_below_fz_at_low_immersion():
    f = cut_forces(GEOM, MATERIALS["DELRIN"], 0.937, 0.020, FZ0, RPM)
    assert f.h_avg_mm < FZ0*25.4

def test_helix_smearing_duty():
    deep = cut_forces(GEOM, MATERIALS["DELRIN"], 0.937, 0.020, FZ0, RPM)
    shal = cut_forces(GEOM, MATERIALS["DELRIN"], 0.187, 0.020, FZ0, RPM)
    assert deep.duty == 1.0 and shal.duty < 1.0

def test_baseline_level_reproduces_target_when_force_bound():
    res = solve_levels(GEOM, BEAM, MATERIALS["DELRIN"], RPM, FZ0, AE0,
                       LEVELS, Limits(), kc11_override=180.0)
    assert abs(res[0].ae_in - AE0) < 0.0005          # level 1 = baseline
    assert abs(res[0].f_lat_peak_N - res[0].force_target_N) < 1.0

def test_monotonic_gains_when_force_bound():
    res = solve_levels(GEOM, BEAM, MATERIALS["DELRIN"], RPM, FZ0, AE0,
                       LEVELS, Limits(), kc11_override=180.0)
    for a, b in zip(res, res[1:]):
        assert b.ae_in >= a.ae_in - 1e-9

def test_deflection_never_exceeds_budget():
    for kc in (180.0, 300.0, 550.0):
        res = solve_levels(GEOM, BEAM, MATERIALS["DELRIN"], RPM, FZ0, AE0,
                           LEVELS, Limits(), kc11_override=kc)
        for r in res:
            assert r.deflection_in <= 0.001*1.0001

def test_aluminum_deflection_bound_at_this_stickout():
    res = solve_levels(GEOM, BEAM, MATERIALS["AL6061"], RPM, FZ0, AE0,
                       LEVELS, Limits())
    assert all("deflection" in r.binding for r in res)

def test_measured_power_recalibrates():
    r_lo = solve_levels(GEOM, BEAM, MATERIALS["DELRIN"], RPM, FZ0, AE0,
                        LEVELS, Limits(), measured_power_kw=0.10)
    r_hi = solve_levels(GEOM, BEAM, MATERIALS["DELRIN"], RPM, FZ0, AE0,
                        LEVELS, Limits(), measured_power_kw=0.40)
    assert r_lo[8].ae_in > r_hi[8].ae_in   # softer measured cut -> more ae
