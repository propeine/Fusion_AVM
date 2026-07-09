"""ADAPTIVE3 acceptance run — Tom's Delrin demo part, real probe numbers."""
from avm_physics.beam import standard_endmill
from avm_physics.cutting import CutGeometry, MATERIALS
from avm_physics.solver import solve_levels, Limits

# Probe-verified inputs
D, Z, RPM = 0.375, 3, 6000.0
FZ0 = 88.0 / (RPM * Z)          # 0.004889 in
AE0 = 0.020
STICKOUT, LOC = 2.197, 1.75

geom = CutGeometry(D, Z, helix_deg=38.0)
beam = standard_endmill(D, STICKOUT, LOC, core_factor=0.75)
mat = MATERIALS["DELRIN"]

c_tip = beam.tip_deflection_per_lbf(0.0)
print(f"tip compliance: {c_tip*1e3:.4f} thou/lbf -> 0.001\" at {1/(c_tip*1e3):.1f} lbf tip load")
print(f"fz0 = {FZ0:.5f} in/tooth\n")

levels = [(k+1, 0.937 - k*0.09375) for k in range(9)]

for kc11, tag in [(mat.kc11_lo, "Kc11=180 (soft-cutting Delrin)"),
                  (mat.kc11, "Kc11=300 (nominal)"),
                  (mat.kc11_hi, "Kc11=550 (worst-case Delrin)")]:
    res = solve_levels(geom, beam, mat, RPM, FZ0, AE0, levels,
                       Limits(), kc11_override=kc11)
    print(f"=== {tag} ===")
    print(f"{'lvl':>3} {'ap':>6} {'ae':>6} {'WOC%':>5} {'fz':>7} {'feed':>6} "
          f"{'Flat(N)':>8} {'defl(in)':>9} {'budget(N)':>9} {'kW':>5} {'x':>5}  binding")
    for r in res:
        print(f"{r.level:>3} {r.ap_in:>6.3f} {r.ae_in:>6.3f} {100*r.ae_frac:>5.1f} "
              f"{r.fz_in:>7.4f} {r.feed_inmin:>6.0f} {r.f_lat_peak_N:>8.1f} "
              f"{r.deflection_in:>9.5f} {r.defl_budget_N:>9.1f} {r.power_kw:>5.2f} "
              f"{r.speedup:>5.2f}  {r.binding}")
    print(f"  force target = {res[0].force_target_N:.1f} N (baseline peak lateral)\n")
