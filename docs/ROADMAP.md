# ROADMAP

## Soon
1. **True engagement extraction (v2 core).** Read actual cutting levels &
   wall heights from the source toolpath (`generatedDataCollection` —
   next probe: iterate items, dump contents). Cures the whole
   model-vs-reality bug class: pocket ap, fictional parameter grids,
   mixed slab/column geometry. Biggest remaining gains live here.
2. Restore full per-band force/deflection columns in the report
   (slimmed during the banding restructure).
3. Speedio validation run: full rapids should close the sim/wall-clock
   gap; then RPM scaling to 10k (+67% feed at constant chipload) as a
   separate stacked experiment.
4. Absolute-mode push run: measured_power=0.19 kW (Delrin, calibrated
   2026-07-08) + roughing deflection cap 0.002-0.003 to beat the
   baseline's own deep pass.

## Eventually
5. Package as a proper add-in: toolbar button, right-click on CAM ops,
   folder-level suppress as one-click A/B toggle, auto-prune empties
   after regen.
6. Bending-stress third branch (TRS/SF) — protects stubby tools that
   deflection caps under-protect.
7. Calibration persistence: local DB of measured-power per
   material/machine.
8. Kinematic (accel-aware) time estimate in the report next to Fusion's.
9. Stepover-during-generation research track (medial axis) — parked;
   per-band regeneration captures most of it at far lower cost.

## Not doing
- Manual G-code retract surgery (fragile, per-post; Speedio rapids
  mostly obsolete the problem).
- ML anything in the core loop. It's mechanics.
