# AVM — Adaptive Volume Mode

Per-level feed/engagement optimization for Fusion 360 adaptive clearing.
Holds **tool load constant across stepdown levels** instead of letting the
CAM run every level at worst-case parameters. Where the tool is lightly
engaged, width of cut rises to match the load of the proven deep pass;
where it's buried, nothing changes. Physics, not ML.

## Validated results (2026-07-08, Delrin, Haas Mini Mill)

| Claim | Result |
|---|---|
| Sim cutting time, in-scope part (wavy saddle) | **5:03 → 3:06** (3:41 vs 3:06 after baseline linking cleanup = **15.8% physics**) |
| Wall clock, same part, 50% rapids, incl. 2D pocket op | 4:52 → 4:39 (**4.5%**; delta eaten by inter-op rapids at half override — machine-specific, see FINDINGS) |
| Out-of-scope part (mixed slab/column geometry) | 10:32 → 9:07 sim (**13.5%** — constraint stack degrades gracefully) |
| Posted G-code size | **~10 KB smaller** (matters on memory-limited controls) |
| Predicted cutting power (Kienzle + beam model) | **~0.2 kW predicted, ~0.19 kW measured** (5% load delta on ~5 hp spindle) |
| Stock parity vs baseline | Same rest material (SWEEP op guarantees floor parity) |
| Tool damage across 19 dev versions | Zero |

## How it works

1. **Physics core** (`avm_physics/`, pure stdlib, zero Fusion deps):
   - Segmented cantilever beam (unit-load method; handles necked tools)
   - Kienzle cutting force with radial chip-thinning and a helix-smearing
     duty factor (deep cuts are steady, shallow cuts are spiky)
   - Dual-constraint per-level solver: force branch (ratio-calibrated to
     the op's own proven deep pass — material constants cancel) and
     deflection branch (absolute cap, or baseline-relative "deflect no
     worse than the cut you already trust", which is also material-free)
   - Level banding: merges adjacent levels within a cumulative ae ratio,
     hard-capped clone count
2. **Fusion cloner** (`fusion/AVM_LevelClone/`): duplicates the selected
   adaptive op per band via native `op.duplicate()`, sets per-band
   optimal load / bottom height / minimum-axial-engagement (the level
   selector — requires flat-area detection OFF, see FINDINGS), orders
   deep-first, folders the stack, suppresses the source, and writes a
   full audit report. Non-destructive: the original op is never edited.
3. **A/B validation**: clones active + source suppressed = optimized;
   flip both = baseline. Machine-sim both, diff rest material and time.

The tool has three honest answers: **optimize** (bands with the binding
constraint labeled per level), **no-op** ("your op is already at its
limit under the calibration rule"), and **decline** (slab-dominated
structure where constant load is already correct).

## Install

Each folder under `fusion/` is a Fusion 360 Script: copy the folder (with
`avm_physics/` inside it for AVM_LevelClone — run `./build_release.sh` to
assemble), then Utilities → Add-Ins → Scripts → green + → point at it.

Workflow: select adaptive op(s) → run AVM_LevelClone → set dialog (
baseline mode recommended; it needs no material data) → regen the DEEPEST
band first and inspect → generate the rest from the Fusion UI → run
AVM_Prune to delete empty bands → sim A/B → post.

**Run on a file copy.** Regen cost ≈ source op regen × band count
(serial: rest-machining bands depend on their predecessors).

## Best use / known limits (v1)

- **Best on:** wavy/organic floors with fine stepups — anywhere actual
  engagement varies across levels while the CAM holds one load.
- **No benefit:** straight-wall constant-depth pockets (every pass is
  already worst-case; the tool declines these).
- **Degrades gracefully on mixed geometry:** the ap model assumes virgin
  columns (depth below stock top). On bimodal wall populations some bands
  come up empty and the SWEEP absorbs the slab work — still net faster in
  testing, but v2's toolpath-derived engagement is the real fix.
- Stickout is read from `tool_bodyLength` and is a property of the
  **mounting**, not the tool. Re-verify per machine/holder. It's the
  cube-law input; it gets the paranoia.

## Repo layout

    avm_physics/     physics core (pure Python, pytest-covered)
    fusion/          Fusion 360 scripts (cloner, probe, prune)
    tests/           acceptance = the validated 9-level demo table
    docs/            FINDINGS.md (API bestiary), ROADMAP.md
    run_adaptive3.py standalone acceptance run

## License

TBD by the author.
