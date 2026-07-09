# FINDINGS — Fusion 360 CAM API & environment bestiary

Everything below was learned empirically on Fusion 2703.1.x (2026-07-08)
and is not (well) documented. Each cost at least one broken run.

## Units
- CAM parameter **length** values are internal **centimeters**
  (`tool_diameter` 0.375 in → 0.9525).
- CAM **feed** values are **mm/min** (`tool_feedCutting` 88 in/min →
  2235.2). Deriving chipload from feed with the cm assumption is 10x off.
  **Read `tool_feedPerTooth` directly instead** — it's in consistent
  length units. Sanity-guard every unit read; a wrong unit presents as
  the solver "derating everything," not as an error.
- The **Machining Time dialog reports feed/rapid distance in millimeters
  labeled with document units** (e.g. "9340.563 in" on a program whose
  true feed distance is ~368 in = 9340 mm). Sanity check: distance /
  programmed feed must roughly equal displayed feed time; off by ~25.4x
  means you're reading mm. Percentage comparisons between programs are
  unaffected (units cancel). Caught 2026-07-09 by smell: 10,000 inches
  at 88 ipm cannot finish in five minutes.

## Operation duplication
- `op.createTemplate()` does not exist. `operations.duplicate(op)` does
  not exist. The real methods are **on the operation**: `duplicate`,
  `copyAfter`, `copyBefore`, `copyInto`, plus `moveInto/moveAfter/
  moveBefore` (survey keyword filters must match "moveinto", not
  "moveto").
- **`op.duplicate()` returns None but creates the copy anyway — 
  DEFERRED.** The copy materializes only after control returns to the
  event loop. Pump `adsk.doEvents()` and adopt the new op by tree diff
  (entityToken snapshot before/after). Never trust the return value;
  never keep calling it after a failure (it litters orphan copies).
- `duplicate()` inserts the copy **immediately after the source**, so
  creating N clones deep-first yields an INVERTED tree. Create in
  reverse order and verify the final tree order by reading it back.
  Tree order = execution order = rest-machining correctness.

## Parameters
- Do **not** write `tool_*` parameter expressions onto an op that has a
  tool assigned — they are derived from the tool object and writing them
  desyncs op state (wedged the CAM kernel; document-wide regen refusal,
  restart required). Feeds/speeds subset is safe to override.
- Do **not** cache CAMParameter proxy objects across writes — Fusion
  invalidates them when the parameter graph mutates. Fetch fresh via
  `itemByName` per write.
- `optimalLoad` default expression is `'tool_stepover'` (inherits the
  tool preset). Overwrite with a literal like `'0.035 in'`;
  `maximumLoad` auto-derives (+10%).
- `maximumStepdown` / `fineStepdown` are derived expressions
  (`min(fluteLength*0.75, dia*2.5)` etc.), not typed values.

## Level confinement (the core trick)
- `fineStepdown = maximumStepdown` does NOT confine an adaptive to one
  level — floor-conforming passes are always generated.
- **`minimumAxialEngagement` is a wall-height filter** and works as a
  level selector (threshold midway between adjacent band depths) —
  **but only with Flat Area Detection OFF.** FAD floor passes bypass the
  engagement filter (found by hand-testing in the UI, not the API).
- Turning FAD off orphans between-grid floor slivers → append a SWEEP
  clone (FAD on, minAxial 0, rest machining) to restore stock parity.
  The slivers are tiny-ap cuts, i.e. the ideal wide-load case anyway.
- Rest machining (`defineStockBy='rest'`) is the level-linking
  mechanism: each clone only cuts what predecessors left. Requires the
  SOURCE op to be suppressed before regen or every clone is empty.

## Folders
- `setup.folders.addFolder(name)` (not `.add`). CAMFolder is a full CAM
  object: `allOperations`, `isSuppressed` (folder-level A/B toggle!),
  `moveAfter` (position it after the source op).
- Ops moved into a folder via `op.moveInto(folder)` **append in move
  order** — move deep-first or the folder inverts the sequence. Verify
  order inside the folder after moving.

## Environment
- Fusion's embedded Python **persists modules across script runs** —
  purge `sys.modules` of your package on every launch or a stale version
  silently shadows the new one. Use a unique package name.
- Script API calls run on the main thread; per-call overhead is ~ms.
  Thousands of parameter reads look like a hang at 5% CPU. Batch reads,
  cache immutable data (per run, not across mutations), and never let
  the user click around mid-script.
- Report files: Desktop may be OneDrive-redirected. Write a fixed-name
  latest file with a Desktop → home → temp fallback chain AND mirror to
  the Text Commands palette (`app.log`).
- `generatedDataCollection` exists on operations (count/item/
  itemByIdentifier) — empty-toolpath detection, and the future path to
  reading TRUE cutting levels instead of the parameter grid. Item
  contents not yet probed (v2 opening move).

## Physics notes that survived contact with the machine
- Helix smearing: at ~1" DOC a 38° helix wraps ~220°, so cutting force
  is steady at the average; shallow cuts are spiky. Model duty factor
  = min(1, z(φ + ap·tanβ/R)/2π).
- Chip thickness saturates at 50% WOC — past it, wider ae adds arc time
  and heat, not chip. Cap ~37.5–40%.
- The naive stickout cube law (8x for 2x length) holds only for uniform
  sections; a fat shank blunts it (~3.2x measured for flute+shank).
- 0.001" deflection at long stickout is an accuracy/chatter limit, not
  a breakage limit (bending stress ~9 ksi vs carbide TRS ~400 ksi).
  Deflection caps under-protect SHORT tools — a stress branch is on the
  roadmap.
- Baseline-relative calibration ("never exceed the proven cut") makes
  material constants cancel in BOTH branches. It also means the tool can
  never beat the baseline's own deep pass — pushing past that is
  absolute mode + a measured spindle-power calibration.
- Sim time ignores acceleration: it flatters dense trochoids (baseline)
  and understates the optimized program's real-world advantage on weak-
  lookahead controls — but inter-op rapids at reduced override eat wall-
  clock gains the sim can't see. Both effects observed on the same part.
