# CHANGELOG — one afternoon, 2026-07-08

- v0.1  Cloner: per-level clones, rest-machining suppression fix, multi-op
- v0.2  createInput + parameter transplant fallback, API survey born
- v0.3  Report fallback chain + Text Commands mirror
- v0.4  fz from tool_feedPerTooth (feed params are mm/min!), sanity guards
- v0.5  Surgical transplant blocklist (tool_* wedges kernel), no auto-regen
- v0.6  Cached source capture, timing instrumentation
- v0.7  minimumAxialEngagement level selector (Tom); baseline deflection mode
- v0.8  FAD-off per clone (Tom, hand-tested) + SWEEP floor-parity clone
- v0.9  Package rename + sys.modules purge (stale module shadowing)
- v0.10 Fixed-name latest report, config echo
- v0.11 Native op.duplicate attempt, stale-proxy fix
- v0.12 Tree-diff adoption of duplicate()'s side effect
- v0.13 Command dialog (config popup), script->add-in architecture
- v0.14 doEvents flush (duplicate is deferred!), orphan cleanup, folders probed
- v0.15 Reversed creation order (duplicate inserts after source), order verify
- v0.16 Deep-first folder moves, folder placed after source, in-folder verify
- v0.17 Slab-dominated opportunity detector ("decline"), best-use notes
- v0.18 Level banding + clone cap (80-level grid explosion), dialog knobs
- v0.19 Base-relative banding (transitive collapse fix), no-op detection
- Prune utility: delete empty bands post-regen, report-first
- v0.20 Per-level RPM thermal derating (forum-requested, Practical
  Machinist thread): duty-factor interpolation between the source op's
  proven WOC (0%) and the widest clone (entered %), feed rescaled per
  clone's OWN fz (preserves solver fz raises), dormant at 0%.
  avm_physics/derate.py + 13 invariant tests.

Validated same day: 15.8% sim (in-scope), 13.5% sim (out-of-scope),
4.5% wall clock on a rapid-hostile 2001 Haas, ~0.19 kW cutting power
measured vs ~0.2 predicted, 10 KB smaller posted file, stock parity.
