"""AVM Level Cloner v0.20 — split selected adaptive op(s) into per-level
clones with physics-computed optimal loads.

Run as a Fusion Script. SELECT one or more adaptive operations first
(any setups). Each op is processed within its own parent setup.

v0.1: suppresses the source op after cloning (CRITICAL: clones use rest
machining and sit after the source in the tree — with the source active
they would all generate empty toolpaths). Re-running on the same op
deletes its stale [AVM:<op>] clones first. AVM clones themselves are
never cloned.

What it does:
  1. Reads op parameters (stepdowns, heights, tool geometry, feeds).
  2. Computes the level set (main stepdown + fine stepups, deep-first).
  3. Runs the AVM dual-constraint solver per level (force + deflection).
  4. Duplicates the op once per level into a new folder-like naming scheme:
     - bottomHeight banded to that level
     - fineStepdown = maximumStepdown  (suppress stepups -> single level)
     - optimalLoad = solver's ae for that level
  5. Leaves the ORIGINAL op untouched. Regenerates clones.
  6. Writes a report next to the probe output.

VERIFY-IN-SIM items (flagged V1/V2 in code):
  V1: duplication mechanism (tries createFromCAMTemplate, falls back).
  V2: stepup suppression via fineStepdown=maximumStepdown. Confirm each
      clone generates exactly ONE Z level in machine sim.

Edit the CONFIG block before running.
"""

import adsk.core, adsk.fusion, adsk.cam
import traceback, os, sys, time, math

# make the bundled physics package importable AND force-reload it:
# Fusion's embedded interpreter persists across script runs, so a stale
# cached module from a previous version otherwise shadows the on-disk one.
_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)
for _m in list(sys.modules):
    if _m == "avm_physics" or _m.startswith("avm_physics."):
        del sys.modules[_m]

from avm_physics.beam import standard_endmill
from avm_physics.cutting import CutGeometry, MATERIALS
from avm_physics.solver import solve_levels, Limits, band_levels
from avm_physics.derate import solve_derates

# ------------------------ CONFIG (dialog defaults) ---------------------
MATERIAL = "DELRIN"          # key into physics.cutting.MATERIALS
KC11_OVERRIDE = None         # None = material nominal; or e.g. 180.0
MEASURED_POWER_KW = None     # spindle-load delta calibration, overrides Kc11
DEFLECTION_MODE = "absolute"  # "absolute" = hard cap below; "baseline" =
                              # no level deflects worse than the proven
                              # baseline cut (Kc-free, per rule #5)
DEFLECTION_LIMIT_IN = 0.001
DEFLECTION_SF = 1.0
WOC_CAP_FRAC = 0.375
CORE_FACTOR = 0.75           # effective flute core diameter / nominal
PREFIX = "[AVM"   # full tag becomes [AVM:<source op name>]
REGENERATE = False           # v0.5 default: create only. Validate the
                             # DEEPEST clone by regenerating it manually
                             # in the UI first, then generate the rest.
# -----------------------------------------------------------------------

IN = 2.54  # cm per inch (Fusion internal length unit is cm)


def p_in(params, name):
    """Read a length parameter in inches."""
    return params.itemByName(name).value.value / IN


def p_num(params, name):
    return params.itemByName(name).value.value


def set_expr(params, name, expr):
    params.itemByName(name).expression = expr


def compute_levels(max_step_in, fine_step_in, total_depth_in):
    """Deep-first level list mirroring Fusion's structure: main stepdowns
    every max_step, fine stepups between at fine_step. Returns depths (in)
    below stock top, deepest first."""
    depths = set()
    d = max_step_in
    while d < total_depth_in - 1e-6:
        depths.add(round(d, 6))
        d += max_step_in
    depths.add(round(total_depth_in, 6))
    # fine levels between each pair of main levels (and above the first)
    mains = sorted(depths)
    fine = set()
    prev = 0.0
    for m in mains:
        f = m - fine_step_in
        while f > prev + 1e-6:
            fine.add(round(f, 6))
            f -= fine_step_in
        prev = m
    return sorted(depths | fine, reverse=True)   # deepest first


_API_SURVEYED = False

def survey_api(setup, op, log):
    """One-time: report what duplication/template surfaces this Fusion
    build actually exposes, so we can adopt a native path if one exists."""
    global _API_SURVEYED
    if _API_SURVEYED:
        return
    _API_SURVEYED = True
    keys = ("template", "dupl", "copy", "paste", "clone", "createinput",
            "createfrom", "add", "folder", "container", "children",
            "move", "reorder")
    targets = [("op", op), ("setup", setup),
               ("setup.operations", setup.operations),
               ("adsk.cam", adsk.cam)]
    try:
        f = getattr(setup, "folders", None)
        if f is not None:
            targets.append(("setup.folders [FULL]", f))
    except Exception:
        pass
    for label, obj in targets:
        try:
            if "[FULL]" in label:
                hits = [m for m in dir(obj) if not m.startswith("_")]
            else:
                hits = [m for m in dir(obj)
                        if any(k in m.lower() for k in keys)
                        and not m.startswith("_")]
            log.append(f"[api survey] {label}: {hits}")
        except Exception as e:
            log.append(f"[api survey] {label}: <err {e}>")


_NATIVE_BROKEN = False

_SRC_CACHE = {}   # op entity token -> [(name, expr), ...]

def capture_source(src):
    """Read the source op's allowed parameter expressions ONCE — reused
    for every clone (API round-trips are the real cost, not compute)."""
    key = getattr(src, "entityToken", None) or id(src)
    if key in _SRC_CACHE:
        return _SRC_CACHE[key]
    sp = src.parameters
    out = []
    for i in range(sp.count):
        p = sp.item(i)
        try:
            out.append((p.name, p.expression))
        except Exception:
            pass
    _SRC_CACHE[key] = out
    return out


def transplant_parameters(src, dst, log):
    """Copy allowed parameter expressions src -> dst. Source cached across
    clones; destination mapped in ONE iteration (no per-name scans).
    Multi-pass writes resolve expression name-references."""
    dp = dst.parameters

    META_BLOCK = {"context", "strategy", "metric", "isAssemblyDocument",
                  "isOperationTemplate", "advancedMode", "betaMode",
                  "alphaMode", "isXpress", "undercut"}
    TOOL_ALLOW_PREFIXES = ("tool_feed", "tool_spindleSpeed",
                           "tool_rampSpindleSpeed", "tool_number",
                           "tool_coolant")

    def allowed(name):
        if name in META_BLOCK or name.startswith("license"):
            return False
        if name.startswith("tool_"):
            # tool_* derive from the assigned tool object — writing them
            # desyncs op state (v0.4 kernel wedge). Only feeds/speeds and
            # bookkeeping the op legitimately overrides.
            return name.startswith(TOOL_ALLOW_PREFIXES)
        if name.startswith(("view_", "group_")):
            return False
        return True

    t0 = time.time()
    # destination map: one iteration, cache param objects AND expressions
    dmap, dexpr = {}, {}
    for i in range(dp.count):
        p = dp.item(i)
        try:
            n = p.name
            dmap[n] = p
            dexpr[n] = p.expression
        except Exception:
            pass
    t_map = time.time() - t0

    pending = [(n, e) for n, e in capture_source(src)
               if allowed(n) and n in dmap and dexpr.get(n) != e]
    n_diff = len(pending)
    t1 = time.time()
    for _pass in range(3):
        failed = []
        for name, expr in pending:
            try:
                # fetch FRESH each write: Fusion invalidates cached param
                # proxies when the graph mutates (the 0-by-expression bug)
                tgt = dp.itemByName(name)
                if tgt is not None and tgt.expression != expr:
                    tgt.expression = expr
            except Exception:
                failed.append((name, expr))
        if not failed:
            break
        pending = failed
    t_write = time.time() - t1
    # geometry / CAD-object params can't move by expression — try by value
    cad_fail = []
    if pending:
        sp = src.parameters
        for name, _ in pending:
            try:
                dp.itemByName(name).value.value = \
                    sp.itemByName(name).value.value
            except Exception:
                cad_fail.append(name)
    log.append(f"  transplant: {n_diff} params differed from defaults; "
               f"{n_diff - len(pending)} by expression, "
               f"{len(pending) - len(cad_fail)} by value; "
               f"skipped: {cad_fail if cad_fail else 'none'}; "
               f"map {t_map:.1f}s write {t_write:.1f}s")


def resolve_param(cp, primary, must_contain, report, label):
    """Find a parameter by exact name, else by substring scan. Returns the
    resolved name or None (with a loud report line)."""
    if cp.itemByName(primary) is not None:
        return primary
    cands = []
    for i in range(cp.count):
        n = cp.item(i).name
        ln = n.lower()
        if all(k in ln for k in must_contain):
            cands.append(n)
    if cands:
        report.append(f"  ({label} param resolved to '{cands[0]}')")
        return cands[0]
    report.append(f"  !! {label} param not found — behavior WILL differ")
    return None


def duplicate_operation(setup, op, log):
    """Native op.duplicate() first (survey-confirmed on 2703.x): full
    fidelity, no transplant. Falls back to createInput + transplant."""
    survey_api(setup, op, log)
    global _NATIVE_BROKEN
    # op.duplicate() is DEFERRED: the copy materializes only after control
    # returns to Fusion's event loop. Pump doEvents and re-diff; if it
    # still doesn't appear, mark native broken and stop littering orphans.
    if not _NATIVE_BROKEN:
        try:
            def _tokens():
                s = set()
                for i in range(setup.allOperations.count):
                    o = setup.allOperations.item(i)
                    s.add(getattr(o, "entityToken", None) or o.name + str(i))
                return s
            before = _tokens()
            ret = op.duplicate()
            if ret is not None and hasattr(ret, "parameters"):
                log.append("  (native op.duplicate, by return)")
                return ret
            for _ in range(20):          # flush deferred creation
                adsk.doEvents()
                new_ops = []
                for i in range(setup.allOperations.count):
                    o = setup.allOperations.item(i)
                    tok = getattr(o, "entityToken", None) or o.name + str(i)
                    if tok not in before:
                        new_ops.append(o)
                if new_ops:
                    log.append("  (native op.duplicate, adopted after "
                               "doEvents)")
                    return new_ops[0]
            _NATIVE_BROKEN = True
            log.append("  (op.duplicate deferred copy never appeared — "
                       "native disabled for this run, falling back)")
        except Exception as e:
            _NATIVE_BROKEN = True
            log.append(f"  (op.duplicate failed: {e} — native disabled, "
                       f"falling back)")
    inp = setup.operations.createInput(op.strategy)
    try:
        inp.tool = op.tool
    except Exception:
        pass
    inp.displayName = f"AVM_tmp_{op.name}"
    new_op = setup.operations.add(inp)
    try:
        if new_op.tool is None:
            new_op.tool = op.tool
    except Exception:
        log.append(f"  !! tool assignment failed on clone of {op.name} — "
                   f"check clone tool matches T{op.parameters.itemByName('tool_number').value.value}")
    transplant_parameters(op, new_op, log)
    return new_op



# ======================= COMMAND DIALOG SCAFFOLD ========================
_handlers = []          # keep refs alive (Fusion GC eats handlers)
_captured_ops = []      # selection captured BEFORE the command clears it
_app = None
_ui = None

CMD_ID = "avmLevelCloneCmd"
MATERIAL_KEYS = ["DELRIN", "AL6061", "AL7075", "SS17-4", "TI6AL4V"]


def build_cfg_from_inputs(inputs):
    kc = inputs.itemById("kc11").value
    pw = inputs.itemById("power").value
    return {
        "material": inputs.itemById("material").selectedItem.name,
        "mode": ("baseline" if inputs.itemById("mode").selectedItem.name
                 .startswith("Baseline") else "absolute"),
        "defl_in": inputs.itemById("defl").value,     # internal cm
        "defl_sf": inputs.itemById("sf").value,
        "woc_cap": inputs.itemById("woc").value / 100.0,
        "core_factor": CORE_FACTOR,
        "kc11": kc if kc > 0 else None,
        "power_kw": pw if pw > 0 else None,
        "regen": inputs.itemById("regen").value,
        "force": inputs.itemById("force").value,
        "max_bands": max(2, int(inputs.itemById("maxbands").value)),
        "min_ae_ratio": 1.0 + inputs.itemById("mindelta").value / 100.0,
        "derate_pct": max(0.0, inputs.itemById("deratepct").value),
    }


class AVMCommandCreated(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.isRepeatable = False
            inputs = cmd.commandInputs

            dd = inputs.addDropDownCommandInput(
                "material", "Material",
                adsk.core.DropDownStyles.TextListDropDownStyle)
            for i, m in enumerate(MATERIAL_KEYS):
                dd.listItems.add(m, i == 0)

            md = inputs.addDropDownCommandInput(
                "mode", "Deflection mode",
                adsk.core.DropDownStyles.TextListDropDownStyle)
            md.listItems.add("Baseline (match proven cut, Kc-free)", True)
            md.listItems.add("Absolute (hard cap below)", False)

            inputs.addValueInput("defl", "Deflection cap (absolute mode)",
                                 "in", adsk.core.ValueInput.createByString(
                                     f"{DEFLECTION_LIMIT_IN} in"))
            inputs.addValueInput("sf", "Deflection safety factor", "",
                                 adsk.core.ValueInput.createByReal(
                                     DEFLECTION_SF))
            inputs.addValueInput("woc", "Max WOC (% of diameter)", "",
                                 adsk.core.ValueInput.createByReal(
                                     WOC_CAP_FRAC * 100.0))
            inputs.addValueInput("kc11", "Kc1.1 override (0 = material "
                                 "nominal)", "",
                                 adsk.core.ValueInput.createByReal(0.0))
            inputs.addValueInput("power", "Measured spindle power kW "
                                 "(0 = none)", "",
                                 adsk.core.ValueInput.createByReal(0.0))
            inputs.addBoolValueInput("regen", "Generate toolpaths now",
                                     True, "", False)
            inputs.addBoolValueInput("force", "Force run (skip opportunity "
                                     "check)", True, "", False)
            inputs.addValueInput("maxbands", "Max level clones", "",
                                 adsk.core.ValueInput.createByReal(12))
            inputs.addValueInput("deratepct", "RPM derate at max "
                                 "engagement (%)",
                                 "", adsk.core.ValueInput.createByReal(0.0))
            inputs.addValueInput("mindelta", "Merge levels within (% ae)",
                                 "",
                                 adsk.core.ValueInput.createByReal(15.0))
            inputs.addTextBoxCommandInput(
                "note", "",
                f"<b>{len(_captured_ops)} op(s) captured</b>: "
                + ", ".join(o.name for o in _captured_ops)
                + "<br>Baseline mode ignores Kc/deflection cap. "
                  "Run on a file copy."
                  "<br><b>Best on:</b> wavy/organic floors with fine "
                  "stepups (3D adaptive). <b>No benefit on:</b> straight-"
                  "wall constant-depth pockets — every pass is already at "
                  "max engagement; the tool will decline these unless "
                  "forced. Regen time ~= source op time x level count.",
                  7, True)

            on_exec = AVMCommandExecute()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
            on_destroy = AVMCommandDestroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            if _ui:
                _ui.messageBox("Dialog build failed:\n{}".format(
                    traceback.format_exc()))


class AVMCommandExecute(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            cfg = build_cfg_from_inputs(args.command.commandInputs)
            # ValueInput lengths arrive in internal cm -> inches
            cfg["defl_in"] = cfg["defl_in"] / IN

            all_lines = []
            total = 0
            for op in _captured_ops:
                total += process_op(_app, _ui, op, all_lines, cfg)
            finish_report(_app, _ui, all_lines, len(_captured_ops),
                          total, cfg)
        except Exception:
            if _ui:
                _ui.messageBox("AVM Cloner failed:\n{}".format(
                    traceback.format_exc()))


class AVMCommandDestroy(adsk.core.CommandEventHandler):
    def notify(self, args):
        adsk.terminate()


def finish_report(app, ui, all_lines, n_ops, total_clones, cfg):
    header = [f"config: mode={cfg['mode']} material={cfg['material']} "
              f"kc11={cfg['kc11']} measured_kw={cfg['power_kw']} "
              f"defl={cfg['defl_in']:.4f} sf={cfg['defl_sf']} "
              f"woc_cap={cfg['woc_cap']*100:.1f}% regen={cfg['regen']}", ""]
    all_lines = header + all_lines
    stamped = f"AVM_report_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    out, err = None, ""
    for base in [os.path.join(os.path.expanduser("~"), "Desktop"),
                 os.path.expanduser("~"),
                 os.environ.get("TEMP", "/tmp")]:
        try:
            latest = os.path.join(base, "AVM_report_latest.txt")
            with open(latest, "w", encoding="utf-8") as f:
                f.write("\n".join(all_lines))
            try:
                with open(os.path.join(base, stamped), "w",
                          encoding="utf-8") as f:
                    f.write("\n".join(all_lines))
            except Exception:
                pass
            out = latest
            break
        except Exception as e:
            err = str(e)
    if out is None:
        out = f"(write failed: {err} — see Text Commands palette)"
    try:
        pal = ui.palettes.itemById("TextCommands")
        if pal:
            pal.isVisible = True
        for line in all_lines:
            app.log(line)
    except Exception:
        pass
    ui.messageBox(
        f"[{cfg['mode']} mode, {cfg['material']}, kc11={cfg['kc11']}]\n"
        f"Processed {n_ops} op(s), created {total_clones} level clones.\n"
        f"SOURCE OP(S) NOW SUPPRESSED (clones rest-machine).\n\n"
        f"Regen the DEEPEST clone first, then generate the rest.\n"
        f"A/B: clones active + source suppressed = optimized; flip = "
        f"baseline.\nReport: {out}")


def run(context):
    global _app, _ui, _captured_ops
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        # capture selection NOW — command activation clears it
        _captured_ops = []
        sel = _ui.activeSelections
        for i in range(sel.count):
            e = sel.item(i).entity
            if hasattr(e, "parameters") and hasattr(e, "tool"):
                if not e.name.startswith(PREFIX):
                    _captured_ops.append(e)
        if not _captured_ops:
            _ui.messageBox("Select one or more adaptive operations first "
                           "(AVM clones are skipped).")
            adsk.terminate()
            return

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, "AVM Level Clone", "Per-level adaptive optimization")
        on_created = AVMCommandCreated()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)
        cmd_def.execute()
        adsk.autoTerminate(False)   # keep script alive for the dialog
    except Exception:
        if _ui:
            _ui.messageBox("AVM startup failed:\n{}".format(
                traceback.format_exc()))
        adsk.terminate()


def process_op(app, ui, op, all_lines, cfg):

        setup = op.parentSetup
        cam_obj = adsk.cam.CAM.cast(setup.parent if hasattr(setup, 'parent')
                                    else app.activeProduct)
        tag = f"{PREFIX}:{op.name}]"

        # refuse to touch the tree while toolpaths are generating —
        # deleting/creating ops mid-queue is a kernel-wedge vector
        try:
            for attr in ("isToolpathGenerating", "isGenerating",
                         "isToolpathOperationsGenerating"):
                if getattr(cam_obj, attr, False):
                    raise RuntimeError(
                        "Toolpath generation in progress — let it finish "
                        "(or cancel it), then rerun. Nothing was changed.")
        except RuntimeError:
            raise
        except Exception:
            pass  # attribute probing only; absence is fine

        # delete stale clones from previous runs on this op
        stale = []
        for i in range(setup.allOperations.count):
            o = setup.allOperations.item(i)
            if o.name.startswith(tag):
                stale.append(o)
        for o in stale:
            try:
                o.deleteMe()
            except Exception:
                pass

        # snapshot for orphan cleanup at the end of this op's processing
        pre_run = set()
        for i in range(setup.allOperations.count):
            o = setup.allOperations.item(i)
            pre_run.add(getattr(o, "entityToken", None) or o.name)

        P = op.parameters
        max_step = p_in(P, "maximumStepdown")
        fine_step = p_in(P, "fineStepdown")
        ae0 = p_in(P, "optimalLoad")
        D = p_in(P, "tool_diameter")
        loc = p_in(P, "tool_fluteLength")
        stickout = p_in(P, "tool_bodyLength")     # verified vs calipers
        flutes = int(p_num(P, "tool_numberOfFlutes"))
        rpm = p_num(P, "tool_spindleSpeed")
        # fz read directly (consistent cm units) — do NOT derive from
        # tool_feedCutting: feed params are stored in mm/min, not cm.
        fz0 = p_in(P, "tool_feedPerTooth")
        if not (0.0002 < fz0 < 0.030):
            raise RuntimeError(
                f"SANITY ABORT: fz0={fz0:.5f} in/tooth is not a plausible "
                f"chipload — unit read error, nothing was created.")
        if not (0.02 < D < 2.0) or not (0.2 < stickout < 8.0):
            raise RuntimeError(
                f"SANITY ABORT: D={D:.4f} stickout={stickout:.3f} "
                f"implausible — unit read error, nothing was created.")

        # total depth: stock top to op bottom
        top_v = p_num(P, "topHeight_value") / IN
        bot_v = p_num(P, "bottomHeight_value") / IN
        total_depth = abs(top_v - bot_v)

        depths = compute_levels(max_step, fine_step, total_depth)

        # OPPORTUNITY CHECK: the solver's gains come from ENGAGEMENT
        # VARIANCE across levels (wavy floors, fine stepups). A straight-
        # wall multi-stepdown pocket has none: every pass cuts one
        # increment of wall, worst-case == every-case, and the source op's
        # constant load is already correct. Worse, the virgin-column ap
        # model misreads such ops entirely. Detect and decline.
        if len(depths) > 200:
            all_lines.append(f"=== {op.name}: DECLINED — {len(depths)} "
                             f"theoretical levels (fine stepdown "
                             f"{fine_step:.4f} over {total_depth:.3f} "
                             f"depth). Grid is parameter fiction; fix the "
                             f"fine stepdown or wait for toolpath-derived "
                             f"levels. ===\n")
            return 0
        # probe: true levels live in the generated toolpath, not params
        try:
            gdc = getattr(op, "generatedDataCollection", None)
            if gdc is not None:
                report_probe = [m for m in dir(gdc)
                                if not m.startswith("_")]
                all_lines.append(f"[api survey] op.generatedDataCollection"
                                 f" [FULL]: {report_probe}")
        except Exception:
            pass
        n_mains = max(1, int(total_depth / max_step + 1e-6))
        n_fine = len(depths) - n_mains
        slab_dominated = (n_mains >= 2 and n_fine < n_mains)
        if slab_dominated and not cfg.get("force"):
            all_lines.append(
                f"=== {op.name}: DECLINED — {n_mains} full stepdowns vs "
                f"{n_fine} fine levels. Straight multi-stepdown structure: "
                f"engagement is constant across passes, no per-level "
                f"opportunity exists (the source op's constant load is "
                f"already right for this geometry). Use 'Force run' to "
                f"override. ===")
            all_lines.append("")
            return 0
        if slab_dominated:
            all_lines.append(
                f"!! {op.name}: slab-dominated structure FORCED — ap model "
                f"assumes virgin columns; results here are conservative "
                f"nonsense on straight walls. You were warned.")
        ap_levels = [(i + 1, d if i == 0 else depths[i]) for i, d in
                     enumerate(depths)]
        # ap of a level on virgin wavy stock = its depth below stock top
        ap_levels = [(i + 1, depths[i]) for i in range(len(depths))]

        geom = CutGeometry(D, flutes, helix_deg=38.0)
        beam = standard_endmill(D, stickout, loc, core_factor=cfg['core_factor'])
        mat = MATERIALS[cfg['material']]
        limits = Limits(deflection_in=cfg['defl_in'],
                        deflection_sf=cfg['defl_sf'],
                        woc_cap_frac=cfg['woc_cap'])
        results = solve_levels(geom, beam, mat, rpm, fz0, ae0, ap_levels,
                               limits, kc11_override=cfg['kc11'],
                               measured_power_kw=cfg['power_kw'],
                               deflection_mode=cfg['mode'])

        # --- create clones, deep-first so rest machining links levels ---
        report = [f"=== source op: {op.name} (setup: {setup.name}, "
                  f"{len(stale)} stale clones removed) ===",
                  f"tool D={D:.4f} loc={loc:.3f} stickout={stickout:.3f} "
                  f"z={flutes} rpm={rpm:.0f} fz0={fz0:.5f} ae0={ae0:.4f}",
                  f"material={cfg['material']} kc11_override={cfg['kc11']} "
                  f"measured_kw={cfg['power_kw']}",
                  f"levels={len(results)} (deep-first)", ""]
        made = []
        # attempt design-side compute deferral (may or may not gate CAM
        # parameter cascades — timing in report tells us)
        design = None
        try:
            design = adsk.fusion.Design.cast(
                app.activeDocument.products.itemByProductType("DesignProductType"))
            if design:
                design.isComputeDeferred = True
        except Exception:
            design = None

        # CREATION ORDER: duplicate() inserts each copy immediately AFTER
        # the source (empirical), so we create in REVERSE — sweep first,
        # then shallowest->deepest — and the tree lands deep-first with
        # the sweep at the bottom. Tree order = execution order.
        derate_targets = []   # (clone, woc, current_feed) for RPM derate

        # SWEEP clone: FAD on, min axial 0, rest machining — catches the
        # floor slivers the FAD-off level clones leave between grid levels.
        # Tiny-ap leftovers are the ideal wide-load cut: use the
        # shallowest level's solved ae/fz.
        try:
            shallow = results[-1]
            sweep = duplicate_operation(setup, op, report)
            scp = sweep.parameters
            set_expr(scp, "optimalLoad", f"{shallow.ae_in:.4f} in")
            ma = resolve_param(scp, "minimumAxialEngagement",
                               ("axial", "engag"), report, "min axial")
            if ma:
                set_expr(scp, ma, "0 in")
            new_fz = shallow.fz_in
            if abs(new_fz - fz0) > 1e-6:
                set_expr(scp, "tool_feedCutting",
                         f"{new_fz * rpm * flutes:.1f} in/min")
            sweep.name = f"{tag} SWEEP ae{shallow.ae_in:.3f} (floor parity)"
            made.append(sweep)
            derate_targets.append((sweep, shallow.ae_in,
                                   shallow.fz_in * rpm * flutes))
            report.append(f"SWEEP: ae={shallow.ae_in:.4f} "
                          f"fz={shallow.fz_in:.5f} FAD=on minAxial=0")
        except Exception as e:
            report.append(f"!! sweep clone failed: {e}")

        bands = band_levels(results, cfg.get("max_bands", 12),
                            cfg.get("min_ae_ratio", 1.15))
        report.append(f"banding: {len(results)} levels -> {len(bands)} "
                      f"clones (max {cfg.get('max_bands', 12)}, merge "
                      f"within {(cfg.get('min_ae_ratio', 1.15)-1)*100:.0f}%)")

        # NO-OP DETECTION: one band at ~the baseline load means the source
        # op is already at its limit under rule #5 for this level
        # structure. Create nothing — including no sweep — and say so.
        if (len(bands) == 1
                and abs(bands[0]["ae"] - ae0) / max(ae0, 1e-9) < 0.05
                and abs(bands[0]["fz"] - fz0) / max(fz0, 1e-9) < 0.05):
            # undo the sweep clone already created above
            for m in made:
                try:
                    m.deleteMe()
                except Exception:
                    pass
            report.append(
                "NO OPPORTUNITY: solver holds every level at the baseline "
                "load — the source op is already optimal under rule #5 "
                "for this level structure. Nothing created, source op "
                "untouched.")
            report.append("")
            all_lines.extend(report)
            return 0

        class _B:            # adapter so the clone loop stays unchanged
            def __init__(self, d, i):
                self.level = i + 1
                self.ap_in = d["ap_deep"]
                self.ae_in = d["ae"]
                self.fz_in = d["fz"]
                self.binding = d["binding"]
                self.thresh = d["thresh"]
                self.span = (d["level_lo"], d["level_hi"])
        band_rs = [_B(d, i) for i, d in enumerate(bands)]

        for r in reversed(band_rs):
            t_op = time.time()
            clone = duplicate_operation(setup, op, report)
            cp = clone.parameters
            set_expr(cp, "optimalLoad", f"{r.ae_in:.4f} in")
            # bottom band: from stock top, down r.ap_in
            set_expr(cp, "bottomHeight_mode", "'from stock top'")
            set_expr(cp, "bottomHeight_offset", f"{-r.ap_in:.4f} in")
            # V2 fix (Tom): minimum axial engagement = wall-height filter.
            # Threshold midway to the next-shallower level: this clone's
            # full columns pass, shallower regions' floor passes are culled.
            thresh = r.thresh     # band boundary (0 = shallowest)
            name = resolve_param(cp, "minimumAxialEngagement",
                                 ("axial", "engag"), report, "min axial")
            if name:
                set_expr(cp, name, f"{thresh:.4f} in")
            # min axial engagement is only honored with flat-area
            # detection OFF (empirical — FAD floor passes bypass the
            # engagement filter). Sweep clone below restores floor parity.
            fad = resolve_param(cp, "flatAreaMachining",
                                ("flatarea",), report, "flat area detect")
            if fad:
                set_expr(cp, fad, "false")
            new_fz = r.fz_in
            if abs(new_fz - fz0) > 1e-6:
                set_expr(cp, "tool_feedCutting",
                         f"{new_fz * rpm * flutes:.1f} in/min")
            span = (f"L{r.span[0]}" if r.span[0] == r.span[1]
                    else f"L{r.span[0]}-{r.span[1]}")
            clone.name = (f"{tag} B{r.level} {span} z-{r.ap_in:.3f} "
                          f"ae{r.ae_in:.3f} ({r.binding})")
            derate_targets.append((clone, r.ae_in,
                                   r.fz_in * rpm * flutes))
            made.append(clone)
            report.append(f"  L{r.level} clone total {time.time()-t_op:.1f}s")
            report.append(
                f"B{r.level} (L{r.span[0]}-{r.span[1]}): ap={r.ap_in:.3f} "
                f"ae={r.ae_in:.4f} "
                f"fz={r.fz_in:.5f} feed={r.fz_in*rpm*flutes:.0f} "
                f"-> {r.binding}")


        try:
            if design:
                design.isComputeDeferred = False
        except Exception:
            pass

        # CRITICAL: suppress source BEFORE regen — clones rest-machine and
        # sit after the source in the tree; active source = empty clones.
        try:
            op.isSuppressed = True
        except Exception:
            report.append("!! could not suppress source op — suppress it "
                          "manually BEFORE regenerating or clones are empty")

        if cfg['regen']:
            for c in reversed(made):
                try:
                    cam_obj.generateToolpath(c)
                except Exception as e:
                    report.append(f"!! regen failed on {c.name}: {e}")

        # RPM DERATE post-pass (thermal, forum-requested). Dormant at 0%.
        # Anchors: source op's proven WOC -> 0%; widest clone -> entered %.
        # Interpolates on duty factor. feed_scale applies to each clone's
        # OWN feed, preserving solver fz raises (L10/SWEEP). Pure
        # post-pass: touches only spindle speed + cutting feed.
        if cfg.get("derate_pct", 0.0) > 0.0 and derate_targets:
            ders = solve_derates([w for _, w, _ in derate_targets], D,
                                 ae0, rpm, cfg["derate_pct"])
            report.append(f"derate: {cfg['derate_pct']:.0f}% at widest "
                          f"(ae {max(w for _, w, _ in derate_targets):.4f}), "
                          f"anchored to ref ae {ae0:.4f} @ {rpm:.0f} rpm")
            for (cl, w, feed_now), res in zip(derate_targets, ders):
                if res.derate_frac <= 0.0:
                    continue           # reference-and-narrower: untouched
                dp = cl.parameters     # fresh fetch — proxy staleness
                set_expr(dp, "tool_spindleSpeed", f"{res.rpm}")
                set_expr(dp, "tool_feedCutting",
                         f"{feed_now * res.feed_scale:.1f} in/min")
                report.append(f"  {cl.name.split('] ')[-1]}: "
                              f"-{100*res.derate_frac:.1f}% -> "
                              f"{res.rpm} rpm, "
                              f"{feed_now * res.feed_scale:.0f} ipm "
                              f"(fz held)")

        # ORDER VERIFICATION: read back actual tree positions of our ops
        try:
            seq = []
            made_toks = {(getattr(m, "entityToken", None) or m.name): m.name
                         for m in made}
            for i in range(setup.allOperations.count):
                o = setup.allOperations.item(i)
                tok = getattr(o, "entityToken", None) or o.name
                if tok in made_toks:
                    seq.append(o.name.split("] ")[-1])
            report.append(f"  tree order: {' > '.join(seq)}")
            lvl_idx = [s for s in seq if s.startswith("B")]
            expect = sorted(lvl_idx, key=lambda s: int(s[1:].split()[0]))
            if lvl_idx != expect:
                report.append("  !! ORDER WRONG — execution will not be "
                              "deep-first; do not run this program")
        except Exception as e:
            report.append(f"  (order check failed: {e})")

        # orphan cleanup: anything new, unadopted, named like a raw copy
        try:
            adsk.doEvents()
            made_toks = set()
            for m in made:
                made_toks.add(getattr(m, "entityToken", None) or m.name)
            killed = 0
            for i in range(setup.allOperations.count - 1, -1, -1):
                o = setup.allOperations.item(i)
                tok = getattr(o, "entityToken", None) or o.name
                if tok in pre_run or tok in made_toks:
                    continue
                nm = o.name
                if nm.startswith(op.name) and "(" in nm[len(op.name):]:
                    try:
                        o.deleteMe()
                        killed += 1
                    except Exception:
                        pass
            if killed:
                report.append(f"  (cleaned {killed} orphan raw copies)")
        except Exception:
            pass

        # folder organization: addFolder confirmed by survey
        try:
            folders = getattr(setup, "folders", None)
            folder = None
            if folders is not None and hasattr(folders, "addFolder"):
                for maker in (lambda: folders.addFolder(f"{tag} levels"),
                              lambda: folders.addFolder()):
                    try:
                        folder = maker()
                        break
                    except Exception:
                        continue
            if folder is None:
                report.append("  (folder: no working addFolder signature)")
            else:
                try:
                    fdir = [m for m in dir(folder) if not m.startswith("_")]
                    report.append(f"  [api survey] CAMFolder [FULL]: {fdir}")
                except Exception:
                    pass
                try:
                    folder.name = f"{tag} levels"
                except Exception:
                    pass
                moved = 0
                # move DEEP-FIRST: moveInto appends, and made[] is in
                # creation order (sweep, L10..L1) — reversed = L1..sweep
                for m in reversed(made):
                    for mover in (lambda o=m: o.moveInto(folder),
                                  lambda o=m: folder.children.add(o),
                                  lambda o=m: folder.operations.add(o)):
                        try:
                            mover()
                            moved += 1
                            break
                        except Exception:
                            continue
                if moved == 0:
                    report.append("  (folder: created but no move method "
                                  "worked — deleting empty folder)")
                    try:
                        folder.deleteMe()
                    except Exception:
                        pass
                else:
                    # position folder directly after the (suppressed)
                    # source — correct rest-machining order vs downstream
                    try:
                        folder.moveAfter(op)
                        report.append(f"  (folder: {moved}/{len(made)} "
                                      f"moved, placed after source)")
                    except Exception as e:
                        report.append(f"  (folder: {moved}/{len(made)} "
                                      f"moved; moveAfter failed: {e} — "
                                      f"folder sits at setup end, drag it "
                                      f"below {op.name} manually)")
                    # verify order INSIDE the folder
                    try:
                        fseq = []
                        fops = folder.allOperations
                        for i in range(fops.count):
                            fseq.append(fops.item(i).name.split("] ")[-1])
                        report.append(f"  folder order: "
                                      f"{' > '.join(fseq)}")
                        lv = [s for s in fseq if s.startswith("B")]
                        if lv != sorted(lv, key=lambda s:
                                        int(s[1:].split()[0])):
                            report.append("  !! FOLDER ORDER WRONG — do "
                                          "not run; drag to L1..L10..SWEEP")
                    except Exception as e:
                        report.append(f"  (folder order check failed: {e})")
        except Exception as e:
            report.append(f"  (folder attempt: {e})")

        report.append("")
        all_lines.extend(report)
        return len(made)


def stop(context):
    pass
