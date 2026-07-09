"""AVM Probe — dump CAM operation parameters + tool geometry to a text file.

Run as a Fusion 360 Script (Utilities > Add-Ins > Scripts tab > add this folder).

Purpose: learn the exact parameter names/expressions Fusion uses for an
adaptive clearing operation (optimal load, stepdowns, heights) and verify
what the tool library / tool assembly gives us (diameter, flutes, LOC,
shank, stickout/projection), before writing the clone-and-regen machinery.

Output: AVM_probe_<opname>_<timestamp>.txt on the Desktop (falls back to
home dir), plus a summary in the Text Commands palette.

Usage: select a CAM operation in the browser first (or it will probe every
operation in every setup if nothing is selected).
"""

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import os
import time

app = None
ui = None


def out_path(op_name: str) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in op_name)
    desktop = os.path.join(os.path.expanduser("~"), "Desktop")
    base = desktop if os.path.isdir(desktop) else os.path.expanduser("~")
    return os.path.join(base, f"AVM_probe_{safe}_{stamp}.txt")


def fmt(v) -> str:
    try:
        return repr(v)
    except Exception:
        return "<unprintable>"


def dump_parameters(op, lines):
    lines.append("")
    lines.append("=" * 78)
    lines.append(f"OPERATION: {op.name}")
    lines.append(f"  strategy       : {getattr(op, 'strategy', '<n/a>')}")
    lines.append(f"  operationState : {getattr(op, 'operationState', '<n/a>')}")
    lines.append(f"  isValid        : {getattr(op, 'isValid', '<n/a>')}")
    lines.append("=" * 78)

    try:
        params = op.parameters
    except Exception as e:
        lines.append(f"  !! could not read op.parameters: {e}")
        return

    lines.append(f"  parameter count: {params.count}")
    lines.append("")
    header = f"  {'NAME':<42} {'EXPRESSION':<28} VALUE"
    lines.append(header)
    lines.append("  " + "-" * (len(header) - 2))

    for i in range(params.count):
        try:
            p = params.item(i)
            name = p.name
        except Exception as e:
            lines.append(f"  [param {i}] <error reading param object: {e}>")
            continue

        # expression (string form, units included)
        try:
            expr = p.expression
        except Exception as e:
            expr = f"<err: {e}>"

        # value — parameter values are typed (CAMParameterValue subclasses)
        val_str = "<n/a>"
        try:
            pv = p.value
            cls = pv.classType().split("::")[-1] if hasattr(pv, "classType") else type(pv).__name__
            try:
                val_str = f"{cls}({fmt(pv.value)})"
            except Exception:
                val_str = f"{cls}(<no .value>)"
        except Exception as e:
            val_str = f"<err: {e}>"

        lines.append(f"  {name:<42} {fmt(expr):<28} {val_str}")

    # Flag the parameters we care about most so they're easy to grep
    lines.append("")
    lines.append("  --- AVM parameters of interest (name contains...) ---")
    keys = ("load", "stepdown", "stepup", "stock", "height", "top", "bottom",
            "feed", "speed", "tolerance", "engagement", "radius")
    for i in range(params.count):
        try:
            p = params.item(i)
            lname = p.name.lower()
            if any(k in lname for k in keys):
                lines.append(f"  * {p.name:<40} = {fmt(p.expression)}")
        except Exception:
            pass


def dump_tool(op, lines):
    lines.append("")
    lines.append("  --- TOOL ---")
    try:
        tool = op.tool
    except Exception as e:
        lines.append(f"  !! could not read op.tool: {e}")
        return
    if tool is None:
        lines.append("  (no tool on this operation)")
        return

    # Tool parameters collection (diameter, flutes, LOC, shoulder, body, etc.)
    try:
        tparams = tool.parameters
        lines.append(f"  tool parameter count: {tparams.count}")
        interesting = ("diameter", "flute", "length", "shoulder", "body",
                       "shaft", "holder", "overall", "type", "description",
                       "product", "vendor", "material")
        for i in range(tparams.count):
            try:
                tp = tparams.item(i)
                lname = tp.name.lower()
                if any(k in lname for k in interesting):
                    try:
                        expr = tp.expression
                    except Exception:
                        expr = "<n/a>"
                    lines.append(f"  {tp.name:<42} {fmt(expr)}")
            except Exception:
                pass
    except Exception as e:
        lines.append(f"  !! could not read tool.parameters: {e}")

    # Full dump too — tool param names vary and we want the real list once
    lines.append("")
    lines.append("  --- TOOL (full parameter dump) ---")
    try:
        tparams = tool.parameters
        for i in range(tparams.count):
            try:
                tp = tparams.item(i)
                try:
                    expr = tp.expression
                except Exception:
                    expr = "<n/a>"
                lines.append(f"  {tp.name:<42} {fmt(expr)}")
            except Exception:
                lines.append(f"  [tool param {i}] <unreadable>")
    except Exception:
        pass


def dump_holder_and_assembly(op, lines):
    """Stickout / projection lives on the operation's tool-holder relationship.
    Where exactly varies by API version — probe every plausible location and
    report what exists so we learn the real one."""
    lines.append("")
    lines.append("  --- HOLDER / ASSEMBLY / STICKOUT probes ---")
    candidates = []
    try:
        tool = op.tool
    except Exception:
        tool = None

    probes = [
        ("op.toolPreset", lambda: op.toolPreset),
        ("tool.holder", lambda: tool.holder if tool else None),
        ("op.parameters['tool_stickout']", lambda: op.parameters.itemByName("tool_stickout")),
        ("op.parameters['tool_holderStickout']", lambda: op.parameters.itemByName("tool_holderStickout")),
        ("op.parameters['holder_attachPoint']", lambda: op.parameters.itemByName("holder_attachPoint")),
    ]
    for label, fn in probes:
        try:
            v = fn()
            if v is None:
                candidates.append(f"  {label}: None")
            else:
                extra = ""
                for attr in ("expression", "name", "value"):
                    try:
                        extra += f" .{attr}={fmt(getattr(v, attr))}"
                    except Exception:
                        pass
                candidates.append(f"  {label}: EXISTS ({type(v).__name__}){extra}")
        except Exception as e:
            candidates.append(f"  {label}: <err: {e}>")
    lines.extend(candidates)


def collect_ops(cam_product, ui):
    """Selected operation(s) if any, else everything."""
    ops = []
    try:
        sel = ui.activeSelections
        for i in range(sel.count):
            ent = sel.item(i).entity
            if hasattr(ent, "parameters") and hasattr(ent, "tool"):
                ops.append(ent)
    except Exception:
        pass
    if ops:
        return ops, True

    for s in range(cam_product.setups.count):
        setup = cam_product.setups.item(s)
        for o in range(setup.allOperations.count):
            ops.append(setup.allOperations.item(o))
    return ops, False


def run(context):
    global app, ui
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        cam_product = None
        for i in range(app.activeDocument.products.count):
            p = app.activeDocument.products.item(i)
            if p.productType == "CAMProductType":
                cam_product = adsk.cam.CAM.cast(p)
                break
        if cam_product is None:
            ui.messageBox("No CAM (Manufacture) data in this document.")
            return

        ops, was_selection = collect_ops(cam_product, ui)
        if not ops:
            ui.messageBox("No CAM operations found.")
            return

        lines = []
        lines.append("AVM PROBE — Fusion CAM parameter dump")
        lines.append(f"document : {app.activeDocument.name}")
        lines.append(f"fusion   : {app.version}")
        lines.append(f"selection: {'selected op(s) only' if was_selection else 'ALL operations'}")
        lines.append(f"time     : {time.strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("NOTE: API length values are internal units (cm). Expressions show real units.")

        for op in ops:
            dump_parameters(op, lines)
            dump_tool(op, lines)
            dump_holder_and_assembly(op, lines)

        path = out_path(ops[0].name if was_selection else "all_ops")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        # Also echo the greppable summary to the text palette
        palette = ui.palettes.itemById("TextCommands")
        if palette:
            palette.isVisible = True
        app.log(f"AVM probe written: {path}")
        ui.messageBox(f"Probe complete.\n\nWrote:\n{path}\n\n"
                      f"Operations probed: {len(ops)}")

    except Exception:
        if ui:
            ui.messageBox("AVM Probe failed:\n{}".format(traceback.format_exc()))


def stop(context):
    pass
