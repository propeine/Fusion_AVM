"""AVM Prune — delete AVM clones whose generated toolpath is empty.

Run as a Fusion Script AFTER regenerating an AVM stack. Select the AVM
folder, any ops inside it, or nothing (scans all [AVM: ops in the doc).

Detection is defensive: an op is a PRUNE CANDIDATE only if it is an AVM
clone AND its toolpath evidence says empty. Evidence checked, in order:
  1. generatedDataCollection.count == 0 (probe-confirmed attribute)
  2. op.hasToolpath is False (if the attribute exists)
Ops that are ungenerated (never had regen run) are SKIPPED, not pruned —
no toolpath yet is not the same as an empty toolpath.

Report-first: shows the list and asks OK/Cancel before deleting anything.
Never touches non-AVM operations. Never touches the SWEEP clone.
"""

import adsk.core, adsk.fusion, adsk.cam
import traceback

TAG = "[AVM"


def toolpath_state(op):
    """Returns 'empty', 'has_toolpath', or 'ungenerated'."""
    generated = None
    try:
        st = op.operationState
        # OperationStates enum: guard by name to avoid magic numbers
        generated = "IsValid" in str(st) or st in (2, 3)
    except Exception:
        pass
    try:
        if hasattr(op, "hasToolpath"):
            if op.hasToolpath:
                return "has_toolpath"
            return "empty" if generated else "ungenerated"
    except Exception:
        pass
    try:
        gdc = getattr(op, "generatedDataCollection", None)
        if gdc is not None:
            if gdc.count > 0:
                return "has_toolpath"
            return "empty" if generated else "ungenerated"
    except Exception:
        pass
    return "ungenerated"


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        cam = None
        for i in range(app.activeDocument.products.count):
            p = app.activeDocument.products.item(i)
            if p.productType == "CAMProductType":
                cam = adsk.cam.CAM.cast(p)
                break
        if cam is None:
            ui.messageBox("No CAM data in this document.")
            return

        # collect AVM ops: from selection if any, else the whole document
        ops = []
        sel = ui.activeSelections
        picked = []
        for i in range(sel.count):
            e = sel.item(i).entity
            picked.append(e)
        if picked:
            for e in picked:
                if hasattr(e, "allOperations"):        # folder or setup
                    for j in range(e.allOperations.count):
                        ops.append(e.allOperations.item(j))
                elif hasattr(e, "parameters"):
                    ops.append(e)
        else:
            for s in range(cam.setups.count):
                setup = cam.setups.item(s)
                for j in range(setup.allOperations.count):
                    ops.append(setup.allOperations.item(j))

        empties, kept, skipped = [], [], []
        for o in ops:
            if not o.name.startswith(TAG):
                continue
            if "SWEEP" in o.name:
                kept.append(o.name)
                continue
            state = toolpath_state(o)
            if state == "empty":
                empties.append(o)
            elif state == "ungenerated":
                skipped.append(o.name)
            else:
                kept.append(o.name)

        if not empties:
            ui.messageBox(f"No empty AVM clones found.\n"
                          f"With toolpath: {len(kept)}  "
                          f"Ungenerated (skipped): {len(skipped)}")
            return

        msg = (f"Empty AVM clones found ({len(empties)}):\n\n"
               + "\n".join(o.name for o in empties)
               + f"\n\nWith toolpath: {len(kept)}   "
               f"Ungenerated (skipped): {len(skipped)}"
               "\n\nDelete the empty clones?")
        result = ui.messageBox(msg, "AVM Prune",
                               adsk.core.MessageBoxButtonTypes
                               .OKCancelButtonType)
        if result != adsk.core.DialogResults.DialogOK:
            return
        killed = 0
        for o in empties:
            try:
                o.deleteMe()
                killed += 1
            except Exception:
                pass
        ui.messageBox(f"Deleted {killed} empty clones.")

    except Exception:
        if ui:
            ui.messageBox("AVM Prune failed:\n{}".format(
                traceback.format_exc()))


def stop(context):
    pass
