"""
derate — per-level RPM/feed thermal derating for AVM level clones.

Forum-requested (Practical Machinist): heat-limited materials (Ti,
stainless, Inconel) want lower surface speed as radial engagement grows.
AVM knows every clone's solved WOC, so it derates RPM deterministically.

Physics
-------
Arc of engagement for radial width of cut `ae` on cutter diameter `D`:

    phi = acos(1 - 2*ae/D)      # radians; ae=D/2 -> 90 deg, ae=D -> 180

Per-tooth duty factor (fraction of one revolution spent in the cut):

    df = phi / (2*pi)

Derating is linear in duty factor between two anchors the user owns:

    reference op WOC  -> 0 % derate   (their proven speed)
    widest clone WOC  -> user %       (dialog value)

Clones between the anchors interpolate; clones at or below the reference
WOC clamp to 0 % (never speed up past the proven number).

Feed handling — IMPORTANT INTEGRATION CONTRACT
----------------------------------------------
The module returns RPM plus a `feed_scale` (rounded_rpm / ref_rpm). The
caller multiplies EACH CLONE'S OWN current feed by its scale. This holds
each clone's own feed-per-tooth exactly constant — including clones whose
fz the WOC solver deliberately RAISED (capped levels, SWEEP). Returning
an absolute feed derived from the reference would clobber those raises.

Constant per-clone fz => constant force per tooth => the WOC solver's
force and deflection branches are unaffected. Pure post-pass: touches
only spindle speed and cutting feed, never geometry/heights/optimalLoad.

Units: unit-agnostic. WOC and D must share a unit; RPM is rev/min;
feed_scale is dimensionless and applies to feed in any unit.

This module does NOT model heat. It applies the user's own published-
chart derate automatically at every level; interpolation basis (duty
factor) is stated, nothing more is claimed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "engagement_angle",
    "duty_factor",
    "round_rpm",
    "LevelDerate",
    "solve_derates",
]


def engagement_angle(woc: float, diameter: float) -> float:
    """Arc of engagement in radians for radial width `woc` on `diameter`.

    woc is clamped to (0, diameter]. Raises ValueError on non-positive
    inputs.
    """
    if diameter <= 0.0:
        raise ValueError(f"diameter must be > 0, got {diameter}")
    if woc <= 0.0:
        raise ValueError(f"woc must be > 0, got {woc}")
    ratio = min(woc, diameter) / diameter
    return math.acos(1.0 - 2.0 * ratio)


def duty_factor(woc: float, diameter: float) -> float:
    """Fraction of one revolution a tooth spends in the cut (0, 0.5]."""
    return engagement_angle(woc, diameter) / (2.0 * math.pi)


def round_rpm(rpm: float, step: int = 10) -> int:
    """Round RPM to the nearest `step` (default 10), minimum one step.

    Ties round half-to-even (Python round()): 8005 -> 8000 at step 10.
    """
    if step <= 0:
        raise ValueError(f"step must be > 0, got {step}")
    return max(step, int(round(rpm / step)) * step)


@dataclass(frozen=True)
class LevelDerate:
    """Solved derate for one level clone."""

    woc: float           # radial width of cut for this level (echoed)
    derate_frac: float   # applied derate, 0.0 .. derate_pct/100
    rpm: int             # rounded spindle speed to program
    feed_scale: float    # rounded_rpm / ref_rpm — multiply the clone's
                         # OWN current feed by this to hold its own fz


def solve_derates(
    level_wocs: list,
    diameter: float,
    ref_woc: float,
    ref_rpm: float,
    derate_pct: float,
    rpm_step: int = 10,
):
    """Compute per-level RPM and feed scale for a list of clone WOCs.

    Anchors:
      ref_woc  (the reference op's proven engagement) -> 0 % derate
      max(level_wocs)                                 -> derate_pct

    Levels between anchors interpolate linearly in duty factor.
    Levels at or below ref_woc clamp to the reference RPM.

    Returns one LevelDerate per entry, same order. derate_pct == 0 (or
    all WOCs <= ref_woc) returns reference numbers unchanged except RPM
    rounding — the feature is dormant.
    """
    if not level_wocs:
        return []
    if not (0.0 <= derate_pct < 100.0):
        raise ValueError(f"derate_pct must be in [0, 100), got {derate_pct}")
    if ref_rpm <= 0.0:
        raise ValueError(f"ref_rpm must be > 0, got {ref_rpm}")

    df_ref = duty_factor(ref_woc, diameter)
    df_max = max(duty_factor(w, diameter) for w in level_wocs)
    df_span = df_max - df_ref

    out = []
    for woc in level_wocs:
        if derate_pct == 0.0 or df_span <= 1e-12:
            frac = 0.0
        else:
            t = (duty_factor(woc, diameter) - df_ref) / df_span
            t = min(max(t, 0.0), 1.0)   # clamp: never past anchor or ref
            frac = (derate_pct / 100.0) * t

        rpm = round_rpm(ref_rpm * (1.0 - frac), rpm_step)
        out.append(LevelDerate(woc=woc, derate_frac=frac, rpm=rpm,
                               feed_scale=rpm / ref_rpm))
    return out
