"""Segmented cantilever tip-deflection model.

Tool modeled as a stepped beam, fixed at the holder face, tip free.
Segments are listed FROM THE TIP: [(length, diameter), ...].
Cutting load is applied as a point load at height `load_height` above the
tip (force centroid ~ ap/2 for an engaged flute section).

Tip deflection under a load NOT at the tip, on a stepped beam, via the
unit-load (virtual work) method:

    delta_tip = integral over 0..b of  M(x) * m(x) / (E * I(x)) dx

with x measured from the FIXED end,
    b = L - load_height          (load position from fixed end)
    M(x) = P * (b - x)           (real moment, zero beyond load point)
    m(x) = 1 * (L - x)           (unit tip load moment)

Numerical integration keeps this exact-enough and general for any number
of segments (necked tools = 3+ segments, no code change).

Units: inches, lbf, psi internally. Helpers convert.
"""

from __future__ import annotations
from dataclasses import dataclass, field

PSI_PER_GPA = 145037.7
N_PER_LBF = 4.4482216
IN_PER_MM = 1.0 / 25.4


@dataclass(frozen=True)
class BeamSegment:
    length_in: float      # axial length of segment
    diameter_in: float    # effective bending diameter of segment


@dataclass(frozen=True)
class ToolBeam:
    """Segments listed tip-first. E defaults to solid carbide."""
    segments: tuple
    E_gpa: float = 600.0

    @property
    def length_in(self) -> float:
        return sum(s.length_in for s in self.segments)

    def _I_at(self, x_from_fixed: float) -> float:
        """Second moment of area (in^4) at position x from the fixed end."""
        import math
        # walk from tip: position from tip = L - x
        pos_from_tip = self.length_in - x_from_fixed
        acc = 0.0
        for seg in self.segments:
            acc += seg.length_in
            if pos_from_tip <= acc + 1e-12:
                return math.pi * seg.diameter_in ** 4 / 64.0
        # beyond last segment (shouldn't happen) -> last segment
        return 3.14159265358979 * self.segments[-1].diameter_in ** 4 / 64.0

    def tip_deflection_per_lbf(self, load_height_in: float, n: int = 2000) -> float:
        """Tip deflection (inches) per 1 lbf of lateral load applied at
        `load_height_in` above the tip. This is a compliance (in/lbf)."""
        L = self.length_in
        b = L - min(max(load_height_in, 0.0), L)   # load pos from fixed end
        if b <= 0.0:
            return 0.0
        E_psi = self.E_gpa * PSI_PER_GPA
        dx = b / n
        total = 0.0
        for i in range(n):
            x = (i + 0.5) * dx
            M = (b - x)              # per unit P
            m = (L - x)
            total += M * m / self._I_at(x) * dx
        return total / E_psi

    def tip_deflection_in(self, force_N: float, load_height_in: float) -> float:
        return (force_N / N_PER_LBF) * self.tip_deflection_per_lbf(load_height_in)

    def allowable_force_N(self, deflection_limit_in: float,
                          load_height_in: float) -> float:
        c = self.tip_deflection_per_lbf(load_height_in)
        if c <= 0.0:
            return float("inf")
        return (deflection_limit_in / c) * N_PER_LBF


def standard_endmill(diameter_in: float, stickout_in: float, loc_in: float,
                     core_factor: float = 0.75, E_gpa: float = 600.0,
                     neck_diameter_in: float | None = None,
                     neck_length_in: float = 0.0) -> ToolBeam:
    """Build a beam for a common end mill.

    tip-first: fluted section (reduced core diameter), optional neck,
    then full shank for the remainder of the stickout.
    """
    segs = [BeamSegment(min(loc_in, stickout_in), diameter_in * core_factor)]
    used = segs[0].length_in
    if neck_diameter_in and neck_length_in > 0 and used < stickout_in:
        nl = min(neck_length_in, stickout_in - used)
        segs.append(BeamSegment(nl, neck_diameter_in))
        used += nl
    if used < stickout_in:
        segs.append(BeamSegment(stickout_in - used, diameter_in))
    return ToolBeam(tuple(segs), E_gpa)
