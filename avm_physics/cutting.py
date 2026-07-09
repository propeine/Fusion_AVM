"""Mechanistic cutting force model.

Kienzle: Kc = Kc1.1 * h^(-mc)   [N/mm^2, h in mm]

Engagement geometry (peripheral milling, ae <= D/2 typical adaptive):
    phi     = arccos(1 - 2*ae/D)                 radial engagement arc (rad)
    h_avg   = fz * (2*ae/D) / phi                mean chip thickness over arc
    h_max   = 2*fz*sqrt((ae/D)*(1 - ae/D))       max chip thickness

Average tangential force (steady, power-equivalent):
    Ft_avg = Kc(h_avg) * ap * h_avg * (z * phi / (2*pi))
           = Kc(h_avg) * MRR / v_c               (identical)

Helix smearing: a flute at depth ap wraps angle
    wrap = ap * tan(helix) / R
Effective in-cut duty of the cutter:
    duty = min(1, z * (phi + wrap) / (2*pi))
Peak lateral force = average / duty. At deep ap the wrap >> flute spacing
and force is steady (duty -> 1); at shallow ap the cut is interrupted and
peaks matter. Deflection is checked against PEAK.

Lateral resultant for deflection: F_lat = Ft_peak * sqrt(1 + kr^2)
with kr = radial/tangential force ratio (material-ish, ~0.3-0.4).

Units: geometry in inches at the interface, converted to mm internally.
Forces in N.
"""

from __future__ import annotations
import math
from dataclasses import dataclass

MM = 25.4


@dataclass(frozen=True)
class Material:
    name: str
    kc11: float          # N/mm^2 at h = 1 mm
    mc: float            # Kienzle exponent
    kr: float = 0.35     # radial/tangential ratio
    kc11_lo: float = 0.0 # uncertainty band (0 = tight data)
    kc11_hi: float = 0.0

    def kc(self, h_mm: float, kc11_override: float | None = None) -> float:
        h = max(h_mm, 1e-4)
        k = self.kc11 if kc11_override is None else kc11_override
        return k * h ** (-self.mc)


MATERIALS = {
    # kc11 / mc from Sandvik/Kennametal-style tabulations; bands honest.
    "AL6061": Material("AL6061", kc11=800.0, mc=0.23, kr=0.35),
    "AL7075": Material("AL7075", kc11=850.0, mc=0.24, kr=0.35),
    "DELRIN": Material("DELRIN", kc11=300.0, mc=0.20, kr=0.35,
                       kc11_lo=180.0, kc11_hi=550.0),   # polymer data is mush
    "SS17-4": Material("SS17-4", kc11=2400.0, mc=0.21, kr=0.45),
    "TI6AL4V": Material("TI6AL4V", kc11=1400.0, mc=0.23, kr=0.45),
}


@dataclass(frozen=True)
class CutGeometry:
    tool_diameter_in: float
    flutes: int
    helix_deg: float = 38.0


@dataclass(frozen=True)
class CutForces:
    phi_rad: float
    h_avg_mm: float
    h_max_mm: float
    kc_navg: float        # N/mm^2 at h_avg
    ft_avg_N: float
    duty: float
    ft_peak_N: float
    f_lat_peak_N: float   # what bends the tool
    mrr_in3min: float


def engagement(geom: CutGeometry, ap_in: float, ae_in: float,
               fz_in: float) -> tuple:
    D = geom.tool_diameter_in
    r = min(max(ae_in / D, 1e-6), 0.999)
    phi = math.acos(max(-1.0, 1.0 - 2.0 * r))
    fz = fz_in * MM
    h_avg = fz * (2.0 * r) / phi
    h_max = 2.0 * fz * math.sqrt(r * (1.0 - r)) if r <= 0.5 else fz
    return phi, h_avg, h_max


def cut_forces(geom: CutGeometry, mat: Material, ap_in: float, ae_in: float,
               fz_in: float, rpm: float,
               kc11_override: float | None = None) -> CutForces:
    phi, h_avg, h_max = engagement(geom, ap_in, ae_in, fz_in)
    kc = mat.kc(h_avg, kc11_override)
    ap_mm = ap_in * MM
    ft_avg = kc * ap_mm * h_avg * (geom.flutes * phi / (2.0 * math.pi))

    wrap = ap_in * math.tan(math.radians(geom.helix_deg)) / (geom.tool_diameter_in / 2.0)
    duty = min(1.0, geom.flutes * (phi + wrap) / (2.0 * math.pi))
    ft_peak = ft_avg / max(duty, 1e-9)
    f_lat = ft_peak * math.sqrt(1.0 + mat.kr ** 2)

    feed_inmin = fz_in * geom.flutes * rpm
    mrr = ap_in * ae_in * feed_inmin
    return CutForces(phi, h_avg, h_max, kc, ft_avg, duty, ft_peak, f_lat, mrr)


def spindle_power_kw(forces: CutForces, geom: CutGeometry, rpm: float) -> float:
    """Average cutting power from average tangential force."""
    vc_m_s = math.pi * (geom.tool_diameter_in * 0.0254) * rpm / 60.0
    return forces.ft_avg_N * vc_m_s / 1000.0
