"""Per-level ae/fz solver.

For each stepup level (given its ap):
  1. FORCE branch  : largest ae such that peak lateral force == baseline
                     peak lateral force (rule #5: never exceed what the
                     original full-DOC cut produced). Kc11 cancels here.
  2. DEFLECTION br.: largest ae such that predicted tip deflection at the
                     level's load height == deflection limit * safety
                     factor. Needs absolute force -> Kc11 (or measured
                     spindle-power calibration).
  3. ae_level = min(force, deflection, geometric WOC cap)
  4. If ae capped below both budgets, raise fz until the tighter budget
     (or feed/chipload ceilings) binds.

Reports the binding constraint per level.
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field

from .beam import ToolBeam
from .cutting import CutGeometry, Material, cut_forces, spindle_power_kw


@dataclass(frozen=True)
class Limits:
    deflection_in: float = 0.001
    deflection_sf: float = 1.0          # multiply budget (0.8 = conservative)
    woc_cap_frac: float = 0.375         # max ae as fraction of D
    fz_max_in: float = 0.012            # chipload ceiling
    fz_min_in: float = 0.0005           # rubbing floor
    feed_max_inmin: float = 700.0       # machine cutting-feed ceiling
    power_cont_kw: float = 6.7          # Speedio continuous
    power_peak_kw: float = 18.9         # Speedio burst


@dataclass
class LevelResult:
    level: int
    ap_in: float
    ae_in: float
    ae_frac: float
    fz_in: float
    feed_inmin: float
    f_lat_peak_N: float
    deflection_in: float
    defl_budget_N: float
    force_target_N: float
    power_kw: float
    binding: str
    speedup: float                      # (ae*fz)/(ae0*fz0)
    ae_force_in: float = 0.0
    ae_defl_in: float = 0.0


def _lat_force(geom, mat, ap, ae, fz, rpm, kc11=None):
    return cut_forces(geom, mat, ap, ae, fz, rpm, kc11).f_lat_peak_N


def _solve_ae_for_force(geom, mat, ap, fz, rpm, target_N, ae_lo, ae_hi,
                        kc11=None) -> float:
    """Bisect ae so peak lateral force == target. Force is monotonic in ae."""
    f_hi = _lat_force(geom, mat, ap, ae_hi, fz, rpm, kc11)
    if f_hi <= target_N:
        return ae_hi          # even max ae doesn't reach target
    lo, hi = ae_lo, ae_hi
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _lat_force(geom, mat, ap, mid, fz, rpm, kc11) < target_N:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _solve_fz_for_force(geom, mat, ap, ae, rpm, target_N, fz_lo, fz_hi,
                        kc11=None) -> float:
    f_hi = _lat_force(geom, mat, ap, ae, fz_hi, rpm, kc11)
    if f_hi <= target_N:
        return fz_hi
    lo, hi = fz_lo, fz_hi
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _lat_force(geom, mat, ap, ae, mid, rpm, kc11) < target_N:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def solve_levels(geom: CutGeometry, beam: ToolBeam, mat: Material,
                 rpm: float, fz0_in: float, ae0_in: float, ap_levels,
                 limits: Limits = Limits(),
                 kc11_override: float | None = None,
                 measured_power_kw: float | None = None,
                 deflection_mode: str = "absolute"):
    """ap_levels: iterable of (level_index, ap_in), deepest first.
    Level with the max ap is the calibration/baseline level.

    kc11_override      : force absolute scale from a chosen Kc11
    measured_power_kw  : spindle-load-delta calibration; rescales Kc11 so
                         the model's baseline average power matches.
    """
    ap0 = max(ap for _, ap in ap_levels)

    # --- absolute-force calibration for the deflection branch ---
    kc11 = kc11_override if kc11_override is not None else mat.kc11
    if measured_power_kw is not None:
        model_p = spindle_power_kw(
            cut_forces(geom, mat, ap0, ae0_in, fz0_in, rpm, kc11), geom, rpm)
        if model_p > 1e-9:
            kc11 *= measured_power_kw / model_p

    # --- force target: rule #5, baseline deep pass peak lateral force ---
    base = cut_forces(geom, mat, ap0, ae0_in, fz0_in, rpm, kc11)
    target_N = base.f_lat_peak_N

    ae_cap = limits.woc_cap_frac * geom.tool_diameter_in
    defl_budget_in = limits.deflection_in * limits.deflection_sf
    c_base = beam.tip_deflection_per_lbf(ap0 / 2.0)

    results = []
    for lvl, ap in ap_levels:
        load_h = ap / 2.0
        if deflection_mode == "baseline":
            # rule #5 extended: no level may deflect more than the proven
            # baseline cut does. Kc cancels; pure beam-geometry ratio.
            c_k = beam.tip_deflection_per_lbf(load_h)
            budget_N = target_N * (c_base / c_k)
        else:
            budget_N = beam.allowable_force_N(defl_budget_in, load_h)
        eff_target = min(target_N, budget_N)

        ae_force = _solve_ae_for_force(geom, mat, ap, fz0_in, rpm, target_N,
                                       ae0_in * 0.05, ae_cap, kc11)
        ae_defl = _solve_ae_for_force(geom, mat, ap, fz0_in, rpm, budget_N,
                                      ae0_in * 0.05, ae_cap, kc11)
        ae = min(ae_force, ae_defl, ae_cap)

        fz = fz0_in
        # if geometric cap binds below both budgets, spend the rest on feed
        f_now = _lat_force(geom, mat, ap, ae, fz, rpm, kc11)
        binding = "force" if ae_force <= ae_defl else "deflection"
        if ae >= ae_cap - 1e-9 and f_now < eff_target * 0.999:
            fz_hi = min(limits.fz_max_in,
                        limits.feed_max_inmin / (geom.flutes * rpm))
            fz = _solve_fz_for_force(geom, mat, ap, ae, rpm, eff_target,
                                     fz0_in, fz_hi, kc11)
            f_now = _lat_force(geom, mat, ap, ae, fz, rpm, kc11)
            if fz >= fz_hi - 1e-9:
                binding = ("chipload/feed ceiling")
            else:
                binding = "force" if eff_target == target_N else "deflection"
            binding += " @ WOC cap"

        forces = cut_forces(geom, mat, ap, ae, fz, rpm, kc11)
        defl = beam.tip_deflection_in(forces.f_lat_peak_N, load_h)
        pkw = spindle_power_kw(forces, geom, rpm)
        feed = fz * geom.flutes * rpm

        results.append(LevelResult(
            level=lvl, ap_in=ap, ae_in=ae, ae_frac=ae / geom.tool_diameter_in,
            fz_in=fz, feed_inmin=feed,
            f_lat_peak_N=forces.f_lat_peak_N, deflection_in=defl,
            defl_budget_N=budget_N, force_target_N=target_N,
            power_kw=pkw, binding=binding,
            speedup=(ae * fz) / (ae0_in * fz0_in),
            ae_force_in=ae_force, ae_defl_in=ae_defl,
        ))
    return results


def band_levels(results, max_bands: int = 12, min_ae_ratio: float = 1.15):
    """Merge adjacent level results into bands. Two rules:
    1. Merge any adjacent pair whose ae ratio < min_ae_ratio (an op split
       isn't worth a <15% load change).
    2. Keep merging the smallest-ratio adjacent pair until <= max_bands.
    Each band runs its DEEPEST member's ae/fz (conservative for the rest).
    Returns list of dicts: level_lo/hi, ap_deep, ap_shallow, ae, fz,
    binding, speedup, thresh (min-axial boundary vs next band, 0 for last).
    """
    bands = [[r] for r in results]           # results are deep-first

    def ratio(a, b):
        # BASE-RELATIVE: candidate band's shallowest ae vs this band's
        # DEEPEST (base) ae. Adjacent-neighbor comparison transitively
        # swallows smooth curves sampled finely (75 levels @ 1.5%/step
        # merged into one band); base-relative breaks a new band every
        # min_ae_ratio of CUMULATIVE growth, as intended.
        base = a[0].ae_in
        cand = b[-1].ae_in
        return max(base, cand) / max(min(base, cand), 1e-9)

    i = 0
    while i < len(bands) - 1:
        if ratio(bands[i], bands[i + 1]) < min_ae_ratio:
            bands[i] += bands.pop(i + 1)
        else:
            i += 1
    while len(bands) > max_bands:
        idx = min(range(len(bands) - 1),
                  key=lambda i: ratio(bands[i], bands[i + 1]))
        bands[idx] += bands.pop(idx + 1)

    out = []
    for i, b in enumerate(bands):
        deep = b[0]                          # deepest member drives loads
        shallow = b[-1]
        if i + 1 < len(bands):
            nxt_deep = bands[i + 1][0]
            thresh = 0.5 * (shallow.ap_in + nxt_deep.ap_in)
        else:
            thresh = 0.0
        out.append({
            "level_lo": deep.level, "level_hi": shallow.level,
            "ap_deep": deep.ap_in, "ap_shallow": shallow.ap_in,
            "ae": deep.ae_in, "fz": deep.fz_in, "binding": deep.binding,
            "speedup": deep.speedup, "thresh": thresh,
        })
    return out
