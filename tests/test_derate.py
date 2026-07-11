"""Invariant tests for avm_physics.derate — the handoff contract, plus
the per-clone feed-scale contract that preserves solver fz raises."""
import math, sys, os
import pytest
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from avm_physics.derate import (engagement_angle, duty_factor, round_rpm,
                                solve_derates)

D = 0.5


def test_geometry_known_points():
    assert math.isclose(engagement_angle(0.25, D), math.pi / 2)   # half slot
    assert math.isclose(engagement_angle(0.5, D), math.pi)        # full slot
    assert math.isclose(engagement_angle(0.9, D), math.pi)        # over-slot clamps


def test_garbage_inputs_raise():
    with pytest.raises(ValueError):
        engagement_angle(0.1, 0.0)
    with pytest.raises(ValueError):
        engagement_angle(-0.1, D)
    with pytest.raises(ValueError):
        round_rpm(1000, 0)
    with pytest.raises(ValueError):
        solve_derates([0.1], D, 0.075, 8000, 100.0)
    with pytest.raises(ValueError):
        solve_derates([0.1], D, 0.075, 8000, -1.0)
    with pytest.raises(ValueError):
        solve_derates([0.1], D, 0.075, 0.0, 40.0)


def test_rounding():
    assert round_rpm(4796) == 4800
    assert round_rpm(4794) == 4790
    assert round_rpm(3) == 10            # floor at one step
    assert round_rpm(4796, step=25) == 4800
    assert round_rpm(4787, step=25) == 4775


SMOKE = dict(diameter=D, ref_woc=0.075, ref_rpm=8000.0, derate_pct=40.0)
LEVELS = [0.150, 0.130, 0.110, 0.090, 0.075]   # 30..15% WOC


def test_reference_level_is_identity():
    r = solve_derates(LEVELS, **SMOKE)[-1]     # 15% = reference
    assert r.derate_frac == 0.0
    assert r.rpm == 8000
    assert math.isclose(r.feed_scale, 1.0)


def test_widest_gets_exact_entered_pct():
    r = solve_derates(LEVELS, **SMOKE)[0]      # 30% = anchor
    assert math.isclose(r.derate_frac, 0.40)
    assert r.rpm == round_rpm(8000 * 0.60)     # 4800


def test_smoke_table_matches_handoff():
    # handoff-validated run: RPMs 4800/5580/6410/7290/8000
    rpms = [r.rpm for r in solve_derates(LEVELS, **SMOKE)]
    assert rpms == [4800, 5580, 6410, 7290, 8000]


def test_rpm_monotone_decreasing_with_woc():
    rs = solve_derates(LEVELS, **SMOKE)
    for a, b in zip(rs, rs[1:]):               # deep->narrow ordering
        assert a.rpm <= b.rpm


def test_narrower_than_reference_clamps_no_speedup():
    r = solve_derates([0.150, 0.040], **SMOKE)[1]
    assert r.derate_frac == 0.0
    assert r.rpm == 8000


def test_fz_held_exactly_per_clone_own_feed():
    """feed_scale applied to each clone's OWN feed holds that clone's own
    fz to machine precision from the ROUNDED rpm — including a clone
    whose fz the WOC solver raised (the L10/SWEEP case)."""
    own_feeds = [2200.0, 2100.0, 2000.0, 1900.0, 5400.0]  # last = raised fz
    rs = solve_derates(LEVELS, diameter=D, ref_woc=0.075, ref_rpm=8000.0,
                       derate_pct=37.0)                   # ugly pct
    for feed, r in zip(own_feeds, rs):
        new_feed = feed * r.feed_scale
        assert math.isclose(new_feed / r.rpm, feed / 8000.0, rel_tol=1e-12)


def test_zero_pct_is_identity_modulo_rounding():
    rs = solve_derates(LEVELS, D, 0.075, 8005.0, 0.0)
    assert all(r.derate_frac == 0.0 for r in rs)
    assert all(r.rpm == 8000 for r in rs)      # nearest 10; ties half-even


def test_all_levels_equal_reference_no_div_by_zero():
    rs = solve_derates([0.075, 0.075], **SMOKE)
    assert all(r.derate_frac == 0.0 and r.rpm == 8000 for r in rs)


def test_empty_input():
    assert solve_derates([], **SMOKE) == []


def test_derate_sublinear_in_woc():
    """Duty-factor interpolation: derate grows sublinearly with WOC
    (arc grows faster at low immersion) — handoff note, not a bug."""
    rs = solve_derates(LEVELS, **SMOKE)
    mid = rs[2]                                # 22% WOC, midway in WOC terms
    woc_linear = 0.40 * (0.110 - 0.075) / (0.150 - 0.075)   # 18.7%
    assert mid.derate_frac > woc_linear
