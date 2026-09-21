"""Regression checks for the photo/video cable routing registry."""

import numpy as np

from exact_joint.mechanics import CABLE_FAMILIES, CABLE_GROUPS, tension_pattern


def test_photo_pattern_has_seven_disjoint_named_families():
    assert tuple(CABLE_GROUPS) == CABLE_FAMILIES
    assert set(CABLE_GROUPS["Compression"]) == {0, 1, 2, 3}
    assert set(CABLE_GROUPS["Twist CW"]) == {4, 5, 6, 7}
    assert set(CABLE_GROUPS["Twist CCW"]) == {8, 9, 10, 11}
    assert set(CABLE_GROUPS["Twist CW"]).isdisjoint(CABLE_GROUPS["Twist CCW"])


def test_tension_pattern_only_loads_the_selected_photo_group():
    for family in CABLE_FAMILIES:
        values = tension_pattern(family, 3.0)
        assert values.shape == (12,)
        assert np.all(values >= 0)
        assert np.all(values[list(CABLE_GROUPS[family])] == 3.0)
        unloaded = sorted(set(range(12)) - set(CABLE_GROUPS[family]))
        assert np.all(values[unloaded] == 0.0)
