"""The offline Hungarian-vs-greedy benchmark (scripts/bench_frontier_allocation.py), small N, no ROS."""
import importlib.util
import os

import pytest

pytest.importorskip('scipy')
_spec = importlib.util.spec_from_file_location(
    'bench_frontier_allocation', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'bench_frontier_allocation.py'))
b = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(b)


@pytest.mark.parametrize('dist', b.DISTS)
def test_reserved_mode_never_assigns_two_robots_the_same_goal(dist):
    r = b.benchmark(40, 'hungarian_reserved', dist)
    assert r['conflicts'] == 0 and r['greedy_conflicts'] == 0


def test_prefix_imitation_does_produce_conflicts():
    # the duplicate-goal bug the reservation fixed: this mode must still show it, or the before/after figures mean nothing
    assert b.benchmark(200, 'hungarian', 'base')['conflicts'] > 0


def test_base_run_is_reproducible():
    assert b.benchmark(30, 'hungarian_reserved', 'base') == b.benchmark(30, 'hungarian_reserved', 'base')


def test_report_has_the_headline_numbers():
    txt = b.report(b.benchmark(20, 'hungarian_reserved', 'fewer'))
    assert 'gain' in txt and 'strictly better' in txt and 'conflicting goal pairs' in txt


@pytest.mark.parametrize('scorer', ['utility', 'weighted'])
def test_production_scorer_mode_assigns_without_conflicts(scorer):
    r = b.benchmark_production(40, 'base', scorer)
    assert r['hungarian']['conflicts'] == 0 and r['greedy']['conflicts'] == 0
    assert r['hungarian']['assigned'] >= r['greedy']['assigned'] * 0.95


def test_production_scorer_is_the_coordinators_own_code():
    table, robots, pos, size = b.production_scenario(3, 'base', 'utility')
    (score, xy), r = table[robots[0]][0], robots[0]
    d = ((pos[r][0] - xy[0]) ** 2 + (pos[r][1] - xy[1]) ** 2) ** 0.5
    assert score <= 1.0 * size[xy] - 3.0 * d + 1e-9          # utility formula, minus any suspicious-frontier penalty
    assert 'report' not in table and 'difference' in b.report_production(b.benchmark_production(10, 'base', 'utility'))
