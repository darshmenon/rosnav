import os

from rosnav_check.cli import main
from rosnav_check.mapcheck import cell_state, check_map, read_map


def _map(tmp_path, name, w, h, origin, fill=254, res=0.05, extra=None):
    px = bytearray([fill] * (w * h))
    for (cx, cy), v in (extra or {}).items():   # cell (cx, cy) from the bottom-left origin
        px[(h - 1 - cy) * w + cx] = v
    (tmp_path / f'{name}.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode() + bytes(px))
    (tmp_path / f'{name}.yaml').write_text(f'image: {name}.pgm\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0]\n'
                                           'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return str(tmp_path / f'{name}.yaml')


def codes(items):
    return {i.code for i in items}


def test_cell_state_and_origin(tmp_path):
    y = _map(tmp_path, 'm', 40, 20, (-1.0, -0.5), extra={(20, 10): 0})
    mp = read_map(y)
    assert cell_state(mp, 0.0, 0.0) == 'occupied'          # cell (20, 10) = (-1+20*.05, -.5+10*.05)
    assert cell_state(mp, -0.5, 0.0) == 'free'
    assert cell_state(mp, 5.0, 5.0) == 'outside'


def test_start_outside_or_occupied_is_error(tmp_path):
    y = _map(tmp_path, 'm', 40, 20, (5.0, 5.0))
    assert 'MAP_START_BAD' in codes(check_map(y, (0.0, 0.0)))
    y2 = _map(tmp_path, 'n', 40, 20, (-1.0, -0.5), extra={(20, 10): 0})
    assert 'MAP_START_BAD' in codes(check_map(y2, (0.0, 0.0)))


def test_unknown_start_warns(tmp_path):
    y = _map(tmp_path, 'm', 40, 20, (-1.0, -0.5), fill=150)   # occ 0.41: between free_thresh and occupied_thresh
    c = codes(check_map(y, (0.0, 0.0)))
    assert 'MAP_START_UNKNOWN' in c and 'MAP_MOSTLY_UNKNOWN' in c


def test_rotated_and_small_vs_reference(tmp_path):
    ref = _map(tmp_path, 'ref', 440, 200, (-13.0, -5.0))                       # landscape 22 x 10 m
    new = _map(tmp_path, 'new', 190, 260, (-3.0, -6.0))                        # portrait 9.5 x 13 m = 124 m2
    c = codes(check_map(new, (0.0, 0.0), ref))
    assert {'MAP_ROTATED', 'MAP_SMALL'} <= c


def test_consistent_map_is_ok(tmp_path):
    ref = _map(tmp_path, 'ref', 440, 200, (-13.0, -5.0))
    same = _map(tmp_path, 'same', 430, 200, (-13.0, -5.0))
    assert codes(check_map(same, (0.0, 0.0), ref)) == {'MAP_INFO', 'MAP_OK'}


def test_unreadable_map(tmp_path):
    assert codes(check_map(str(tmp_path / 'nope.yaml'))) == {'MAP_UNREADABLE'}


def test_cli_exit_code(tmp_path, capsys):
    ref = _map(tmp_path, 'ref', 440, 200, (-13.0, -5.0))
    bad = _map(tmp_path, 'bad', 190, 260, (-3.0, -6.0))
    assert main(['map-guard', bad, '--ref', ref]) == 1
    assert 'MAP_ROTATED' in capsys.readouterr().out
    assert main(['map-guard', ref, '--ref', ref]) == 0
    assert os.path.exists(ref)
