import numpy as np
import pytest

from slam_doctor.doctor import calibrate_main
from slam_doctor.features import calibrate


def test_calibrate_suggests_floors_below_the_healthy_range():
    rng = np.random.default_rng(1)
    frames = {i: rng.integers(0, 256, (120, 160), dtype=np.uint8) for i in range(10)}
    r = calibrate([str(i) for i in frames], loader=lambda p: frames[int(p)])
    assert r['frames'] == 10
    assert r['suggest']['KEYPOINTS_MIN'] < r['keypoints']['min']          # no calibration frame would be flagged
    assert r['suggest']['SHARPNESS_MIN'] < r['sharpness']['min']


def test_calibrate_cli_on_a_folder(tmp_path, capsys):
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(2)
    for i in range(4):
        cv2.imwrite(str(tmp_path / f'f{i}.png'), rng.integers(0, 256, (120, 160), dtype=np.uint8))
    assert calibrate_main(str(tmp_path)) == 0
    assert 'suggested KEYPOINTS_MIN=' in capsys.readouterr().out


def test_calibrate_empty_folder(tmp_path, capsys):
    assert calibrate_main(str(tmp_path)) == 2
