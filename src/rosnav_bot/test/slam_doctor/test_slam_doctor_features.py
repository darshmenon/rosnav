import numpy as np

from slam_doctor.core import SlamDoctor
from slam_doctor.features import frame_problems, image_stats, to_gray


def textured(seed=0):
    return np.random.default_rng(seed).integers(0, 256, (240, 320), dtype=np.uint8)


def test_textured_frame_is_fine():
    assert frame_problems(image_stats(textured())) == []


def test_blank_wall_is_feature_poor_and_blurred():
    p = frame_problems(image_stats(np.full((240, 320), 128, np.uint8)))
    assert 'FEATURE_POOR' in p and 'BLURRED' in p


def test_dark_and_overexposed():
    assert 'DARK' in frame_problems(image_stats(np.full((100, 100), 5, np.uint8)))
    assert 'OVEREXPOSED' in frame_problems(image_stats(np.full((100, 100), 250, np.uint8)))


def test_blur_lowers_sharpness():
    g = textured().astype(np.float32)
    blurred = sum(np.roll(g, s, axis=1) for s in range(-6, 7)) / 13
    assert image_stats(blurred.astype(np.uint8), use_orb=False)['sharpness'] < image_stats(textured(), use_orb=False)['sharpness'] / 10


def test_numpy_fallback_counts_corners_not_flat():
    assert image_stats(textured(), use_orb=False)['keypoints'] > 200
    assert image_stats(np.full((240, 320), 90, np.uint8), use_orb=False)['keypoints'] == 0


def test_to_gray_encodings():
    rgb = np.zeros((4, 6, 3), np.uint8); rgb[..., 0] = 200
    g = to_gray(rgb.tobytes(), 6, 4, 18, 'rgb8')
    assert g.shape == (4, 6) and abs(int(g[0, 0]) - 59) <= 1
    assert to_gray(b'\0' * 24, 6, 4, 6, 'mono8').shape == (4, 6)
    assert to_gray(b'\0' * 48, 6, 4, 12, '16UC1') is None


def test_doctor_warns_after_consecutive_bad_frames():
    d = SlamDoctor(); d.map_odom(0, 0, 0, 0); d.map_stats(1, 0, 0, 0)
    bad = image_stats(np.full((100, 100), 128, np.uint8))
    for i in range(4):
        d.frame(bad, i)
    assert 'FEATURE_POOR' not in {f.code for f in d.diagnose(0.5)}
    d.frame(bad, 4)
    assert 'FEATURE_POOR' in {f.code for f in d.diagnose(0.5)}
    d.frame(image_stats(textured()), 5)       # one good frame clears it
    assert 'FEATURE_POOR' not in {f.code for f in d.diagnose(0.5)}
