import importlib.util
import math
import pathlib

import numpy as np

spec = importlib.util.spec_from_file_location(
    'adapter', pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'lidar_ring_time_adapter.py')
ad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ad)


def pt(az_deg, elev_deg, r=5.0):
    a, e = math.radians(az_deg), math.radians(elev_deg)
    return [r * math.cos(e) * math.cos(a), r * math.cos(e) * math.sin(a), r * math.sin(e)]


def test_ring_spans_all_channels_bottom_to_top():
    xyz = np.array([pt(0, e) for e in np.linspace(-10, 10, 16)])
    ring, _ = ad.ring_and_time(xyz, 16, 10.0)
    assert list(ring) == list(range(16))


def test_ring_clamped_outside_fov():
    ring, _ = ad.ring_and_time(np.array([pt(0, -40), pt(0, 40)]), 16, 10.0)
    assert list(ring) == [0, 15]


def test_time_is_zero_at_start_and_one_period_at_end_of_sweep():
    _, t = ad.ring_and_time(np.array([pt(-179.9, 0), pt(0, 0), pt(179.9, 0)]), 16, 10.0, 0.1)
    assert t[0] < 0.001 and math.isclose(t[1], 0.05, abs_tol=1e-4) and t[2] > 0.099


def test_time_monotonic_in_azimuth_and_bounded():
    az = np.linspace(-179, 179, 200)
    _, t = ad.ring_and_time(np.array([pt(a, 3) for a in az]), 16, 10.0, 0.1)
    assert np.all(np.diff(t) > 0) and t.min() >= 0 and t.max() <= 0.1


def test_dtypes():
    ring, t = ad.ring_and_time(np.array([pt(10, 2)]))
    assert ring.dtype == np.uint16 and t.dtype == np.float32


def test_message_layout_matches_record_size_and_alignment():
    # PointField datatypes: 4 = UINT16, 7 = FLOAT32 (sensor_msgs/PointField)
    size = {4: 2, 7: 4}
    assert ad.POINT_STEP == ad.OUT_DTYPE.itemsize == 24
    for name, off, dt in ad.FIELD_LAYOUT:
        assert ad.OUT_DTYPE.fields[name][1] == off, name          # declared offset == real offset
        assert off % size[dt] == 0, name                          # naturally aligned
        assert off + size[dt] <= ad.POINT_STEP, name
    rec = np.zeros(2, dtype=ad.OUT_DTYPE)
    rec['ring'], rec['time'] = [3, 9], [0.01, 0.07]
    raw = rec.tobytes()
    back = np.frombuffer(raw, dtype=ad.OUT_DTYPE)
    assert list(back['ring']) == [3, 9] and np.allclose(back['time'], [0.01, 0.07])
    assert len(raw) == 2 * ad.POINT_STEP
