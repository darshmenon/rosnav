"""LIO-SAM sim config must match the gz IMU sensor (urdf/imu.xacro) and the URDF mounts."""
import math
import os
import re

import yaml

ROOT = os.path.join(os.path.dirname(__file__), '..')


def _cfg():
    with open(os.path.join(ROOT, 'config', 'lio_sam_sim.yaml')) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def _imu_sigmas():
    s = open(os.path.join(ROOT, 'urdf', 'imu.xacro')).read()
    rate = float(re.search(r'<update_rate>([\d.]+)', s).group(1))
    gyr = float(re.search(r'<angular_velocity>.*?<stddev>([\d.e-]+)', s, re.S).group(1))
    acc = float(re.search(r'<linear_acceleration>.*?<stddev>([\d.e-]+)', s, re.S).group(1))
    return rate, gyr, acc


def test_noise_is_continuous_time_density_of_sim_imu():
    rate, gyr, acc = _imu_sigmas()
    c = _cfg()
    assert math.isclose(c['imuGyrNoise'], gyr / math.sqrt(rate), rel_tol=0.1)
    assert math.isclose(c['imuAccNoise'], acc / math.sqrt(rate), rel_tol=0.1)
    assert 0 < c['imuGyrBiasN'] < c['imuGyrNoise'] and 0 < c['imuAccBiasN'] < c['imuAccNoise']


def test_gravity_and_extrinsic_translation():
    c = _cfg()
    assert c['imuGravity'] == 9.8
    assert c['extrinsicTrans'] == [0.1, 0.0, 0.2]  # laser_frame (0.1,0,0.25) minus imu_link (0,0,0.05)


def test_imu_yaw_relay_math_and_topic():
    src = open(os.path.join(ROOT, 'scripts', 'imu_yaw_relay.py')).read()
    ns = {}
    exec(src.split('class ImuYawRelay')[0].split('from sensor_msgs.msg import Imu')[1], {'math': math}, ns)
    x, y, z, w = ns['rotate_yaw']((0.0, 0.0, math.sin(-math.pi / 4), math.cos(-math.pi / 4)), 90.0)  # -90 deg + 90 deg
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9 and abs(w - 1.0) < 1e-9
    assert _cfg()['imuTopic'] == '/imu_rot'


def _relay_ns():
    src = open(os.path.join(ROOT, 'scripts', 'imu_yaw_relay.py')).read()
    ns = {}
    exec(src.split('class ImuYawRelay')[0].split('from sensor_msgs.msg import Imu')[1], {'math': math}, ns)
    return ns


def test_relay_measures_offset_not_hardcodes_it():
    ns = _relay_ns()
    qz = lambda deg: (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))  # noqa: E731
    assert abs(math.degrees(ns['quat_yaw'](qz(37.0))) - 37.0) < 1e-9
    # IMU reads -90 deg off odom: odom - imu = +90 for every sample, incl. wrap-around at +-180
    diffs = [math.radians(90.0)] * 20
    mean, std = ns['circ_mean_std_deg'](diffs)
    assert abs(mean - 90.0) < 1e-9 and std < 1e-3
    mean, std = ns['circ_mean_std_deg']([math.radians(179.0), math.radians(-179.0)] * 10)
    assert abs(abs(mean) - 180.0) < 1e-6 and std < 3.0
    assert ns['circ_mean_std_deg']([0.0, math.pi / 2, math.pi, -math.pi / 2])[1] > 30.0   # scattered -> rejected by cal_max_std_deg


def test_relay_default_is_passthrough_with_auto_cal():
    src = open(os.path.join(ROOT, 'scripts', 'imu_yaw_relay.py')).read()
    assert "declare_parameter('yaw_deg', 0.0)" in src and "declare_parameter('auto_cal', True)" in src
    launch = open(os.path.join(ROOT, 'launch', 'lio_sam_sim.launch.py')).read()
    assert "'auto_cal': True" in launch and "'yaw_deg': 90.0" not in launch


def test_adapter_has_stride_and_timing_log():
    src = open(os.path.join(ROOT, 'scripts', 'lidar_ring_time_adapter.py')).read()
    assert "declare_parameter('stride', 1)" in src and 'ms/cloud' in src


def test_cloud_xyzi_matches_point_layout_and_drops_nan():
    import numpy as np
    src = open(os.path.join(ROOT, 'scripts', 'lidar_ring_time_adapter.py')).read()
    ns = {'np': np, 'math': math}
    exec(src[src.index('_NP = {'):src.index('# Velodyne-style XYZIRT record')], ns)
    F = type('F', (), {})
    mk = lambda n, o, d: type('F', (), dict(name=n, offset=o, datatype=d))   # noqa: E731
    fields = [mk('x', 0, 7), mk('y', 4, 7), mk('z', 8, 7), mk('intensity', 16, 7)]   # padded: intensity at 16, step 20
    rec = np.zeros(5, dtype=np.dtype({'names': ['x', 'y', 'z', 'i'], 'formats': ['<f4'] * 4, 'offsets': [0, 4, 8, 16], 'itemsize': 20}))
    rec['x'], rec['y'], rec['z'], rec['i'] = [1, 2, np.nan, 4, 5], [0, 0, 0, 0, np.inf], [0, 1, 1, 1, 1], [9, 8, 7, 6, 5]
    out = ns['cloud_xyzi'](fields, rec.tobytes(), 20, 5)
    assert out.shape == (3, 4) and out[:, 0].tolist() == [1, 2, 4] and out[:, 3].tolist() == [9, 8, 6]
    assert ns['cloud_xyzi'](fields, rec.tobytes(), 20, 5, stride=2).shape[0] == 1   # points 0, 2, 4 -> only point 0 finite
    assert F is not None


def test_relay_pairs_by_timestamp_not_latest_message():
    ns = _relay_ns()
    buf = [(1.00, 0.1), (1.01, 0.2), (1.02, 0.3)]
    assert ns['nearest_yaw'](buf, 1.012, 0.05) == 0.2
    assert ns['nearest_yaw'](buf, 5.0, 0.05) is None     # stale odom (CPU-starved) is never paired
    assert ns['nearest_yaw']([], 1.0, 0.05) is None
