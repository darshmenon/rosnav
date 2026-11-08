"""Per-frame visual quality: will a feature-based SLAM (ORB-SLAM3, RTAB-Map, ...) be able to track this view?"""
import numpy as np

# Calibrated on 20 real Gazebo cafe frames (robot camera, 640x480). ORB is run with fastThreshold=7, the lowest
# threshold ORB-SLAM3 itself falls back to, so a view counts as poor only if even ORB-SLAM3 would struggle:
# fast7 gave 681-1000 keypoints on every cafe frame, while the default fast20 gave as few as 8 (and 74 in a live
# run where the tracker was fine). The NumPy proxy (no OpenCV) had min 146 / p10 297 on the same frames.
KEYPOINTS_MIN = 150        # ORB fast7 count below this = feature-poor
KEYPOINTS_MIN_PROXY = 80   # NumPy corner proxy
SHARPNESS_MIN = 10.0       # variance of the Laplacian; cafe frames: min 15, p10 28, median 127
DARK_MEAN = 35.0
BRIGHT_MEAN = 220.0


def to_gray(data, width, height, step, encoding):
    """Raw sensor_msgs/Image bytes -> uint8 gray (H, W), or None for an unsupported encoding."""
    raw = np.frombuffer(data, dtype=np.uint8)
    if encoding in ('mono8', '8UC1'):
        return raw.reshape(height, step)[:, :width]
    if encoding in ('rgb8', 'bgr8'):
        img = raw.reshape(height, step)[:, :width * 3].reshape(height, width, 3).astype(np.float32)
        w = (0.299, 0.587, 0.114) if encoding == 'rgb8' else (0.114, 0.587, 0.299)
        return (img[..., 0] * w[0] + img[..., 1] * w[1] + img[..., 2] * w[2]).astype(np.uint8)
    return None


def _laplacian_var(g):
    g = g.astype(np.float32)
    lap = g[1:-1, :-2] + g[1:-1, 2:] + g[:-2, 1:-1] + g[2:, 1:-1] - 4 * g[1:-1, 1:-1]
    return float(lap.var())


def _corner_count(g, block=8):
    """Corner-ish keypoint proxy without OpenCV: strongest gradient per block, above a contrast floor."""
    g = g.astype(np.float32)
    gx = np.abs(g[1:-1, 2:] - g[1:-1, :-2])
    gy = np.abs(g[2:, 1:-1] - g[:-2, 1:-1])
    score = np.minimum(gx, gy)           # high only where both directions change: corners, not edges
    h, w = (score.shape[0] // block) * block, (score.shape[1] // block) * block
    blocks = score[:h, :w].reshape(h // block, block, w // block, block).max(axis=(1, 3))
    return int((blocks > 20.0).sum())


def image_stats(gray, use_orb=True):
    """{keypoints, sharpness, mean, dark_frac, sat_frac, orb}. Uses OpenCV ORB when importable."""
    n, orb = None, False
    if use_orb:
        try:
            import cv2
            n, orb = len(cv2.ORB_create(nfeatures=1000, fastThreshold=7).detect(gray, None)), True
        except Exception:
            n = None
    if n is None:
        n = _corner_count(gray)
    return {'keypoints': n, 'orb': orb, 'sharpness': _laplacian_var(gray), 'mean': float(gray.mean()),
            'dark_frac': float((gray < 16).mean()), 'sat_frac': float((gray > 239).mean())}


def frame_problems(s):
    """Codes for one frame's stats."""
    out = []
    if s['mean'] < DARK_MEAN or s['dark_frac'] > 0.6:
        out.append('DARK')
    if s['mean'] > BRIGHT_MEAN or s['sat_frac'] > 0.6:
        out.append('OVEREXPOSED')
    if s['sharpness'] < SHARPNESS_MIN:
        out.append('BLURRED')
    if s['keypoints'] < (KEYPOINTS_MIN if s.get('orb') else KEYPOINTS_MIN_PROXY):
        out.append('FEATURE_POOR')
    return out


def calibrate(paths, loader=None):
    """Stats over your own frames -> suggested floors at 80% of the lowest healthy frame, so none of them is flagged."""
    import numpy as np
    if loader is None:
        import cv2
        loader = lambda p: cv2.imread(p, 0)
    rows = []
    for p in paths:
        g = loader(p)
        if g is not None:
            st = image_stats(g)
            rows.append((st['keypoints'], st['sharpness'], st['mean'], st['orb']))
    if not rows:
        raise ValueError('no readable images')
    a = np.array([r[:3] for r in rows], dtype=float)
    p5 = lambda i: float(np.percentile(a[:, i], 5))
    return {'frames': len(rows), 'orb': bool(rows[0][3]),
            'keypoints': {'min': float(a[:, 0].min()), 'p5': p5(0), 'median': float(np.median(a[:, 0]))},
            'sharpness': {'min': float(a[:, 1].min()), 'p5': p5(1), 'median': float(np.median(a[:, 1]))},
            'mean_brightness': {'min': float(a[:, 2].min()), 'median': float(np.median(a[:, 2])), 'max': float(a[:, 2].max())},
            'suggest': {'KEYPOINTS_MIN': int(max(10, 0.8 * float(a[:, 0].min()))), 'SHARPNESS_MIN': round(max(1.0, 0.8 * float(a[:, 1].min())), 1)}}
