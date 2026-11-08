"""Offline scoring of a rosbag2: replay /map and /odom in bag time and score like `record`."""
from .mapio import from_msg
from .runreport import Scorer, build_report


def read_bag(path, ns='', gt_topic=''):
    """Yield (t_seconds_from_start, kind, msg) for map/odom messages, ordered by bag time."""
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.serialization import deserialize_message
    uri = path.rstrip('/')
    fmt = 'sqlite3'
    if uri.endswith('.mcap'):
        fmt = 'mcap'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=fmt),
                rosbag2_py.ConverterOptions('', ''))
    pre = ('/' + ns.strip('/')) if ns else ''
    topics = {pre + '/map': ('map', OccupancyGrid), pre + '/odom': ('odom', Odometry)}
    if gt_topic:
        topics[gt_topic] = ('gt', Odometry)
    have = {t.name for t in reader.get_all_topics_and_types()}
    if pre + '/map' not in have:
        raise ValueError(f'bag has no {pre}/map topic (topics: {sorted(have)})')
    t0 = None
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name not in topics:
            continue
        if t0 is None:
            t0 = stamp
        kind, typ = topics[name]
        yield (stamp - t0) / 1e9, kind, deserialize_message(data, typ)


def score_bag(path, truth, label='bag', interval=2.0, scorer_kw=None, ns='', stop_at=100.0, truth_path='', gt_topic=''):
    """Score a bag -> run report dict. Raises ValueError if the bag has no map."""
    scorer = Scorer(truth, **(scorer_kw or {}))
    cur, pts, series, next_t, last_t, done = None, [], [], 0.0, 0.0, False
    est, gt = [], []
    for t, kind, msg in read_bag(path, ns, gt_topic):
        if kind == 'gt':
            gt.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
            continue
        if kind == 'odom' or not done:
            last_t = t
        if kind == 'odom':
            p = msg.pose.pose.position
            est.append((t, p.x, p.y))
            if not pts or abs(p.x - pts[-1][0]) + abs(p.y - pts[-1][1]) > 0.02:
                pts.append((p.x, p.y))
            continue
        while cur is not None and next_t <= t and not done:         # samples due before this map arrives
            series.append((next_t, scorer(cur)))
            next_t += interval
            done = series[-1][1]['coverage_pct'] >= stop_at
        if done:
            continue                                                 # target reached: keep reading odom/gt only
        if cur is None:
            next_t = max(next_t, t)                                  # first sample = first map
        cur = from_msg(msg)
    if cur is not None and (not series or series[-1][0] < last_t) and not done:
        series.append((max(last_t, series[-1][0] if series else 0.0), scorer(cur)))
    if not series:
        raise ValueError('no /map messages in bag - nothing to score')
    extra = {'source': 'rosbag', 'bag': str(path)}
    if gt_topic:
        from .metrics import localisation_error
        loc = localisation_error(est, gt)
        if loc:
            extra['localisation'] = loc
    return build_report(label, truth_path, series, pts, extra)
