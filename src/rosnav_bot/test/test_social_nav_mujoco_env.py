import os
import sys

import numpy as np
import pytest

pytest.importorskip('mujoco')
pytest.importorskip('gymnasium')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
MAP = os.path.join(HERE, '..', 'maps', 'map_hospital.yaml')
pytestmark = pytest.mark.skipif(not os.path.exists(MAP), reason='hospital map missing')

from rosnav_bot.rl.social_nav_mujoco_env import SocialNavMujocoEnv  # noqa: E402


@pytest.fixture(scope='module')
def env():
    return SocialNavMujocoEnv()


def test_reset_step_shapes(env):
    obs, info = env.reset(seed=0)
    assert obs.shape == env.observation_space.shape == (45,)
    assert 4 <= env.n_peds <= 12
    obs, r, term, trunc, info = env.step(env.action_space.sample())
    assert obs.shape == (45,) and np.isfinite(obs).all()
    assert {'collided', 'min_clearance', 'intrusion_s'} <= set(info)


def test_determinism(env):
    def rollout():
        env.reset(seed=7)
        out = []
        for _ in range(15):
            o, r, *_ = env.step([0.5, 0.2])
            out.append((o.copy(), r))
        return out
    a, b = rollout(), rollout()
    for (oa, ra), (ob, rb) in zip(a, b):
        assert np.allclose(oa, ob) and ra == rb


def test_goal_reached_ends_episode(env):
    env.reset(seed=3)
    env.crowd.pos[:] = 500.0 + 10.0 * np.arange(env.n_peds)[:, None]   # peds far away, distinct
    env.goal = env.robot_xy + 0.3 * np.array([np.cos(env.theta), np.sin(env.theta)])
    env.prev_dist = 0.3
    _, r, term, _, info = env.step([0.8, 0.0])
    assert term and info['success'] and r > 5


def test_collision_detected(env):
    env.reset(seed=4)
    env.crowd.pos[0] = env.robot_xy + 0.1         # on top of the robot
    env.crowd.vel[:] = 0.0
    _, r, term, _, info = env.step([0.0, 0.0])
    assert term and info['collided'] and r < -5
    assert info['min_clearance'] < 0


def test_reward_finite_and_timeout(env):
    env.reset(seed=5)
    env.max_steps = 5
    try:
        for _ in range(5):
            _, r, term, trunc, _ = env.step([0.0, 0.0])
            assert np.isfinite(r)
            if term:
                break
        assert term or trunc
    finally:
        env.max_steps = 400
