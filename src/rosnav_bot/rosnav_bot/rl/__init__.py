"""rosnav_bot.rl — offline PPO local-planner training (research)."""

# Lazy so `import rosnav_bot.rl.frontier_env` doesn't pull in torch/Pillow
# (CI and lightweight users only need numpy + gymnasium for the frontier env).
__all__ = ['ScanNavEnv', 'ActorCritic']


def __getattr__(name):
    if name == 'ScanNavEnv':
        from rosnav_bot.rl.scan_nav_env import ScanNavEnv
        return ScanNavEnv
    if name == 'ActorCritic':
        from rosnav_bot.rl.policy import ActorCritic
        return ActorCritic
    raise AttributeError(f"module 'rosnav_bot.rl' has no attribute {name!r}")
