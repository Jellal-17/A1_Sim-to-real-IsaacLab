import gymnasium as gym

from omni.isaac.a1_tasks.agents.rsl_rl_ppo_cfg import A1PPORunnerCfg

from . import a1envCfg

"""Creating PPO runners for RSL_RL"""

a1_base_runner_cfg = A1PPORunnerCfg()
a1_base_runner_cfg.experiment_name = "a1_base_v1"

"""Register Gym Environments"""

gym.register(
    id="Isaac-A1-Base-v1",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": a1envCfg.A1baseEnvCfg,
        "rsl_rl_cfg_entry_point": a1_base_runner_cfg,
    },
)

gym.register(
    id="Isaac-A1-Base-Play-v1",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": a1envCfg.A1baseEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": a1_base_runner_cfg,
    },
)
