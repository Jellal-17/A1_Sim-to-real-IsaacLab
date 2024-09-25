
import numpy
import math

# required configs
from omni.isaac.lab.utils import configclass
from omni.isaac.a1_tasks.config.env_config import A1EnvCfg
from omni.isaac.a1_tasks.config.terrain_cfg import TERRAIN1_CFG, TERRAIN1_PLAY_CFG

#robot Configuration
from omni.isaac.lab_assets.unitree import UNITREE_A1_CFG  # isort: skip

@configclass
class A1baseEnvCfg(A1EnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_A1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": math.pi / 4,
            ".*calf_joint": -math.pi / 2,
        }

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = TERRAIN1_CFG

        # update viewport camera
        self.viewer.eye = (0.0, 0.0, 75.0)


@configclass
class A1baseEnvCfg_PLAY(A1EnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 64

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = TERRAIN1_PLAY_CFG

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.push_robot = None
        # remove random base mass addition event
        self.events.add_base_mass = None