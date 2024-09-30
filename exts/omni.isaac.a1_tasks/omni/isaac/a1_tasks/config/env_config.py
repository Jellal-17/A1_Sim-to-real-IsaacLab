import math
from dataclasses import MISSING

from omni.isaac.a1_tasks import mdp

from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import ContactSensorCfg
from omni.isaac.lab.sim import DistantLightCfg, DomeLightCfg, MdlFileCfg, RigidBodyMaterialCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from omni.isaac.lab.utils.noise import AdditiveGaussianNoiseCfg as GaussianNoise
from omni.isaac.lab.utils.noise import AdditiveUniformNoiseCfg as Unoise

"""Scene settings"""


@configclass
class A1SceneCfg(InteractiveSceneCfg):

    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        terrain_generator=None,
        max_init_terrain_level=0,
        collision_group=-1,
        physics_material=RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/"
            + "TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    # sky light
    light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=DomeLightCfg(
            intensity=750.0,
            color=(0.9, 0.9, 0.9),
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # robot
    robot: ArticulationCfg = MISSING

    # contact sensors
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=4, track_air_time=True, update_period=0.0
    )


""" MDP settings"""


@configclass
class A1ActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)


@configclass
class A1CommandsCfg:
    """Command specifications for the MDP."""

    # base_velocity = mdp.UniformVelocityCommandCfg(
    #     asset_name="robot",
    #     resampling_time_range=(10.0, 10.0),
    #     rel_standing_envs=0.02,
    #     rel_heading_envs=1.0,
    #     heading_command=False,
    #     debug_vis=False,
    #     ranges=mdp.UniformVelocityCommandCfg.Ranges(
    #         lin_vel_x=(-1.0, 1.0), lin_vel_y=(-1.0, 1.0), ang_vel_z=(-1.0, 1.0),
    #     ),
    # )
    """for the "pos_z range", refer to the terrain noise range, which generally represents the min and max height of the terrain"""
    # pose_command = mdp.UniformPose2dCommandCfg(
    #     asset_name="robot",
    #     resampling_time_range=(8.0, 8.0),
    #     debug_vis=True,
    #     simple_heading=True,
    #     ranges=mdp.UniformPose2dCommandCfg.Ranges(
    #         pos_x=(-4.0, 4.0), pos_y=(-4.0, 4.0)
    #     )
    # )
    """Used the above pose command till the exp on 27/09/24"""

    pose_command = mdp.Waypoint2dCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 8.0),
        debug_vis=True,
        simple_heading=True,
        num_intermediates=3,
        ranges=mdp.Waypoint2dCommandCfg.Ranges(pos_x=(-4.0, 4.0), pos_y=(-4.0, 4.0)),
    )


@configclass
class A1ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # `` observation terms (order preserved)
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "pose_command"})
        heading_error = ObsTerm(
            func=mdp.heading_command_error,
            params={"command_name": "pose_command"},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        # height_scan = ObsTerm(
        #     func=mdp.height_scan,
        #     params={"sensor_cfg": SceneEntityCfg("height_scanner")},
        #     noise=Unoise(n_min=-0.1, n_max=0.1),
        #     clip=(-1.0, 1.0),
        # )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class A1EventCfg:
    """Configuration for randomization."""

    # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="trunk"),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )

    # reset
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="trunk"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_around_default,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    # interval
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)},
        },
    )


@configclass
class A1RewardsCfg:
    # -- task
    air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.0001,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "command_name": "pose_command",
            "threshold": 0.5,
        },
    )
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-400.0)

    position_command_error = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=5.0,
        params={"std": 2.0, "command_name": "pose_command"},
    )
    position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=5.0,
        params={"std": 0.2, "command_name": "pose_command"},
    )
    orientation_tracking = RewTerm(
        func=mdp.heading_command_error_abs,
        weight=-1.0,
        params={"command_name": "pose_command"},
    )
    # foot_clearance = RewTerm(
    #     func=mdp.foot_clearance_reward,
    #     weight=0.05,
    #     params={
    #         "std": 0.05,
    #         "tanh_mult": 2.0,
    #         "target_height": 0.1,
    #         "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
    #     },
    # )

    # -- task rewards
    position_tracking = RewTerm(
        func=mdp.position_tracking_reward,
        weight=10.0,
        params={"std": 1.0},
    )

    # If you have a goal achievement reward
    goal_achievement = RewTerm(
        func=mdp.goal_achievement_reward,
        weight=20.0,
        params={
            "success_threshold": 0.1,  # Adjust as needed
        },
    )

    # -- penalties

    base_motion = RewTerm(func=mdp.base_motion_penalty, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot")})

    base_angular_motion = RewTerm(
        func=mdp.base_angular_motion_penalty, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot")}
    )

    base_orientation = RewTerm(
        func=mdp.base_orientation_penalty, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot")}
    )

    action_smoothness = RewTerm(func=mdp.action_smoothness_penalty, weight=-0.1)

    foot_slip = RewTerm(
        func=mdp.foot_slip_penalty,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "threshold": 1.0,
        },
    )

    collision = RewTerm(
        func=mdp.collision,
        weight=-5.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=[".*_calf", ".*_thigh"]),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*_calf", ".*_thigh"]),
            "threshold": 0.1,
        },
    )

    feet_stumble = RewTerm(
        func=mdp.contact_penality,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )

    joint_acc = RewTerm(
        func=mdp.joint_acceleration_penalty,
        weight=-2.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    )
    joint_torques = RewTerm(
        func=mdp.joint_torques_penalty,
        weight=-1.0e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    )
    joint_vel = RewTerm(
        func=mdp.joint_velocity_penalty,
        weight=-1.0e-2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    )
    joint_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-0.004,
    )
    hip_limit = RewTerm(
        func=mdp.hip_pos_error,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_hip"),
        },
    )


@configclass
class A1TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    body_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="trunk"), "threshold": 1.0},
    )


@configclass
class A1CurriculumCfg:
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)


""" Environment basic generation"""


@configclass
class A1EnvCfg(ManagerBasedRLEnvCfg):

    # scene settings
    scene: A1SceneCfg = A1SceneCfg(num_envs=4098, env_spacing=2.5)

    # Basic Settings
    observations: A1ObservationsCfg = A1ObservationsCfg()
    actions: A1ActionsCfg = A1ActionsCfg()
    commands: A1CommandsCfg = A1CommandsCfg()

    # MDP Settings
    rewards: A1RewardsCfg = A1RewardsCfg()
    terminations: A1TerminationsCfg = A1TerminationsCfg()
    events: A1EventCfg = A1EventCfg()
    curriculum: A1CurriculumCfg = A1CurriculumCfg()

    def __post_init__(self):

        # general settings
        self.decimation = 10  # 50 Hz
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.002  # 500 Hz
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
