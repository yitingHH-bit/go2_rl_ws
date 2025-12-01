# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR#, ROBOT_DIR
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
import isaaclab.terrains as terrain_gen
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns, RayCasterCameraCfg
import isaaclab.sim as sim_utils
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise  
#import isaaclab_tasks.manager_based.navigation.mdp as mdp
import robot_lab.tasks.locomotion.velocity.mdp as mdp
from isaaclab.assets import ArticulationCfg, AssetBaseCfg 
from robot_lab.assets.unitree import UNITREE_GO2_CFG  # isort: skip
from isaaclab.markers.config import CUBOID_MARKER_CFG 

from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg, FlatPatchSamplingCfg
#from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_c.flat_env_cfg import AnymalCFlatEnvCfg
from robot_lab.tasks.locomotion.velocity.config.quadruped.unitree_go2.rough_env_cfg  \
                    import UnitreeGo2RoughEnvCfg
from robot_lab.tasks.navigation.terrain.hf_env \
    import TunnelTerrainSceneCfg,AlienGoRoughSceneCfg
from isaaclab.markers.config import GREEN_ARROW_X_MARKER_CFG
from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_c.flat_env_cfg import AnymalCFlatEnvCfg

#LOW_LEVEL_ENV_CFG = AnymalCFlatEnvCfg()


LOW_LEVEL_ENV_CFG = UnitreeGo2RoughEnvCfg()



# @configclass
# class LeggedSceneCfg(TunnelTerrainSceneCfg):
#     """
#     Legged Scene Configuration

#     Note:
#         Terrains can be changed by changing the parent class e.g.
#         LeggedSceneCfg(MarsTerrainSceneCfg) -> LeggedSceneCfg(DebugTerrainSceneCfg)

#     """
#     sky_light = AssetBaseCfg(
#         prim_path="/World/skyLight",
#         spawn=sim_utils.DomeLightCfg(color=(0.2, 0.2, 0.3), intensity=2000.0),
#     )
#     cylinder_light = AssetBaseCfg(
#         prim_path="/World/cylinderLight",
#         spawn=sim_utils.CylinderLightCfg(
#             length=100, radius=0.3, treat_as_line=False, intensity=5000.0
#         ),
#     )
#     cylinder_light.init_state.pos = (0, 0, 5.0)

#     robot: ArticulationCfg = MISSING

#     # contact_sensor: ContactSensorCfg = MISSING

#     height_scanner: RayCasterCfg = MISSING
#     lidar_sensor: RayCasterCfg = MISSING
#     #depth_sensor : RayCasterCameraCfg = MISSING
#     contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)



Go2_Vision_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0), #size=(10.0, 10.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        # "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
        #     proportion=0.1,
        #     step_height_range=(0.05, 0.17),
        #     step_width=0.3,
        #     platform_width=3.0,
        #     border_width=1.0,
        #     holes=False,
        # ),
        # "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
        #     proportion=0.2,
        #     step_height_range=(0.05, 0.17),
        #     step_width=0.3,
        #     platform_width=3.0,
        #     border_width=1.0,
        #     holes=False,
        # ),
        # "boxes": terrain_gen.MeshRandomGridTerrainCfg(
        #     proportion=0.10, grid_width=0.45, grid_height_range=(0.05, 0.1), platform_width=2.0
        # ),
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.20),
        # "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
        #     proportion=0.01, noise_range=(0.02, 0.05), noise_step=0.02, border_width=0.25
        # ),
        "init_pos": terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.60, 
            num_obstacles=10,
            obstacle_height_mode="fixed",
            obstacle_height_range=(0.3, 2.0), obstacle_width_range=(0.4, 1.0), 
            platform_width=0.0
        ),
       
        # "gaps": terrain_gen.MeshGapTerrainCfg(
        #     proportion=0.1, gap_width_range=(0.5, 1.0), platform_width=2.0
        # ),
        # "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
        #     proportion=0.1, slope_range=(0.0, 0.2), platform_width=2.0, border_width=0.25
        # ),
        # "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
        #     proportion=0.15,
        #     step_height_range=(0.05, 0.17),
        #     step_width=0.3,
        #     platform_width=5.0,
        #     border_width=1.0,
        #     holes=False,
        # ),
        # "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
        #     proportion=0.1, slope_range=(0.0, 0.2), platform_width=2.0, border_width=0.25 
        # ),
  
        "cylinder": terrain_gen.MeshRepeatedCylindersTerrainCfg(
            proportion=0.10,
            platform_width=0.0,
            object_type="cylinder",
            object_params_start=terrain_gen.MeshRepeatedCylindersTerrainCfg.ObjectCfg(
                num_objects=4,
                height=1.5,
                radius=0.5
            ),
            object_params_end=terrain_gen.MeshRepeatedCylindersTerrainCfg.ObjectCfg(
                num_objects=8,
                height=1.6,
                radius=0.4
            ),
        )
    },
)
for sub_terrain_name, sub_terrain_cfg in Go2_Vision_TERRAINS_CFG.sub_terrains.items():
    sub_terrain_cfg.flat_patch_sampling = {
        sub_terrain_name: FlatPatchSamplingCfg(num_patches=2, patch_radius=[0.01,0.1,0.2, 0.3, 0.5,0.6, 0.7, 1.0, 1.2, 1.5], max_height_diff=0.3)
    }

@configclass
#class Go2VisionSceneCfg(InteractiveSceneCfg):
class Go2VisionSceneCfg(TunnelTerrainSceneCfg):
    """Configuration for the terrain scene with a legged robot."""
    # scene: LeggedSceneCfg = LeggedSceneCfg(
    #     num_envs=1, env_spacing=4.0, replicate_physics=False)
    
    # ground terrain
    # terrain = TerrainImporterCfg(
    #     prim_path="/World/ground",
    #     terrain_type="generator",
    #     terrain_generator=Go2_Vision_TERRAINS_CFG,
    #     max_init_terrain_level=5,
    #     collision_group=-1,
    #     physics_material=sim_utils.RigidBodyMaterialCfg(
    #         friction_combine_mode="multiply",
    #         restitution_combine_mode="multiply",
    #         static_friction=1.0,
    #         dynamic_friction=1.0,
    #     ),
    #     visual_material=sim_utils.MdlFileCfg(
    #         mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
    #         project_uvw=True,
    #         texture_scale=(0.25, 0.25),
    #     ),
    #     debug_vis=False,
    # )

    # robots
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # sensors
    # height_scanner = RayCasterCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/base",
    #     offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
    #     attach_yaw_only=True,
    #     pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[3.0, 2.0]),
    #     #pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
    #     debug_vis=False,
    #     mesh_prim_paths=["/World/ground"],   #ground  Terrain
    # )

    # lidar_sensor = RayCasterCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/Head_lower",
    #     # offset=RayCasterCfg.OffsetCfg(pos=(0.28945, 0.0, -0.046), rot=(0., -0.991,0.0,-0.131)),
    #     offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, -0.0), rot=(0., -0.991,0.0,-0.131)),
    #     attach_yaw_only=False,  
    #     pattern_cfg=patterns.LidarPatternCfg(
    #         channels=32, vertical_fov_range=(0.0, 90.0), horizontal_fov_range=(-180, 180.0), horizontal_res=4.0
    #     ),
    #     debug_vis=False,
    #     mesh_prim_paths=["/World/ground"],
    # )
            # xt16  lidar   or camera  
    lidar_sensor = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.05), rot=(1.0, 0.0, 0.0, 0.0)),
        attach_yaw_only=False,
        pattern_cfg=patterns.LidarPatternCfg(
            channels=16, vertical_fov_range=(-7.0, 52.0), horizontal_fov_range=(-180, 180.0), horizontal_res=1.3
        ),
        debug_vis=False,
        mesh_prim_paths=["/World/terrain"], #/ #ground Terrain
        )
    
    # depth_sensor = RayCasterCameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/base",  # Ensure the sensor is attached to the correct part
    #     mesh_prim_paths=["/World/ground"],  #  /Obstacles  Assuming the obstacles mesh is needed
    #     update_period=0.1,  # Sensor update rate
    #     offset=RayCasterCameraCfg.OffsetCfg(
    #         pos=(0.2, 0.0, 0.3),
    #         rot=(0.7071, 0.0, 0.7071, 0.0),
    #     ),    #convention="parent"
    #     data_types=["distance_to_image_plane"],  # Using depth information
    #     debug_vis=False,
    #     pattern_cfg=patterns.PinholeCameraPatternCfg(
    #         focal_length=1.93,  # Camera focal length
    #         horizontal_aperture=3.8,  # Camera horizontal field of view
    #         # height=30,  # Resolution height
    #         # width=53   # Resolution width
    #         height=39,  # Resolution height
    #         width=39   # Resolution width
    #     ),
    #     max_distance=3,  # Maximum distance for depth sensing
    #     )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)

    # lights
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )

@configclass
class EventCfg:
    """Configuration for events."""

    reset_base = EventTerm(   #reset_body_state_uniform
        func=mdp.reset_root_state_uniform, #reset_root_state_uniform,# reset_root_state_from_terrain
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5),"yaw": (-3.14, 3.14)},
             "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
            "asset_cfg": SceneEntityCfg(name="robot"),
        },
    )


import os
base_export_dir="/home/jack/NA2/robot_lab/logs/rsl_rl/unitree_go2_rough/2025-07-29_14-26-47/exported"
# @configclass
# class ActionsCfg:
#     """Action terms for the MDP."""

#     pre_trained_policy_action: mdp.PreTrainedPolicyActionCfg = mdp.PreTrainedPolicyActionCfg(
#         asset_name="robot",
#         policy_path=os.path.join(base_export_dir,"policy.pt"),
        
#         low_level_decimation=4,
#         low_level_actions=LOW_LEVEL_ENV_CFG.actions.joint_pos,
          
#         low_level_observations=LOW_LEVEL_ENV_CFG.observations.policy,
#     )
@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    pre_trained_policy_action: mdp.PreTrainedPolicyActionCfg = mdp.PreTrainedPolicyActionCfg(
        asset_name="robot",
        policy_path=os.path.join(base_export_dir,"policy.pt"),
        low_level_decimation=4, 
        low_level_actions=LOW_LEVEL_ENV_CFG.actions.joint_pos,
        low_level_observations=LOW_LEVEL_ENV_CFG.observations.policy,
        high_level_actions_clipping=[(-0.5, 0.5), (-0.5, 0.5), (-1.0, 1.0)],
        low_level_observations_command_name="velocity_commands",
        action_threshold=0.1,
        action_dim=3,
        debug_vis=True,
    ) 

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2), params={"asset_cfg": SceneEntityCfg("robot")}
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "pose_command"})

        lidar_scan = ObsTerm( 
            func=mdp.lidar_scan,# lidar_scan,
            params={  
                "sensor_cfg": SceneEntityCfg("lidar_sensor"),
                "lidar_range": 5.0,
                # 160×1 = 160 点
                #"lidar_resolution": (30, 30),# (50, 50) ok 
                "lidar_resolution": (36, 4), 
            },
            clip=(-5.0, 5.0),
            scale=1.0/5.0,  
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
    policy: PolicyCfg = PolicyCfg()

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
  
    position_tracking = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=1.5,
        params={"std": 2.0, "command_name": "pose_command"},
    )
    position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.5,
        params={"std": 0.2, "command_name": "pose_command"},
    )
    orientation_tracking = RewTerm(
        func=mdp.heading_command_error_abs,
        weight=-0.2,
        params={"command_name": "pose_command"},
    )

    #termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    
    collision_penalty = RewTerm(
    func=mdp.collision_with_obstacles_lidar,
    weight=-20.0,
    params={"sensor_cfg": SceneEntityCfg("lidar_sensor"), "min_distance": 0.40},
    )

@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    pose_command = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=False,
        resampling_time_range=(15.0, 25.0),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(pos_x=(-3.0, 3.0), pos_y=(-3.0, 3.0), heading=(-math.pi, math.pi)),
        goal_pose_visualizer_cfg=GREEN_ARROW_X_MARKER_CFG.replace(
            prim_path="/Visuals/Command/pose_goal",
        ),
    )
    
    def __post_init__(self):
        self.pose_command.goal_pose_visualizer_cfg.markers["arrow"].visual_material.diffuse_color = (1.0, 1.0, 0.0)
        self.pose_command.goal_pose_visualizer_cfg.markers["arrow"].scale = (0.2, 0.2, 0.8)
 
@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )

    is_success = DoneTerm(  
        func=mdp.is_success,
        params={"command_name": "pose_command", "threshold": 0.38},
    )
    collision = DoneTerm( 
        func=mdp.collision_with_obstacles_lidar,
        params={  
            "sensor_cfg": SceneEntityCfg(
            "lidar_sensor"), "min_distance": 0.35},
    )

@configclass
class NavigationRoughEnvCfg(UnitreeGo2RoughEnvCfg):  #UnitreeGo2RoughEnvCfg    ManagerBasedRLEnvCfg
    """Configuration for the navigation environment."""

    # environment settings
    #scene: SceneEntityCfg = LOW_LEVEL_ENV_CFG.scene
    scene: Go2VisionSceneCfg =Go2VisionSceneCfg(num_envs=4096, env_spacing=2.5)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    # mdp settings
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    
    def __post_init__(self):
        """Post initialization."""
    
        self.curriculum=None
   
        self.sim.dt = LOW_LEVEL_ENV_CFG.sim.dt
        self.sim.render_interval = LOW_LEVEL_ENV_CFG.decimation
        #高层导航每隔 40 帧物理步（40/200 Hz = 0.2 s，也就是 5 Hz）才会收集一次观测并下发一次新的速度命令   
        self.decimation = LOW_LEVEL_ENV_CFG.decimation * 10
        self.episode_length_s =self.commands.pose_command.resampling_time_range[1]  

        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

class NavigationRoughEnvCfg_PLAY(NavigationRoughEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
