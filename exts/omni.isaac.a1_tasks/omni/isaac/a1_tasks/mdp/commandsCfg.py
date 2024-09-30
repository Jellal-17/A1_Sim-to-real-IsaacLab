
import math
from dataclasses import MISSING

from omni.isaac.lab.managers import CommandTermCfg
from omni.isaac.lab.markers import VisualizationMarkersCfg
from omni.isaac.lab.markers.config import BLUE_ARROW_X_MARKER_CFG, FRAME_MARKER_CFG, GREEN_ARROW_X_MARKER_CFG, CUBOID_MARKER_CFG
from omni.isaac.lab.utils import configclass
from . import Waypoint2dCommand


@configclass
class Waypoint2dCommandCfg(CommandTermCfg):
    """Configuration for the uniform 2D-pose command generator."""

    class_type: type = Waypoint2dCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    simple_heading: bool = MISSING
    """Whether to use simple heading or not.

    If True, the heading is in the direction of the target position.
    """

    num_intermediates: int = MISSING
    """Number of waypoints that are required to be generated on the path to the position"""

    @configclass
    class Ranges:
        """Uniform distribution ranges for the position commands."""

        pos_x: tuple[float, float] = MISSING
        """Range for the x position (in m)."""
        pos_y: tuple[float, float] = MISSING
        """Range for the y position (in m)."""
        heading: tuple[float, float] = MISSING
        """Heading range for the position commands (in rad).

        Used only if :attr:`simple_heading` is False.
        """

    ranges: Ranges = MISSING
    """Distribution ranges for the position commands."""

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/pose_goal"
    )
    """The configuration for the goal pose visualization marker. Defaults to GREEN_ARROW_X_MARKER_CFG."""
    waypoint_visualizer_cfg: VisualizationMarkersCfg = CUBOID_MARKER_CFG.replace(
        prim_path="/Visuals/Command/pose_waypoint"
    )
    """The configuration for the goal waypoint visualization marker. Defaults to CUBOID_MARKER_CFG."""

    # Set the scale of the visualization markers to (0.2, 0.2, 0.8)
    goal_pose_visualizer_cfg.markers["arrow"].scale = (0.2, 0.2, 0.8)
    waypoint_visualizer_cfg.markers["cuboid"].scale = (0.2, 0.2, 0.8)