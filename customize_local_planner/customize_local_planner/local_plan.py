"""
Struct/container class for local_planner's current path
Author: Matthew Lauriault
Created: 10/18/24
"""


# ROS MODULES
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class LocalPlan:

        """Struct class for LocalPlanner Path"""

        def __init__(self):
                self.future_poses: list[PoseStamped] = None
                self.completed_poses: list[PoseStamped] = None

        def set_path(self, path: Path):
                """Set/change the path to navigate."""
                self.future_poses = path.poses
                self.completed_poses = list[PoseStamped]()

        def is_path_navigated(self) -> bool:
                """Return if all poses have been navigated."""
                return len(self.future_poses) == 0
        
        def has_path(self) -> bool:
                """Return if a path has been set yet."""
                return self.future_poses is not None

        def get_goal_pose(self) -> PoseStamped:
                """Return the current goal pose."""
                if self.is_path_navigated():
                        raise IndexError("No current goal pose. Path has been fully navigated.")
                # return the first future pose
                return self.future_poses[0]
        
        def get_goal_xy(self) -> tuple[float]:
                """Return the (x,y) coordinates of the current goal pose."""
                return (
                        self.get_goal_pose().pose.position.x,
                        self.get_goal_pose().pose.position.y
                )
        
        def complete_goal_pose(self):
                """Current goal pose has been reached -> set next goal pose."""
                if self.is_path_navigated():
                        raise IndexError("No goal pose to complete. Path has been fully navigated.")
                # move completed goal pose from future poses to completed poses
                self.completed_poses.append(self.future_poses.pop(0))
                
        def __eq__(self, other):
                if isinstance(other, Path):
                        return (
                                self.future_poses == other.poses or
                                (self.completed_poses + self.future_poses) == other.poses
                        )
                elif isinstance(other, LocalPlan):
                        return self.future_poses == other.future_poses
                else:
                        return False