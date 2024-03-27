"""
A basic dm environment for transporter data collection on the Franka robot.
"""
from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation as R
import dm_env

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def plan_and_execute(
    robot,
    planning_component,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        print("Failed to plan trajectory")
    time.sleep(sleep_time)

class FrankaTable(dm_env.Environment):
    """
    This dm_env is intended to be used in conjunction with PyQt data collection application.
    The management of ROS communication is handled by the data collection application.
    This application is intended to make data collection compatible with env_logger.
    """

    def __init__(self):
        robot_ip = "" # not applicable for fake hardware
        use_gripper = "true" 
        use_fake_hardware = "true" 

        moveit_config = (
            MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro",
                mappings={
                    "robot_ip": robot_ip,
                    "robotiq_gripper": use_gripper,
                    "use_fake_hardware": use_fake_hardware,
                    })
            .robot_description_semantic("config/panda.srdf.xacro", 
                mappings={
                    "robotiq_gripper": use_gripper,
                    })
            .trajectory_execution("config/moveit_controllers.yaml")
            .moveit_cpp(
                file_path=get_package_share_directory("panda_motion_planning_demos")
                + "/config/moveit_cpp.yaml"
            )
            .to_moveit_configs()
            ).to_dict()

        self.panda = MoveItPy(config_dict=moveit_config)
        self.panda_arm = self.panda.get_planning_component("panda_arm") 
        self.mode="pick"

    def reset(self, obs) -> dm_env.TimeStep:
        return dm_env.TimeStep(
                step_type=dm_env.StepType.FIRST,
                reward=0.0,
                discount=0.0,
                observation=obs,
                )

    def step(self, pose, obs) -> dm_env.TimeStep:
        if self.mode == "pick":
            self.pick(pose)
        else:
            self.place(pose)

        return dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=0.0,
                discount=0.0,
                observation=obs,
                )

    def observation_spec(self) -> Dict[str, dm_env.specs.Array]:
        return {
                "overhead_camera/depth": dm_env.specs.Array(shape=(480,480,3), dtype=np.float32),
                "overhead_camera/rgb": dm_env.specs.Array(shape=(480,480), dtype=np.float32),
                }

    def action_spec(self) -> dm_env.specs.Array:
        return dm_env.specs.Array(
                shape=(7,), # [x, y, z, qx, qy, qz, qw]
                dtype=np.float32,
                )

    def close(self):
        raise NotImplementedError

    def pick(self, world_coords):
        pick_pose_msg = PoseStamped()
        pick_pose_msg.header.frame_id = "panda_link0"
        pick_pose_msg.pose.orientation.x = 0.9238795
        pick_pose_msg.pose.orientation.y = -0.3826834
        pick_pose_msg.pose.orientation.z = 0.0
        pick_pose_msg.pose.orientation.w = 0.0
        pick_pose_msg.pose.position.x = world_coords[0]
        pick_pose_msg.pose.position.y = world_coords[1]
        pick_pose_msg.pose.position.z = world_coords[2]
        
        # prepick pose
        panda_arm.set_start_state_to_current_state()
        pre_pick_pose_msg = deepcopy(pick_pose_msg)
        pre_pick_pose_msg.pose.position.z += 0.1
        panda_arm.set_goal_state(pose_stamped_msg=pre_pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # pick pose
        panda_arm.set_start_state_to_current_state()
        panda_arm.set_goal_state(pose_stamped_msg=pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # close gripper
        gripper_client.close_gripper()
        time.sleep(2.0)
        
        # raise arm
        panda_arm.set_start_state_to_current_state()
        pre_pick_pose_msg.pose.position.z += 0.2
        panda_arm.set_goal_state(pose_stamped_msg=pre_pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        self.mode = "place"

    def place(self, world_coords):
        place_pose_msg = PoseStamped()
        place_pose_msg.header.frame_id = "panda_link0"
        place_pose_msg.pose.orientation.x = 0.9238795
        place_pose_msg.pose.orientation.y = -0.3826834
        place_pose_msg.pose.orientation.z = 0.0
        place_pose_msg.pose.orientation.w = 0.0
        place_pose_msg.pose.position.x = world_coords[0]
        place_pose_msg.pose.position.y = world_coords[1]
        place_pose_msg.pose.position.z = world_coords[2]
        
        # preplace pose
        panda_arm.set_start_state_to_current_state()
        pre_place_pose_msg = deepcopy(place_pose_msg)
        pre_place_pose_msg.pose.position.z += 0.1
        panda_arm.set_goal_state(pose_stamped_msg=pre_place_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # place pose
        panda_arm.set_start_state_to_current_state()
        panda_arm.set_goal_state(pose_stamped_msg=place_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # open gripper
        gripper_client.open_gripper()
        time.sleep(2.0)
        
        # raise arm
        panda_arm.set_start_state_to_current_state()
        pre_place_pose_msg.pose.position.z += 0.2
        panda_arm.set_goal_state(pose_stamped_msg=pre_place_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        self.mode = "pick"




