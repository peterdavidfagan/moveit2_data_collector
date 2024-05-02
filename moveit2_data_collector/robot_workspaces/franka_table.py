"""
A basic dm environment for transporter data collection on the Franka robot.
"""
import time
from typing import Dict
from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation as R
import dm_env

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.logging import get_logger

from moveit.planning import MoveItPy, PlanRequestParameters, MultiPipelinePlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import GripperCommand

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

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
        raise RuntimeError("Failed to plan trajectory")

    time.sleep(sleep_time)


class GripperClient(Node):

    def __init__(self, gripper_controller):
        super().__init__("gripper_client")
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand, 
            gripper_controller,
        )
    
    def close_gripper(self):
        goal = GripperCommand.Goal()
        goal.command.position = 0.8
        goal.command.max_effort = 3.0
        self.gripper_action_client.wait_for_server()
        return self.gripper_action_client.send_goal_async(goal)

    def open_gripper(self):
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 3.0
        self.gripper_action_client.wait_for_server()
        return self.gripper_action_client.send_goal_async(goal)

class FrankaTable(dm_env.Environment):
    """
    This dm_env is intended to be used in conjunction with PyQt data collection application. This environment abstraction is intended to make data collection compatible with env_logger.
    """

    def __init__(self, args):
        robot_ip = args.robot_ip
        use_gripper = args.use_gripper
        use_fake_hardware = args.use_fake_hardware
        self.robotiq_tcp_z_offset = 0.17 # TODO: consider moving to franka_robotiq launch file as static tf
        
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
        self.planning_scene_monitor = self.panda.get_planning_scene_monitor()
        self.panda_arm = self.panda.get_planning_component("panda_arm") 
        self.gripper_client = GripperClient(args.gripper_controller)

        # add ground plane
        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "panda_link0"
            collision_object.id = "ground_plane"

            box_pose = Pose()
            box_pose.position.x = 0.0
            box_pose.position.y = 0.0
            box_pose.position.z = 0.0

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [2.0, 2.0, 0.001]

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
       
            # finally handle the allowed collisions for the object
            scene.allowed_collision_matrix.set_entry("ground_plane", "panda_link0", True)
            scene.allowed_collision_matrix.set_entry("ground_plane", "panda_link1", True)
            scene.allowed_collision_matrix.set_entry("ground_plane", "robotiq_85_left_finger_tip_link", True)
            scene.allowed_collision_matrix.set_entry("ground_plane", "robotiq_85_right_finger_tip_link", True)

            scene.current_state.update()  # Important to ensure the scene is updated

        self.mode="pick"
        self.current_observation = None

    def reset(self) -> dm_env.TimeStep:        
        # return to home state
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name="ready")
        plan_and_execute(self.panda, self.panda_arm, sleep_time=1.0)
        
        # open gripper
        self.gripper_client.open_gripper()
        self.mode = "pick"

        return dm_env.TimeStep(
                step_type=dm_env.StepType.FIRST,
                reward=0.0,
                discount=0.0,
                observation=deepcopy(self.current_observation),
                )
    
    def done_step(self) -> dm_env.TimeStep:
        return dm_env.TimeStep(
                step_type=dm_env.StepType.LAST,
                reward=0.0,
                discount=0.0,
                observation=deepcopy(self.current_observation),
                )

    def step(self, action_dict) -> dm_env.TimeStep:
        if self.mode == "pick":
            self.pick(action_dict["pose"])
        else:
            self.place(action_dict["pose"])

        return dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=0.0,
                discount=0.0,
                observation=deepcopy(self.current_observation),
                )

    def set_observation(self, rgb, depth):
        self.current_observation = {
            "overhead_camera/rgb": rgb,
            "overhead_camera/depth": depth,
        }

    def set_metadata(self, config):
        self.metadata = config

    def observation_spec(self) -> Dict[str, dm_env.specs.Array]:
        return {
                "overhead_camera/rgb": dm_env.specs.Array(shape=(621,1104, 3), dtype=np.float32),
                "overhead_camera/depth": dm_env.specs.Array(shape=(621, 1104), dtype=np.float32),
                }

    def action_spec(self) -> dm_env.specs.Array:
        return {
                "pose": dm_env.specs.Array(shape=(7,), dtype=np.float64), # [x, y, z, qx, qy, qz, qw]
                "pixel_coords": dm_env.specs.Array(shape=(2,), dtype=np.int), # [u, v]
                "gripper_rot": dm_env.specs.Array(shape=(1,), dtype=np.float64),
                }
    
    def reward_spec(self) -> dm_env.specs.Array:
        return dm_env.specs.Array(
                shape=(),
                dtype=np.float64,
                )

    def discount_spec(self) -> dm_env.specs.Array:
        return dm_env.specs.Array(
                shape=(),
                dtype=np.float64,
                )

    def close(self):
        print("closing")

    def pick(self, pose):
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
            self.panda, ["pilz_lin", "pilz_ptp", "ompl_rrtc"]
        )

        pick_pose_msg = PoseStamped()
        pick_pose_msg.header.frame_id = "panda_link0"
        pick_pose_msg.pose.position.x = pose[0]
        pick_pose_msg.pose.position.y = pose[1]
        pick_pose_msg.pose.position.z = pose[2] + self.robotiq_tcp_z_offset
        pick_pose_msg.pose.orientation.x = pose[3]
        pick_pose_msg.pose.orientation.y = pose[4]
        pick_pose_msg.pose.orientation.z = pose[5]
        pick_pose_msg.pose.orientation.w = pose[6]
        
        self.pick_height=pose[2] # set this variable so it can be referenced in place motion

        # prepick pose
        self.panda_arm.set_start_state_to_current_state()
        pre_pick_pose_msg = deepcopy(pick_pose_msg)
        pre_pick_pose_msg.pose.position.z = 0.6
        self.panda_arm.set_goal_state(pose_stamped_msg=pre_pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # pick pose
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # close gripper
        self.gripper_client.close_gripper()
        time.sleep(3.0)
        
        # prepick arm
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=pre_pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # go to ready position
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name="ready")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # switch mode to place
        self.mode = "place"

    def place(self, pose):
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
            self.panda, ["pilz_lin", "pilz_ptp", "ompl_rrtc"]
        )

        place_pose_msg = PoseStamped()
        place_pose_msg.header.frame_id = "panda_link0"
        place_pose_msg.pose.position.x = pose[0]
        place_pose_msg.pose.position.y = pose[1]
        place_pose_msg.pose.position.z = self.pick_height + self.robotiq_tcp_z_offset
        place_pose_msg.pose.orientation.x = pose[3]
        place_pose_msg.pose.orientation.y = pose[4]
        place_pose_msg.pose.orientation.z = pose[5]
        place_pose_msg.pose.orientation.w = pose[6]
        
        # preplace pose
        self.panda_arm.set_start_state_to_current_state()
        pre_place_pose_msg = deepcopy(place_pose_msg)
        pre_place_pose_msg.pose.position.z = 0.6
        self.panda_arm.set_goal_state(pose_stamped_msg=pre_place_pose_msg, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # place pose
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=place_pose_msg, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # open gripper
        self.gripper_client.open_gripper()
        time.sleep(3.0)
        
        # preplace arm
        self.panda_arm.set_start_state_to_current_state()
        pre_place_pose_msg = deepcopy(place_pose_msg)
        pre_place_pose_msg.pose.position.z = 0.6
        self.panda_arm.set_goal_state(pose_stamped_msg=pre_place_pose_msg, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # go to ready position
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name="ready")
        plan_and_execute(self.panda, self.panda_arm, multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=0.5)

        # switch mode to pick
        self.mode = "pick"


