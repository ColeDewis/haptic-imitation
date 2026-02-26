#!/usr/bin/env python3
import collections

import cv_bridge
import message_filters
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from il_recorder.pointcloud_utils import flat_pc_from_ros, idp3_preprocess_point_cloud
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from wam_msgs.msg import RTCartPose

# policy code
from .HumanScoredFlowMatching.flow_policy.train import TrainDP3Workspace

# --- CONFIGURATION ---
HISTORY_LENGTH = 2  # How many past steps to condition on (RTC)
EXECUTION_HORIZON = 8  # How many steps we buffer/execute
CONTROL_RATE = 15.0  # Hz


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class FlowInferenceNode(Node):
    def __init__(self):
        super().__init__("flow_inference_node")

        # Load model
        ckpt_path = "/home/user/wam-ros2-docker/wam_ws/src/wam_flow/ckpts/epoch=0300-test_mean_score=-0.047.ckpt"
        self.get_logger().info(f"Loading checkpoint: {ckpt_path}")

        self.workspace = TrainDP3Workspace.create_from_checkpoint(ckpt_path)
        self.workspace.model.eval()
        self.workspace.model.cuda()

        # RTC Buffers
        self.obs_window_size = 2
        self.states_deque = collections.deque(maxlen=self.obs_window_size)
        self.pcd_deque = collections.deque(maxlen=self.obs_window_size)

        # Action Queues
        self.action_buffer = []
        self.action_history = collections.deque(maxlen=HISTORY_LENGTH)

        # ROS
        cb_group = ReentrantCallbackGroup()

        self.pose_callback = self.create_subscription(
            PoseStamped,
            "/wam/ToolPose",
            self.feedback_callback,
            10,
            callback_group=cb_group,
        )

        self.sub_pts = message_filters.Subscriber(
            self, PointCloud2, "/camera1/depth/color/points"
        )
        self.sub_joints = message_filters.Subscriber(self, JointState, "/joint_states")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_joints, self.sub_pts],
            queue_size=2,
            slop=0.1,
            allow_headerless=True,
        )
        self.ts.registerCallback(self.infer_callback)

        # Robot Command
        self.pub_pose = self.create_publisher(RTCartPose, "/wam/RTCartPose", 10)

        # --- 4. CONTROL LOOP TIMER ---
        self.timer = self.create_timer(
            1.0 / CONTROL_RATE, self.control_loop, callback_group=cb_group
        )

        self.get_logger().info("Flow Inference Node Ready.")

    def feedback_callback(self, msg: PoseStamped):
        """Updates the robot's current pose."""

        xyz = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
        )
        rpy = R.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        ).as_euler("xyz")

        self.last_pos = np.concatenate([xyz, rpy])

    def infer_callback(self, joint_msg, pc_msg):
        # If we have no history, we wait for our current pose to fill with.
        if len(self.action_history) == 0:
            if self.last_pos is not None:
                self.get_logger().info("Seeding History with Current Pose...")

                for _ in range(HISTORY_LENGTH):
                    self.action_history.append(self.last_pos.copy())
            else:
                return  # Wait for feedback callback

        # Skip inference if buffer is full
        if len(self.action_buffer) > 2:
            return

        # Get and process point cloud
        pcloud = flat_pc_from_ros(pc_msg, remove_nans=True)
        pcloud = idp3_preprocess_point_cloud(
            pcloud, num_points=4096, near=0.1, far=1.5
        )  # TODO confirm near, far

        # gripper_pos = 0.0 if joint_msg.position[7] < 0.1 else 1.0
        # joints = np.concatenate([joint_msg.position[:7], [gripper_pos]])
        joints = np.array(joint_msg.position[:7], dtype=np.float32)

        # NOTE these have maxlen so they will automatically pop old entries
        self.states_deque.append(joints)
        self.pcd_deque.append(pcloud)

        if len(self.states_deque) < self.obs_window_size:
            return

        # Prepare Tensors
        state_input = np.array(self.states_deque)
        pc_input = np.array(self.pcd_deque)

        data = {
            "obs": {
                "agent_pos": torch.from_numpy(state_input).unsqueeze(0).cuda().float(),
                "point_cloud": torch.from_numpy(pc_input).unsqueeze(0).cuda().float(),
            }
        }

        # 6. Condition on History (RTC)
        past_actions_arr = np.array(self.action_history)
        past_actions_tensor = (
            torch.from_numpy(past_actions_arr).unsqueeze(0).cuda().float()
        )

        # 7. Inference
        with torch.no_grad():
            result = self.workspace.model.predict_action(
                data, past_actions=past_actions_tensor
            )
            raw_action_seq = result["action"].squeeze(0).cpu().numpy()

        # 8. RTC Slicing
        # Skip the first HISTORY_LENGTH steps (reconstructed past)
        # Keep the future steps
        future_actions = raw_action_seq[HISTORY_LENGTH:]

        self.action_buffer = []
        self.action_buffer.extend(future_actions[:EXECUTION_HORIZON])

    def control_loop(self):
        if not self.action_buffer:
            return

        # 1. Pop Action
        target_pose = self.action_buffer.pop(0)

        # 2. Update History (for future conditioning)
        # NOTE again a deque with maxlen so we keep only latest HISTORY_LENGTH entries
        self.action_history.append(target_pose)

        target_msg = RTCartPose()
        target_msg.point.x = target_pose[0]
        target_msg.point.y = target_pose[1]
        target_msg.point.z = target_pose[2]
        rpy = target_pose[3:6]
        rpy = wrap_to_pi(rpy)
        quat = R.from_euler("xyz", rpy).as_quat()
        target_msg.orientation.x = quat[0]
        target_msg.orientation.y = quat[1]
        target_msg.orientation.z = quat[2]
        target_msg.orientation.w = quat[3]

        target_msg.position_rate_limits = [0.1, 0.1, 0.1]  # m/s
        target_msg.orientation_rate_limits = [0.5, 0.5, 0.5]  # rad/s

        self.pub_pose.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlowInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Stopping...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    main()
