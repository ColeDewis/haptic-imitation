from time import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Empty, Trigger
from wam_msgs.msg import (
    RTCartPose,
    RTJointPositions,
    RTJointVelocities,
    RTLinearandAngularVelocity,
)
from wam_srvs.srv import CartPoseMove, JointMove

# WAM PUBLISHERS:
# "/joint_states", JointState
# "wam/trajectoryStatus", Bool
# "/wam/jointVelocity", JointState
# "/wam/ToolPose", PoseStamped
# "/wam/toolVelocity", TwistStamped

# WAM SUBSCRIBERS:
# "/wam/RTJointPositionCMD", RTJointPositions
# "/wam/RTLinearandAngularVelocityCMD", RTLinearandAngularVelocity
# "/wam/RTAngularVelocityCMD", RTAngularVelocity
# "/wam/RTLinearVelocityCMD", RTLinearVelocity
# "/wam/RTCartPositionCMD", RTCartPosition
# "/wam/RTCartPoseCMD", RTCartPose
# "/wam/RTCartOrientationCMD", RTCartOrientation
# "/wam/RTJointVelocityCMD", RTJointVelocities

# WAM SERVICES:
# "/wam/moveHome", Trigger
# "/wam/gravityCompensate", SetBool
# "/wam/idle", Trigger;
# "/wam/holdCartPosition", SetBool
# "/wam/holdJointPosition", SetBool
# "/wam/holdCartOrientation", SetBool
# "/wam/holdCartPose", SetBool
# "/wam/moveToJointPosition", JointMove
# "/wam/moveToCartPosition", CartPositionMove
# "/wam/moveToCartOrientation", CartOrientationMove
# "/wam/moveToCartPose", CartPoseMove
# "/wam/setVelocityLimit" VelocityLimit>


class WamXboxControl(Node):
    def __init__(self):
        super().__init__("wam_xbox_control")

        self.joy = None
        self.current_pose = None

        qos_fast = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_cart_vel = self.create_publisher(
            RTLinearandAngularVelocity, "/wam/RTLinearandAngularVelocityCMD", qos_fast
        )
        self.pub_cart_pos = self.create_publisher(
            RTCartPose, "/wam/RTCartPoseCMD", qos_fast
        )
        # self.pub_gripper = None # TODO, test later, figure out if barrett's works or if i need to wrap dylan's

        self.sub_joy = self.create_subscription(Joy, "joy", self.joy_callback, qos_fast)
        self.sub_pose = self.create_subscription(
            PoseStamped, "/wam/ToolPose", self.pose_callback, qos_fast
        )
        self.FREQ = 50  # Hz
        self.create_timer(1.0 / self.FREQ, self.control_loop)

        self.get_clock().sleep_for(
            Duration(seconds=1.0)
        )  # Wait for publishers/subscribers to initialize
        self.get_logger().info("Xbox Control Node Started. Listening for xbox input...")

        self.grip_width = 0.08

    def joy_callback(self, msg):
        self.joy = msg

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def control_loop(self):
        if not self.joy or not self.current_pose:
            return

        # 1. Cartesian (Left Stick + Bumpers)
        x = self.joy.axes[1] * 0.05  # Left Stick Y -> X
        y = self.joy.axes[0] * 0.05  # Left Stick X -> -Y
        z = (1 / (self.joy.axes[5] + 1.1) - 1 / (self.joy.axes[2] + 1.1)) / 10

        # 2. Rotation (Right Stick) -> Yaw/Pitch
        yaw = self.joy.axes[3] * 0.1
        pitch = self.joy.axes[4] * 0.1
        roll = (self.joy.buttons[1] - self.joy.buttons[2]) * 0.1  # B(1) - X(2)

        # TODO: probably can do only one conversion by doing everything in
        # quaternion space?
        curr_rpy = Rotation.from_quat(
            [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ]
        ).as_euler("xyz")

        target_rpy = curr_rpy + np.array([roll, pitch, yaw])
        target_quat = Rotation.from_euler("xyz", target_rpy).as_quat()

        LINEAR_GAIN = 1 / 10
        target_xyz = Point()
        target_xyz.x = self.current_pose.position.x + LINEAR_GAIN * x
        target_xyz.y = self.current_pose.position.y + LINEAR_GAIN * y
        target_xyz.z = self.current_pose.position.z + LINEAR_GAIN * z
        target = RTCartPose()
        target.orientation = Quaternion(
            x=target_quat[0],
            y=target_quat[1],
            z=target_quat[2],
            w=target_quat[3],
        )
        target.point = target_xyz
        target.orientation_rate_limits = [0.15, 0.15, 0.15, 0.15]
        target.position_rate_limits = [0.05, 0.05, 0.05]

        self.get_logger().info(f"Publishing EULER: {target_rpy}")
        self.get_logger().info(
            f"Publishing EULER: {target_xyz.x}, {target_xyz.y}, {target_xyz.z}"
        )
        self.pub_cart_pos.publish(target)

        # NOTE: below is velocity control: it does work, but the orientation control
        # is very slow to react, not really sure why. Perhaps gains are bad somewhere,
        # or I need to change the source a bit. Will use position instead as above.
        # msg = RTLinearandAngularVelocity()
        # linear_norm = np.linalg.norm([x, y, z])
        # linear_dir = (
        #     np.array([x, y, z]) / linear_norm if linear_norm > 0 else np.zeros(3)
        # )
        # angular_norm = np.linalg.norm([roll, pitch, yaw])
        # angular_dir = (
        #     np.array([roll, pitch, yaw]) / angular_norm
        #     if angular_norm > 0
        #     else np.zeros(3)
        # )

        # # safety limits
        # linear_norm = min(linear_norm, 0.05)
        # angular_norm = min(angular_norm, 0.5)

        # msg.linear_velocity_direction = linear_dir
        # msg.linear_velocity_magnitude = linear_norm
        # msg.angular_velocity_direction = angular_dir
        # msg.angular_velocity_magnitude = angular_norm

        # self.get_logger().info(
        #     f"Publishing cart vel: {linear_dir * linear_norm}, {angular_dir * angular_norm}"
        # )
        # self.pub_cart_vel.publish(msg)

        # 3. Gripper (Triggers)
        # TODO test w bhand later.
        if self.joy.axes[4] < -0.5:
            # open
            pass
        elif self.joy.axes[5] < -0.5:
            # close
            pass


def main(args=None):
    rclpy.init(args=args)
    control = WamXboxControl()

    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        pass
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
