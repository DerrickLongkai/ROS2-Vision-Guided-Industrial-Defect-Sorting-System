import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String, Bool


# ==========================================================
# UR5 joint order
# ==========================================================

UR_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


# ==========================================================
# Recorded safe waypoints
# ==========================================================

PRE_GRASP = {
    'shoulder_pan_joint': 0.0000,
    'shoulder_lift_joint': -1.3959,
    'elbow_joint': 0.6087,
    'wrist_1_joint': -0.9954,
    'wrist_2_joint': -1.6366,
    'wrist_3_joint': 0.0000,
}


PICK = {
    'shoulder_pan_joint': 0.0000,
    'shoulder_lift_joint': -1.4079,
    'elbow_joint': 0.7478,
    'wrist_1_joint': -1.0168,
    'wrist_2_joint': -1.6366,
    'wrist_3_joint': 0.0000,
}


# Lift back to the safe pre-grasp position
LIFT = PRE_GRASP


# ==========================================================
# Safe BLUE reject-bin route
#
# LIFT -> TRANSIT_HIGH -> ROTATE_BLUE_HIGH -> PLACE_BLUE
# ==========================================================

TRANSIT_HIGH = {
    'shoulder_pan_joint': 0.3486,
    'shoulder_lift_joint': -1.5701,
    'elbow_joint': 0.4344,
    'wrist_1_joint': -1.8653,
    'wrist_2_joint': -1.6366,
    'wrist_3_joint': 0.0000,
}


ROTATE_BLUE_HIGH = {
    'shoulder_pan_joint': 1.4628,
    'shoulder_lift_joint': -1.5701,
    'elbow_joint': 0.4344,
    'wrist_1_joint': -1.8653,
    'wrist_2_joint': -1.6366,
    'wrist_3_joint': 0.0000,
}


PLACE_BLUE = {
    'shoulder_pan_joint': 1.4617,
    'shoulder_lift_joint': -1.3088,
    'elbow_joint': 0.4345,
    'wrist_1_joint': -1.2570,
    'wrist_2_joint': -1.6366,
    'wrist_3_joint': 0.0000,
}


class SortingRobotController(Node):

    def __init__(self):

        super().__init__('sorting_robot_controller')

        # --------------------------------------------------
        # Direct ros2_control trajectory action
        # --------------------------------------------------

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # --------------------------------------------------
        # Simulated grasp service
        # --------------------------------------------------

        self.grasp_client = self.create_client(
            SetBool,
            '/grasp_blue'
        )

        # Notify conveyor after rejection + safe robot return.
        self.rejection_done_client = self.create_client(
            Trigger,
            '/blue_rejection_done'
        )

        # --------------------------------------------------
        # Automatic trigger inputs
        #
        # Condition A:
        #   /detected_color == BLUE
        #
        # Condition B:
        #   /blue_pick_ready == True
        #
        # Only when BOTH are true will the UR5 start.
        # --------------------------------------------------

        self.blue_detected = False
        self.blue_pick_ready = False

        self.color_sub = self.create_subscription(
            String,
            '/detected_color',
            self.color_callback,
            10
        )

        self.pick_ready_sub = self.create_subscription(
            Bool,
            '/blue_pick_ready',
            self.pick_ready_callback,
            10
        )

        self.get_logger().info(
            'Automatic Sorting Robot Controller started.'
        )

        self.get_logger().info(
            'TURBO+ MODE enabled: 1.5-second class trajectories '
            'with safe reject-bin waypoints preserved.'
        )

        self.get_logger().info(
            'Waiting for BLUE detection + calibrated pick position.'
        )

    # ======================================================
    # Vision / conveyor callbacks
    # ======================================================

    def color_callback(self, msg):

        # Latch BLUE once it has been positively detected.
        # We intentionally do not clear it if the camera later
        # publishes NONE while the same workpiece is moving.
        if msg.data == 'BLUE' and not self.blue_detected:

            self.blue_detected = True

            self.get_logger().warn(
                'BLUE defect confirmed by camera.'
            )

            if not self.blue_pick_ready:

                self.get_logger().info(
                    'Waiting for BLUE to reach PICK POINT...'
                )

    def pick_ready_callback(self, msg):

        # Latch TRUE for the current cycle.
        # The main loop clears this explicitly at the start
        # of the next cycle.
        if not msg.data:
            return

        if not self.blue_pick_ready:

            self.blue_pick_ready = True

            self.get_logger().info(
                'BLUE reached calibrated PICK POINT.'
            )

            if not self.blue_detected:

                self.get_logger().info(
                    'Waiting for camera to confirm BLUE...'
                )

    def automatic_trigger_ready(self):

        ready = (
            self.blue_detected
            and self.blue_pick_ready
        )

        return ready

    # ======================================================
    # Wait for controller + grasp service
    # ======================================================

    def wait_for_system(self):

        self.get_logger().info(
            'Waiting for joint_trajectory_controller...'
        )

        while not self.trajectory_client.wait_for_server(
            timeout_sec=1.0
        ):

            self.get_logger().info(
                'Still waiting for joint_trajectory_controller...'
            )

        self.get_logger().info(
            'Trajectory controller connected.'
        )

        self.get_logger().info(
            'Waiting for /grasp_blue...'
        )

        while not self.grasp_client.wait_for_service(
            timeout_sec=1.0
        ):

            self.get_logger().info(
                'Still waiting for /grasp_blue...'
            )

        self.get_logger().info(
            'Grasp service connected.'
        )

        self.get_logger().info(
            'Waiting for /blue_rejection_done...'
        )

        while not self.rejection_done_client.wait_for_service(
            timeout_sec=1.0
        ):

            self.get_logger().info(
                'Still waiting for /blue_rejection_done...'
            )

        self.get_logger().info(
            'Conveyor completion service connected.'
        )

        return True

    # ======================================================
    # Move directly to joint-space waypoint
    # ======================================================

    def move_to_joints(
        self,
        waypoint_name,
        waypoint,
        duration_sec=4
    ):

        self.get_logger().info(
            f'Moving to {waypoint_name}...'
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = UR_JOINTS

        point = JointTrajectoryPoint()

        point.positions = [
            waypoint[joint]
            for joint in UR_JOINTS
        ]

        # Give Gazebo enough time to follow.
        # Support fractional durations such as 1.5 s.
        whole_seconds = int(duration_sec)

        nanoseconds = int(
            (duration_sec - whole_seconds) * 1e9
        )

        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

        goal.trajectory.points.append(
            point
        )

        # Give Gazebo extra time to settle
        goal.goal_time_tolerance = Duration(
            sec=10,
            nanosec=0
        )

        # --------------------------------------------------
        # Send trajectory
        # --------------------------------------------------

        send_future = (
            self.trajectory_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None:

            self.get_logger().error(
                f'{waypoint_name}: no goal handle.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                f'{waypoint_name}: controller rejected goal.'
            )

            return False

        self.get_logger().info(
            f'{waypoint_name}: trajectory accepted.'
        )

        # --------------------------------------------------
        # Wait for completion
        # --------------------------------------------------

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:

            self.get_logger().error(
                f'{waypoint_name}: no trajectory result.'
            )

            return False

        result = wrapped_result.result

        if result.error_code == 0:

            self.get_logger().info(
                f'{waypoint_name} reached successfully.'
            )

            return True

        self.get_logger().error(
            f'{waypoint_name} failed. '
            f'Controller error code: '
            f'{result.error_code}'
        )

        if result.error_string:

            self.get_logger().error(
                result.error_string
            )

        return False

    # ======================================================
    # Simulated grasp
    # ======================================================

    def set_grasp(self, attached):

        request = SetBool.Request()
        request.data = attached

        if attached:

            self.get_logger().info(
                'ATTACHING BLUE object...'
            )

        else:

            self.get_logger().info(
                'RELEASING BLUE object into reject bin...'
            )

        future = self.grasp_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is None:

            self.get_logger().error(
                'No response from grasp_controller.'
            )

            return False

        if not response.success:

            self.get_logger().error(
                response.message
            )

            return False

        self.get_logger().info(
            response.message
        )

        return True

    # ======================================================
    # Notify conveyor that BLUE cycle is finished
    # ======================================================

    def notify_rejection_done(self):

        request = Trigger.Request()

        future = self.rejection_done_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is None:

            self.get_logger().error(
                'No response from /blue_rejection_done.'
            )

            return False

        if not response.success:

            self.get_logger().error(
                response.message
            )

            return False

        self.get_logger().info(
            response.message
        )

        return True

    # ======================================================
    # Automatic BLUE defect rejection sequence
    # ======================================================

    def run_pick_sequence(self, skip_pre_grasp=False):

        self.get_logger().info(
            '====================================='
        )

        self.get_logger().info(
            'Starting BLUE defect rejection cycle'
        )

        self.get_logger().info(
            '====================================='
        )

        # --------------------------------------------------
        # 1. PRE-GRASP
        #
        # First cycle: move into the calibrated wait pose.
        # Later cycles: the robot already returned here, so
        # skip this redundant movement to save several seconds.
        # --------------------------------------------------

        if not skip_pre_grasp:

            if not self.move_to_joints(
                'PRE_GRASP',
                PRE_GRASP,
                duration_sec=2.5
            ):
                return False

            time.sleep(0.1)

        else:

            self.get_logger().info(
                'Already at PRE_GRASP - skipping redundant move.'
            )

        # --------------------------------------------------
        # 2. PICK
        # --------------------------------------------------

        if not self.move_to_joints(
            'PICK',
            PICK,
            duration_sec=1.5
        ):
            return False

        time.sleep(0.1)

        # --------------------------------------------------
        # 3. ATTACH BLUE DEFECT
        # --------------------------------------------------

        if not self.set_grasp(True):
            return False

        time.sleep(0.1)

        # --------------------------------------------------
        # 4. LIFT
        # --------------------------------------------------

        if not self.move_to_joints(
            'LIFT',
            LIFT,
            duration_sec=1.5
        ):
            return False

        time.sleep(0.1)

        # --------------------------------------------------
        # 5. MOVE TO SAFE HIGH TRANSIT
        # --------------------------------------------------

        if not self.move_to_joints(
            'TRANSIT_HIGH',
            TRANSIT_HIGH,
            duration_sec=2.0
        ):
            return False

        time.sleep(0.1)

        # --------------------------------------------------
        # 6. ROTATE TOWARD REJECT BIN WHILE HIGH
        # --------------------------------------------------

        if not self.move_to_joints(
            'ROTATE_BLUE_HIGH',
            ROTATE_BLUE_HIGH,
            duration_sec=2.5
        ):
            return False

        time.sleep(0.1)

        # --------------------------------------------------
        # 7. LOWER TO REJECT BIN
        # --------------------------------------------------

        if not self.move_to_joints(
            'PLACE_BLUE',
            PLACE_BLUE,
            duration_sec=1.5
        ):
            return False

        time.sleep(0.1)

        # --------------------------------------------------
        # 8. RELEASE BLUE DEFECT
        #
        # grasp_controller deletes the STATIC cube and
        # respawns the DYNAMIC cube at the same pose.
        # Gazebo gravity then drops it into the reject bin.
        # --------------------------------------------------

        if not self.set_grasp(False):
            return False

        # Give delete -> respawn -> gravity time to complete.
        time.sleep(0.8)

        # --------------------------------------------------
        # 9. RETURN HIGH ABOVE REJECT BIN
        # --------------------------------------------------

        if not self.move_to_joints(
            'RETURN_ROTATE_HIGH',
            ROTATE_BLUE_HIGH,
            duration_sec=1.5
        ):
            return False

        time.sleep(0.10)

        # --------------------------------------------------
        # 10. RETURN TO SAFE TRANSIT
        # --------------------------------------------------

        if not self.move_to_joints(
            'RETURN_TRANSIT_HIGH',
            TRANSIT_HIGH,
            duration_sec=2.0
        ):
            return False

        time.sleep(0.10)

        # --------------------------------------------------
        # 11. RETURN TO WAIT / PRE-GRASP
        # --------------------------------------------------

        if not self.move_to_joints(
            'WAIT_PRE_GRASP',
            PRE_GRASP,
            duration_sec=1.5
        ):
            return False

        time.sleep(0.10)

        # --------------------------------------------------
        # 12. TELL CONVEYOR THIS BLUE CYCLE IS COMPLETE
        # --------------------------------------------------

        if not self.notify_rejection_done():
            return False

        self.get_logger().info(
            '====================================='
        )

        self.get_logger().info(
            'BLUE DEFECT REJECTION SUCCESSFUL!'
        )

        self.get_logger().info(
            'UR5 returned to WAIT position.'
        )

        self.get_logger().info(
            'Conveyor released for next cycle.'
        )

        self.get_logger().info(
            '====================================='
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = SortingRobotController()

    if not node.wait_for_system():

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        return

    try:

        node.get_logger().info(
            '====================================='
        )

        node.get_logger().info(
            'AUTO MODE ARMED'
        )

        node.get_logger().info(
            'Trigger requires:'
        )

        node.get_logger().info(
            '1) /detected_color = BLUE'
        )

        node.get_logger().info(
            '2) /blue_pick_ready = true'
        )

        node.get_logger().info(
            '====================================='
        )

        cycle_number = 1

        while rclpy.ok():

            node.get_logger().info(
                f'AUTO MODE ARMED - waiting for cycle {cycle_number}.'
            )

            # Reset cycle latch state.
            node.blue_detected = False
            node.blue_pick_ready = False

            # Wait for BOTH camera BLUE detection and
            # calibrated conveyor pick position.
            while (
                rclpy.ok()
                and not node.automatic_trigger_ready()
            ):

                rclpy.spin_once(
                    node,
                    timeout_sec=0.1
                )

            if not rclpy.ok():
                break

            node.get_logger().warn(
                '====================================='
            )

            node.get_logger().warn(
                f'AUTO TRIGGER: BLUE + PICK READY '
                f'(cycle {cycle_number})'
            )

            node.get_logger().warn(
                'Starting UR5 rejection sequence.'
            )

            node.get_logger().warn(
                '====================================='
            )

            success = node.run_pick_sequence(
                skip_pre_grasp=(cycle_number > 1)
            )

            if not success:

                node.get_logger().error(
                    'Automatic rejection cycle failed. '
                    'Stopping AUTO loop for safety.'
                )

                break

            cycle_number += 1

    except KeyboardInterrupt:

        node.get_logger().info(
            'Sequence interrupted.'
        )

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
