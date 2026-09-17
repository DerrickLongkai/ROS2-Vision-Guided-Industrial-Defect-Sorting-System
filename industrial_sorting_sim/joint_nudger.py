import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


UR_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class JointNudger(Node):

    def __init__(self):
        super().__init__('joint_nudger')

        self.current_joints = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def joint_state_callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))

        if all(j in joint_map for j in UR_JOINTS):

            self.current_joints = {
                j: joint_map[j]
                for j in UR_JOINTS
            }

    def wait_for_state(self):

        print('Waiting for /joint_states...')

        while (
            rclpy.ok()
            and self.current_joints is None
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        print('Joint states received.')

    def show(self):

        print('\n========================================')
        print(' CURRENT UR5 JOINT VALUES')
        print('========================================')

        for i, joint in enumerate(
            UR_JOINTS,
            start=1
        ):

            rad = self.current_joints[joint]
            deg = math.degrees(rad)

            print(
                f'{i}. {joint:<22} '
                f'{deg:8.2f} deg   '
                f'{rad:8.4f} rad'
            )

        print('========================================\n')

    def move_joint(self, joint_index, delta_deg):

        joint_name = UR_JOINTS[joint_index]

        target = self.current_joints.copy()

        delta_rad = math.radians(delta_deg)

        target[joint_name] += delta_rad

        print(
            f'\nMoving {joint_name}: '
            f'{delta_deg:+.2f}°'
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = UR_JOINTS

        point = JointTrajectoryPoint()

        point.positions = [
            target[j]
            for j in UR_JOINTS
        ]

        # Slow, controlled movement
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        goal.trajectory.points.append(point)

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

        if not goal_handle.accepted:

            print('Controller rejected goal.')
            return

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result().result

        if result.error_code == 0:

            # Refresh joint state
            for _ in range(10):
                rclpy.spin_once(
                    self,
                    timeout_sec=0.05
                )

            print('Movement successful.')
            self.show()

        else:

            print(
                'Movement failed. '
                f'Error code: {result.error_code}'
            )

            if result.error_string:
                print(result.error_string)


def main(args=None):

    rclpy.init(args=args)

    node = JointNudger()

    if not node.trajectory_client.wait_for_server(
        timeout_sec=10.0
    ):
        print(
            'joint_trajectory_controller '
            'is not available.'
        )

        node.destroy_node()
        rclpy.shutdown()
        return

    node.wait_for_state()
    node.show()

    print('Commands:')
    print('  show')
    print('  <joint number> <degrees>')
    print('  q')
    print()
    print('Examples:')
    print('  1 +2')
    print('  2 -1')
    print('  3 +5')
    print()

    while rclpy.ok():

        try:

            command = input('joint> ').strip()

            if command.lower() in [
                'q',
                'quit',
                'exit'
            ]:
                break

            if command.lower() == 'show':

                for _ in range(5):
                    rclpy.spin_once(
                        node,
                        timeout_sec=0.05
                    )

                node.show()
                continue

            parts = command.split()

            if len(parts) != 2:

                print(
                    'Use: <joint number> <degrees>'
                )

                continue

            joint_number = int(parts[0])
            delta_deg = float(parts[1])

            if not 1 <= joint_number <= 6:

                print(
                    'Joint number must be 1-6.'
                )

                continue

            # Safety: don't accidentally move huge amount
            if abs(delta_deg) > 10:

                print(
                    'Maximum one-step movement is 10°.'
                )

                continue

            node.move_joint(
                joint_number - 1,
                delta_deg
            )

        except ValueError:

            print(
                'Example command: 2 -2'
            )

        except KeyboardInterrupt:
            break

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
