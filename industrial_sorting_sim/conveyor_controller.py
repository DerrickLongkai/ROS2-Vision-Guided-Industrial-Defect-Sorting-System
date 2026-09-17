import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import Entity


PRODUCT_X = 0.50
PRODUCT_Z = 0.52

SOURCE_Y = -0.90
PICK_Y = 0.10
EXIT_Y = 0.90

CONVEYOR_SPEED = 0.20
UPDATE_PERIOD = 0.05

PRODUCT_SEQUENCE = ['RED', 'GREEN', 'BLUE']
NEXT_PRODUCT_DELAY = 1.0


def make_static_cube_sdf(model_name, color):

    rgba = {
        'RED': '1 0 0 1',
        'GREEN': '0 1 0 1',
        'BLUE': '0 0 1 1',
    }.get(color, '0.7 0.7 0.7 1')

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.04 0.04 0.04</size>
          </box>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ConveyorController(Node):

    def __init__(self):

        super().__init__('conveyor_controller')

        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/sorting_world/set_pose'
        )

        self.spawn_client = self.create_client(
            SpawnEntity,
            '/world/sorting_world/create'
        )

        self.delete_client = self.create_client(
            DeleteEntity,
            '/world/sorting_world/remove'
        )

        self.pick_ready_pub = self.create_publisher(
            Bool,
            '/blue_pick_ready',
            10
        )

        # Robot calls this after BLUE has been rejected
        # and the UR5 has safely returned to PRE_GRASP.
        self.rejection_done_service = self.create_service(
            Trigger,
            '/blue_rejection_done',
            self.blue_rejection_done_callback
        )

        self.sequence_index = 0

        self.current_color = None
        self.current_name = None
        self.current_y = SOURCE_Y

        self.moving = False
        self.spawn_in_progress = False
        self.delete_in_progress = False
        self.waiting_for_blue_robot = False

        self.pending_pose_request = None

        self.next_spawn_time = (
            self.get_clock().now()
            + Duration(seconds=1.0)
        )

        self.publish_blue_ready(False)

        self.timer = self.create_timer(
            UPDATE_PERIOD,
            self.update
        )

        self.get_logger().info(
            'Automatic Conveyor Controller started.'
        )

        self.get_logger().info(
            f'Flow: LEFT -> RIGHT '
            f'(Y {SOURCE_Y:.2f} -> {EXIT_Y:.2f})'
        )

        self.get_logger().info(
            f'BLUE PICK Y = {PICK_Y:.2f}'
        )

        self.get_logger().info(
            'Sequence: RED -> GREEN -> BLUE'
        )

    def publish_blue_ready(self, ready):

        msg = Bool()
        msg.data = bool(ready)

        self.pick_ready_pub.publish(msg)

    def model_name_for_color(self, color):

        if color == 'RED':
            return 'red_object'

        if color == 'GREEN':
            return 'green_object'

        if color == 'BLUE':
            return 'blue_object'

        return 'product_object'

    def spawn_next_product(self):

        if self.spawn_in_progress:
            return

        if self.sequence_index >= len(PRODUCT_SEQUENCE):
            return

        color = PRODUCT_SEQUENCE[self.sequence_index]
        name = self.model_name_for_color(color)

        if not self.spawn_client.service_is_ready():
            return

        request = SpawnEntity.Request()

        request.entity_factory.name = name
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf = make_static_cube_sdf(
            name,
            color
        )
        request.entity_factory.relative_to = 'world'

        request.entity_factory.pose.position.x = PRODUCT_X
        request.entity_factory.pose.position.y = SOURCE_Y
        request.entity_factory.pose.position.z = PRODUCT_Z
        request.entity_factory.pose.orientation.w = 1.0

        self.spawn_in_progress = True

        self.get_logger().info(
            '====================================='
        )
        self.get_logger().info(
            f'SPAWNING {color} at LEFT entrance.'
        )

        future = self.spawn_client.call_async(request)

        future.add_done_callback(
            lambda future,
            color=color,
            name=name:
            self.spawn_finished(
                future,
                color,
                name
            )
        )

    def spawn_finished(self, future, color, name):

        self.spawn_in_progress = False

        try:
            result = future.result()

        except Exception as ex:
            self.get_logger().error(
                f'Spawn service failed: {ex}'
            )
            return

        if not result.success:
            self.get_logger().error(
                f'Could not spawn {color}.'
            )
            return

        self.current_color = color
        self.current_name = name
        self.current_y = SOURCE_Y

        self.moving = True
        self.delete_in_progress = False

        self.publish_blue_ready(False)

        self.get_logger().info(
            f'{color} entered conveyor at '
            f'Y={SOURCE_Y:.2f}.'
        )
        self.get_logger().info(
            'CONVEYOR MOVING ->'
        )
        self.get_logger().info(
            '====================================='
        )

    def set_current_pose(self, y):

        if self.current_name is None:
            return False

        if not self.pose_client.service_is_ready():
            return False

        if (
            self.pending_pose_request is not None
            and not self.pending_pose_request.done()
        ):
            return False

        request = SetEntityPose.Request()

        request.entity.name = self.current_name
        request.entity.type = Entity.MODEL

        request.pose.position.x = PRODUCT_X
        request.pose.position.y = y
        request.pose.position.z = PRODUCT_Z
        request.pose.orientation.w = 1.0

        self.pending_pose_request = (
            self.pose_client.call_async(request)
        )

        return True

    def delete_current_product(self):

        if self.delete_in_progress:
            return

        if self.current_name is None:
            return

        if not self.delete_client.service_is_ready():
            return

        self.delete_in_progress = True
        self.moving = False

        color = self.current_color
        name = self.current_name

        self.get_logger().info(
            f'{color} reached RIGHT exit.'
        )
        self.get_logger().info(
            f'{color} PASS -> removing product.'
        )

        request = DeleteEntity.Request()

        request.entity.name = name
        request.entity.type = Entity.MODEL

        future = self.delete_client.call_async(request)

        future.add_done_callback(
            lambda future,
            color=color:
            self.delete_finished(
                future,
                color
            )
        )

    def delete_finished(self, future, color):

        self.delete_in_progress = False

        try:
            result = future.result()

        except Exception as ex:
            self.get_logger().error(
                f'Delete service failed: {ex}'
            )
            return

        if not result.success:
            self.get_logger().error(
                f'Could not remove {color}.'
            )
            return

        self.get_logger().info(
            f'{color} removed at RIGHT exit.'
        )

        self.sequence_index = (
            self.sequence_index + 1
        ) % len(PRODUCT_SEQUENCE)

        self.current_color = None
        self.current_name = None
        self.current_y = SOURCE_Y

        self.next_spawn_time = (
            self.get_clock().now()
            + Duration(
                seconds=NEXT_PRODUCT_DELAY
            )
        )

    def stop_blue_at_pick_point(self):

        self.current_y = PICK_Y
        self.set_current_pose(PICK_Y)

        self.moving = False
        self.waiting_for_blue_robot = True

        self.publish_blue_ready(True)

        self.get_logger().warn(
            '====================================='
        )
        self.get_logger().warn(
            'BLUE reached calibrated PICK POINT.'
        )
        self.get_logger().warn(
            f'Exact position: '
            f'({PRODUCT_X:.2f}, '
            f'{PICK_Y:.2f}, '
            f'{PRODUCT_Z:.2f})'
        )
        self.get_logger().warn(
            'CONVEYOR STOPPED.'
        )
        self.get_logger().warn(
            'Waiting for UR5 rejection.'
        )
        self.get_logger().warn(
            '====================================='
        )

    def blue_rejection_done_callback(
        self,
        request,
        response
    ):

        if not self.waiting_for_blue_robot:

            response.success = False
            response.message = (
                'No BLUE product is waiting for robot completion.'
            )

            return response

        # The original STATIC blue_object has already been
        # removed by grasp_controller during RELEASE.
        self.waiting_for_blue_robot = False
        self.moving = False

        self.publish_blue_ready(False)

        self.sequence_index = (
            self.sequence_index + 1
        ) % len(PRODUCT_SEQUENCE)

        self.current_color = None
        self.current_name = None
        self.current_y = SOURCE_Y

        self.next_spawn_time = (
            self.get_clock().now()
            + Duration(
                seconds=NEXT_PRODUCT_DELAY
            )
        )

        self.get_logger().info(
            '====================================='
        )

        self.get_logger().info(
            'BLUE rejection cycle completed.'
        )

        self.get_logger().info(
            'Conveyor line released for next product.'
        )

        self.get_logger().info(
            'Next cycle will continue automatically.'
        )

        self.get_logger().info(
            '====================================='
        )

        response.success = True
        response.message = (
            'BLUE rejection acknowledged. '
            'Next product scheduled.'
        )

        return response

    def update(self):

        if self.current_name is None:

            if (
                not self.waiting_for_blue_robot
                and not self.spawn_in_progress
                and self.sequence_index
                < len(PRODUCT_SEQUENCE)
                and self.get_clock().now()
                >= self.next_spawn_time
            ):
                self.spawn_next_product()

            return

        if self.waiting_for_blue_robot:

            # Re-publish continuously while BLUE is waiting.
            # This makes the trigger robust even if the robot
            # controller starts late or misses the first message.
            self.publish_blue_ready(True)

            return

        if not self.moving:
            return

        if (
            self.pending_pose_request is not None
            and not self.pending_pose_request.done()
        ):
            return

        distance = (
            CONVEYOR_SPEED
            * UPDATE_PERIOD
        )

        next_y = (
            self.current_y
            + distance
        )

        if (
            self.current_color == 'BLUE'
            and next_y >= PICK_Y
        ):
            self.stop_blue_at_pick_point()
            return

        if (
            self.current_color in (
                'RED',
                'GREEN'
            )
            and next_y >= EXIT_Y
        ):
            self.current_y = EXIT_Y
            self.delete_current_product()
            return

        self.current_y = next_y
        self.set_current_pose(
            self.current_y
        )


def main(args=None):

    rclpy.init(args=args)

    node = ConveyorController()

    node.get_logger().info(
        'Waiting for Gazebo services...'
    )

    while not node.pose_client.wait_for_service(
        timeout_sec=1.0
    ):
        node.get_logger().info(
            'Waiting for set_pose...'
        )

    while not node.spawn_client.wait_for_service(
        timeout_sec=1.0
    ):
        node.get_logger().info(
            'Waiting for create...'
        )

    while not node.delete_client.wait_for_service(
        timeout_sec=1.0
    ):
        node.get_logger().info(
            'Waiting for remove...'
        )

    node.get_logger().info(
        'Gazebo services connected.'
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

