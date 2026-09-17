import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# Gazebo-ROS bridge interfaces for manipulating the simulation world
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import Entity


# --- Physical Environment Constants ---
PRODUCT_X = 0.50
PRODUCT_Z = 0.52

SOURCE_Y = -0.90      # Spawn point (Left side of conveyor)
PICK_Y = 0.10         # Calibrated interception point for the UR5 robot
EXIT_Y = 0.90         # Despawn point (Right side of conveyor)

CONVEYOR_SPEED = 0.20 # Meters per second
UPDATE_PERIOD = 0.05  # 20 Hz update rate for smooth kinematic animation

PRODUCT_SEQUENCE = ['RED', 'GREEN', 'BLUE']
NEXT_PRODUCT_DELAY = 1.0


def make_static_cube_sdf(model_name, color):
    """
    Dynamically generates the SDF (Simulation Description Format) XML string 
    for a colored cube. Injecting SDFs via code avoids maintaining multiple 
    static XML files for simple geometric variations.
    """
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
    """
    ROS 2 Node that orchestrates the factory sorting pipeline.
    It utilizes a "Kinematic Simulation" approach—manually updating the pose 
    of the products at a high frequency—which is often more stable and predictable 
    in robotics CI/CD pipelines than relying on physics engine friction.
    """

    def __init__(self):
        super().__init__('conveyor_controller')

        # --- Gazebo Service Clients ---
        # Connects to Gazebo to move, spawn, and delete models programmatically
        self.pose_client = self.create_client(SetEntityPose, '/world/sorting_world/set_pose')
        self.spawn_client = self.create_client(SpawnEntity, '/world/sorting_world/create')
        self.delete_client = self.create_client(DeleteEntity, '/world/sorting_world/remove')

        # --- Robot Handshake Interfaces ---
        # Publishes a signal when a defective (BLUE) product reaches the pick zone
        self.pick_ready_pub = self.create_publisher(Bool, '/blue_pick_ready', 10)

        # Service server: The UR5 robot calls this after it successfully disposes of the BLUE product
        self.rejection_done_service = self.create_service(
            Trigger,
            '/blue_rejection_done',
            self.blue_rejection_done_callback
        )

        # --- State Machine Variables ---
        self.sequence_index = 0
        self.current_color = None
        self.current_name = None
        self.current_y = SOURCE_Y

        self.moving = False
        self.spawn_in_progress = False
        self.delete_in_progress = False
        self.waiting_for_blue_robot = False

        self.pending_pose_request = None

        self.next_spawn_time = self.get_clock().now() + Duration(seconds=1.0)

        # Ensure initial state is safe
        self.publish_blue_ready(False)

        # Main control loop (20 Hz)
        self.timer = self.create_timer(UPDATE_PERIOD, self.update)

        self.get_logger().info('Automatic Conveyor Controller started.')
        self.get_logger().info(f'Flow: LEFT -> RIGHT (Y {SOURCE_Y:.2f} -> {EXIT_Y:.2f})')
        self.get_logger().info(f'BLUE PICK Y = {PICK_Y:.2f}')
        self.get_logger().info('Sequence: RED -> GREEN -> BLUE')

    def publish_blue_ready(self, ready):
        """Broadcasts the readiness state of the target product to the robot arm."""
        msg = Bool()
        msg.data = bool(ready)
        self.pick_ready_pub.publish(msg)

    def model_name_for_color(self, color):
        """Maps logical colors to Gazebo model names."""
        if color == 'RED':
            return 'red_object'
        if color == 'GREEN':
            return 'green_object'
        if color == 'BLUE':
            return 'blue_object'
        return 'product_object'

    def spawn_next_product(self):
        """
        Asynchronously calls Gazebo to spawn a new product.
        Asynchronous calls prevent the ROS node from freezing while Gazebo processes the request.
        """
        if self.spawn_in_progress or self.sequence_index >= len(PRODUCT_SEQUENCE):
            return

        color = PRODUCT_SEQUENCE[self.sequence_index]
        name = self.model_name_for_color(color)

        if not self.spawn_client.service_is_ready():
            return

        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf = make_static_cube_sdf(name, color)
        request.entity_factory.relative_to = 'world'

        # Set initial spawn pose
        request.entity_factory.pose.position.x = PRODUCT_X
        request.entity_factory.pose.position.y = SOURCE_Y
        request.entity_factory.pose.position.z = PRODUCT_Z
        request.entity_factory.pose.orientation.w = 1.0

        self.spawn_in_progress = True

        self.get_logger().info('=====================================')
        self.get_logger().info(f'SPAWNING {color} at LEFT entrance.')

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda future, color=color, name=name: self.spawn_finished(future, color, name)
        )

    def spawn_finished(self, future, color, name):
        """Callback triggered when Gazebo successfully creates the product."""
        self.spawn_in_progress = False

        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f'Spawn service failed: {ex}')
            return

        if not result.success:
            self.get_logger().error(f'Could not spawn {color}.')
            return

        # Update state machine to start moving the newly spawned object
        self.current_color = color
        self.current_name = name
        self.current_y = SOURCE_Y

        self.moving = True
        self.delete_in_progress = False

        self.publish_blue_ready(False)

        self.get_logger().info(f'{color} entered conveyor at Y={SOURCE_Y:.2f}.')
        self.get_logger().info('CONVEYOR MOVING ->')
        self.get_logger().info('=====================================')

    def set_current_pose(self, y):
        """
        Calculates and sends the next incremental position to Gazebo.
        Implements rate-limiting by checking if the previous async pose request is finished,
        preventing request floods to the physics engine.
        """
        if self.current_name is None:
            return False

        if not self.pose_client.service_is_ready():
            return False

        if self.pending_pose_request is not None and not self.pending_pose_request.done():
            return False

        request = SetEntityPose.Request()
        request.entity.name = self.current_name
        request.entity.type = Entity.MODEL

        request.pose.position.x = PRODUCT_X
        request.pose.position.y = y
        request.pose.position.z = PRODUCT_Z
        request.pose.orientation.w = 1.0

        self.pending_pose_request = self.pose_client.call_async(request)
        return True

    def delete_current_product(self):
        """Removes valid products (RED, GREEN) once they reach the end of the line."""
        if self.delete_in_progress or self.current_name is None:
            return

        if not self.delete_client.service_is_ready():
            return

        self.delete_in_progress = True
        self.moving = False

        color = self.current_color
        name = self.current_name

        self.get_logger().info(f'{color} reached RIGHT exit. PASS -> removing product.')

        request = DeleteEntity.Request()
        request.entity.name = name
        request.entity.type = Entity.MODEL

        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda future, color=color: self.delete_finished(future, color)
        )

    def delete_finished(self, future, color):
        """Cleans up the state machine after despawning, triggering the next sequence cycle."""
        self.delete_in_progress = False

        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f'Delete service failed: {ex}')
            return

        if not result.success:
            self.get_logger().error(f'Could not remove {color}.')
            return

        self.get_logger().info(f'{color} removed at RIGHT exit.')

        # Advance to the next product in the sequence
        self.sequence_index = (self.sequence_index + 1) % len(PRODUCT_SEQUENCE)

        self.current_color = None
        self.current_name = None
        self.current_y = SOURCE_Y

        # Schedule the next spawn
        self.next_spawn_time = self.get_clock().now() + Duration(seconds=NEXT_PRODUCT_DELAY)

    def stop_blue_at_pick_point(self):
        """Halts the conveyor exactly at the target coordinate and signals the robot arm."""
        self.current_y = PICK_Y
        self.set_current_pose(PICK_Y)

        self.moving = False
        self.waiting_for_blue_robot = True

        self.publish_blue_ready(True)

        self.get_logger().warn('=====================================')
        self.get_logger().warn('BLUE reached calibrated PICK POINT. CONVEYOR STOPPED.')
        self.get_logger().warn(f'Exact position: ({PRODUCT_X:.2f}, {PICK_Y:.2f}, {PRODUCT_Z:.2f})')
        self.get_logger().warn('Waiting for UR5 rejection.')
        self.get_logger().warn('=====================================')

    def blue_rejection_done_callback(self, request, response):
        """
        Service Callback: Triggered by the UR5 robot once it completes the pick-and-place task.
        Resumes the factory pipeline logic.
        """
        if not self.waiting_for_blue_robot:
            response.success = False
            response.message = 'No BLUE product is waiting for robot completion.'
            return response

        # Note: The original STATIC blue_object has already been physically attached
        # to the robot and moved, so we don't need to delete it here.
        self.waiting_for_blue_robot = False
        self.moving = False
        self.publish_blue_ready(False)

        # Advance sequence
        self.sequence_index = (self.sequence_index + 1) % len(PRODUCT_SEQUENCE)
        self.current_color = None
        self.current_name = None
        self.current_y = SOURCE_Y

        self.next_spawn_time = self.get_clock().now() + Duration(seconds=NEXT_PRODUCT_DELAY)

        self.get_logger().info('=====================================')
        self.get_logger().info('BLUE rejection cycle completed. Conveyor line released.')
        self.get_logger().info('=====================================')

        response.success = True
        response.message = 'BLUE rejection acknowledged. Next product scheduled.'
        return response

    def update(self):
        """
        The main control loop executed at 20 Hz.
        Manages spawning timing, kinematic translation, and trigger points.
        """
        if self.current_name is None:
            # Check if it's time to spawn a new product
            if (not self.waiting_for_blue_robot and 
                not self.spawn_in_progress and 
                self.sequence_index < len(PRODUCT_SEQUENCE) and 
                self.get_clock().now() >= self.next_spawn_time):
                
                self.spawn_next_product()
            return

        if self.waiting_for_blue_robot:
            # Re-publish continuously while BLUE is waiting.
            # This ensures robustness if the robot controller node boots up late.
            self.publish_blue_ready(True)
            return

        if not self.moving:
            return

        # Do not overload Gazebo with pose updates if it's lagging
        if self.pending_pose_request is not None and not self.pending_pose_request.done():
            return

        # Calculate kinematic step
        distance = CONVEYOR_SPEED * UPDATE_PERIOD
        next_y = self.current_y + distance

        # Check interception logic
        if self.current_color == 'BLUE' and next_y >= PICK_Y:
            self.stop_blue_at_pick_point()
            return

        # Check despawn logic
        if self.current_color in ('RED', 'GREEN') and next_y >= EXIT_Y:
            self.current_y = EXIT_Y
            self.delete_current_product()
            return

        # Apply movement
        self.current_y = next_y
        self.set_current_pose(self.current_y)


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()

    node.get_logger().info('Waiting for Gazebo services...')

    # Ensure Gazebo physics services are fully loaded before starting the logic
    while not node.pose_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for set_pose...')
    while not node.spawn_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for create...')
    while not node.delete_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for remove...')

    node.get_logger().info('Gazebo services connected. Ready to start pipeline.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
