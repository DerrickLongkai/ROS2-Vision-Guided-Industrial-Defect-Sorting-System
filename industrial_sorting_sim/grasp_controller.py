import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

# tf2_ros is the standard ROS 2 library for tracking coordinate frames over time
from tf2_ros import Buffer, TransformListener, TransformException
from std_srvs.srv import SetBool

# Gazebo-ROS interfaces
from ros_gz_interfaces.srv import SetEntityPose, DeleteEntity, SpawnEntity
from ros_gz_interfaces.msg import Entity


class GraspController(Node):
    """
    ROS 2 Node managing the pick-and-place logic for the UR5 robot.
    
    Instead of relying on Gazebo's physics engine to calculate complex friction 
    and contact forces for the vacuum gripper (which is notoriously unstable), 
    this node uses a "Kinematic Teleportation" strategy:
    1. Attach: It continuously updates the static object's pose to match the gripper's TF.
    2. Release: It deletes the static object and spawns a dynamic version of the same 
       object at the exact release coordinates, allowing gravity to naturally drop it into the bin.
    """

    def __init__(self):
        super().__init__('grasp_controller')
        self.reject_count = 0

        # ==========================================
        # TF (Transform) Initialization
        # ==========================================
        # The Buffer stores a history of coordinate transforms.
        # The Listener automatically subscribes to /tf and /tf_static to populate the buffer.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==========================================
        # Gazebo Service Clients
        # ==========================================
        self.pose_client = self.create_client(SetEntityPose, '/world/sorting_world/set_pose')
        self.delete_client = self.create_client(DeleteEntity, '/world/sorting_world/remove')
        self.spawn_client = self.create_client(SpawnEntity, '/world/sorting_world/create')

        # ==========================================
        # User Grasp Service Server
        # ==========================================
        # The robot trajectory controller calls this service to trigger attach/release
        self.grasp_service = self.create_service(
            SetBool, 
            '/grasp_blue', 
            self.grasp_callback
        )

        # ==========================================
        # State Machine Variables
        # ==========================================
        self.attached = False
        self.pending_pose_request = None
        self.release_in_progress = False

        # Path to the dynamic version of the blue cube.
        # Unlike the static conveyor cubes, this one has <gravity>true</gravity> 
        # and <collision> enabled so it interacts with the reject bin physically.
        self.dynamic_model_path = os.path.expanduser(
            '~/ros2_sorting_ws/src/industrial_sorting_sim/models/blue_dynamic/model.sdf'
        )

        # ==========================================
        # Gripper Tracking Loop (20 Hz)
        # ==========================================
        self.timer = self.create_timer(0.05, self.update_object_pose)

        self.get_logger().info('Grasp Controller started.')
        self.get_logger().info('Strategy: STATIC object used during grasp -> DYNAMIC object spawned on release.')

    def get_grasp_transform(self):
        """
        Looks up the real-time spatial transformation between the global 'world' frame 
        and the robot's 'grasp_point' end-effector frame.
        """
        try:
            return self.tf_buffer.lookup_transform(
                'world',
                'grasp_point',
                Time(),
                timeout=Duration(seconds=0.2)
            )
        except TransformException as ex:
            self.get_logger().error(f'Could not get grasp_point TF: {ex}')
            return None

    def set_blue_pose_from_transform(self, transform):
        """
        Converts a TF transform into a Gazebo SetEntityPose request to artificially 
        glue the blue object to the robot's gripper.
        """
        if not self.pose_client.service_is_ready():
            return False

        request = SetEntityPose.Request()
        request.entity.name = 'blue_object'
        request.entity.type = Entity.MODEL

        # Map translation and rotation from TF to Gazebo Pose
        request.pose.position.x = transform.transform.translation.x
        request.pose.position.y = transform.transform.translation.y
        request.pose.position.z = transform.transform.translation.z
        
        request.pose.orientation.x = transform.transform.rotation.x
        request.pose.orientation.y = transform.transform.rotation.y
        request.pose.orientation.z = transform.transform.rotation.z
        request.pose.orientation.w = transform.transform.rotation.w

        self.pending_pose_request = self.pose_client.call_async(request)
        return True

    def grasp_callback(self, request, response):
        """
        Service Callback for '/grasp_blue'.
        request.data == True  -> Attach (start tracking)
        request.data == False -> Release (delete static, spawn dynamic)
        """
        # ------------------------------------------
        # ATTACH LOGIC
        # ------------------------------------------
        if request.data:
            if self.release_in_progress:
                response.success = False
                response.message = 'Release currently in progress.'
                return response

            transform = self.get_grasp_transform()
            if transform is None:
                response.success = False
                response.message = 'Could not get grasp_point transform.'
                return response

            # Enable the 20Hz update loop to start "teleporting" the object to the gripper
            self.attached = True
            self.get_logger().info('BLUE static object ATTACHED.')
            
            response.success = True
            response.message = 'BLUE object ATTACHED to grasp point.'
            return response

        # ------------------------------------------
        # RELEASE LOGIC
        # ------------------------------------------
        transform = self.get_grasp_transform()
        if transform is None:
            response.success = False
            response.message = 'Could not get release pose.'
            return response

        # 1. Stop following the gripper immediately
        self.attached = False

        if not self.delete_client.service_is_ready() or not self.spawn_client.service_is_ready():
            response.success = False
            response.message = 'Gazebo services unavailable for release.'
            return response

        self.release_in_progress = True
        self.get_logger().info('RELEASE: deleting STATIC blue object...')

        # 2. Delete the statically controlled cube
        delete_request = DeleteEntity.Request()
        delete_request.entity.name = 'blue_object'
        delete_request.entity.type = Entity.MODEL

        delete_future = self.delete_client.call_async(delete_request)
        
        # 3. Chain the spawn operation to execute only after deletion is confirmed
        delete_future.add_done_callback(
            lambda future: self.delete_finished_callback(future, transform)
        )

        response.success = True
        response.message = 'BLUE object RELEASE initiated.'
        return response

    def delete_finished_callback(self, future, transform):
        """Triggered when Gazebo successfully deletes the static blue cube."""
        try:
            result = future.result()
        except Exception as ex:
            self.release_in_progress = False
            self.get_logger().error(f'Delete service failed: {ex}')
            return

        if not result.success:
            self.release_in_progress = False
            self.get_logger().error('Could not delete STATIC blue object.')
            return

        self.get_logger().info('STATIC blue object deleted.')
        
        # Proceed to spawn the dynamic version
        self.spawn_dynamic_blue(transform)

    def spawn_dynamic_blue(self, transform):
        """
        Spawns a physics-enabled (dynamic) blue cube exactly where the static one 
        was just deleted, so it naturally falls into the reject bin.
        """
        if not os.path.exists(self.dynamic_model_path):
            self.release_in_progress = False
            self.get_logger().error(f'Dynamic model SDF not found: {self.dynamic_model_path}')
            return

        spawn_request = SpawnEntity.Request()

        # Give each rejected BLUE cube a unique name (e.g., rejected_blue_001).
        # This prevents Gazebo naming conflicts and leaves 'blue_object' free 
        # for the next defective product on the conveyor.
        self.reject_count += 1
        self.current_rejected_name = f'rejected_blue_{self.reject_count:03d}'

        spawn_request.entity_factory.name = self.current_rejected_name
        spawn_request.entity_factory.allow_renaming = False
        spawn_request.entity_factory.sdf_filename = self.dynamic_model_path
        spawn_request.entity_factory.relative_to = 'world'

        # Spawn exactly at the release coordinates captured from TF
        spawn_request.entity_factory.pose.position.x = transform.transform.translation.x
        spawn_request.entity_factory.pose.position.y = transform.transform.translation.y
        spawn_request.entity_factory.pose.position.z = transform.transform.translation.z
        
        spawn_request.entity_factory.pose.orientation.x = transform.transform.rotation.x
        spawn_request.entity_factory.pose.orientation.y = transform.transform.rotation.y
        spawn_request.entity_factory.pose.orientation.z = transform.transform.rotation.z
        spawn_request.entity_factory.pose.orientation.w = transform.transform.rotation.w

        self.get_logger().info(f'Spawning {self.current_rejected_name} as DYNAMIC reject...')
        spawn_future = self.spawn_client.call_async(spawn_request)
        spawn_future.add_done_callback(self.spawn_finished_callback)

    def spawn_finished_callback(self, future):
        """Finalizes the release state machine after the dynamic cube is spawned."""
        try:
            result = future.result()
        except Exception as ex:
            self.release_in_progress = False
            self.get_logger().error(f'Spawn service failed: {ex}')
            return

        self.release_in_progress = False

        if result.success:
            self.get_logger().info(f'{self.current_rejected_name} spawned.')
            self.get_logger().info('Gravity now controls the rejected cube.')
        else:
            self.get_logger().error('Could not spawn dynamic blue object.')

    def update_object_pose(self):
        """
        Timer callback (20 Hz). Continuously updates the static object's pose
        to match the robot's end effector while `self.attached` is True.
        """
        if not self.attached:
            return

        # Rate-limiting: Wait for Gazebo to finish the previous pose update
        if self.pending_pose_request is not None and not self.pending_pose_request.done():
            return

        if not self.pose_client.service_is_ready():
            return

        transform = self.get_grasp_transform()
        if transform is None:
            return

        self.set_blue_pose_from_transform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GraspController()

    node.get_logger().info('Waiting for Gazebo services...')

    while not node.pose_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for set_pose...')
    while not node.delete_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for remove...')
    while not node.spawn_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for create...')

    node.get_logger().info('All Gazebo services connected.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
