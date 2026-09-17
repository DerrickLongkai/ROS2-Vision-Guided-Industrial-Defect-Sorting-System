import os

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)

from std_srvs.srv import SetBool

from ros_gz_interfaces.srv import (
    SetEntityPose,
    DeleteEntity,
    SpawnEntity,
)

from ros_gz_interfaces.msg import Entity


class GraspController(Node):

    def __init__(self):
    
        self.reject_count = 0

        super().__init__('grasp_controller')

        # ==========================================
        # TF
        # ==========================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ==========================================
        # Gazebo services
        # ==========================================

        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/sorting_world/set_pose'
        )

        self.delete_client = self.create_client(
            DeleteEntity,
            '/world/sorting_world/remove'
        )

        self.spawn_client = self.create_client(
            SpawnEntity,
            '/world/sorting_world/create'
        )

        # ==========================================
        # User grasp service
        # ==========================================

        self.grasp_service = self.create_service(
            SetBool,
            '/grasp_blue',
            self.grasp_callback
        )

        # ==========================================
        # State
        # ==========================================

        self.attached = False

        self.pending_pose_request = None

        self.release_in_progress = False

        # Dynamic blue cube SDF
        self.dynamic_model_path = os.path.expanduser(
            '~/ros2_sorting_ws/src/'
            'industrial_sorting_sim/'
            'models/blue_dynamic/model.sdf'
        )

        # ==========================================
        # Follow gripper at 20 Hz
        # ==========================================

        self.timer = self.create_timer(
            0.05,
            self.update_object_pose
        )

        self.get_logger().info(
            'Grasp Controller started.'
        )

        self.get_logger().info(
            'STATIC blue object used during grasp.'
        )

        self.get_logger().info(
            'Dynamic blue object spawned on release.'
        )

    # ==============================================
    # Get grasp point transform
    # ==============================================

    def get_grasp_transform(self):

        try:

            return self.tf_buffer.lookup_transform(
                'world',
                'grasp_point',
                Time(),
                timeout=Duration(seconds=0.2)
            )

        except TransformException as ex:

            self.get_logger().error(
                f'Could not get grasp_point TF: {ex}'
            )

            return None

    # ==============================================
    # Set blue cube pose while attached
    # ==============================================

    def set_blue_pose_from_transform(
        self,
        transform
    ):

        if not self.pose_client.service_is_ready():

            return False

        request = SetEntityPose.Request()

        request.entity.name = 'blue_object'
        request.entity.type = Entity.MODEL

        request.pose.position.x = (
            transform.transform.translation.x
        )

        request.pose.position.y = (
            transform.transform.translation.y
        )

        request.pose.position.z = (
            transform.transform.translation.z
        )

        request.pose.orientation.x = (
            transform.transform.rotation.x
        )

        request.pose.orientation.y = (
            transform.transform.rotation.y
        )

        request.pose.orientation.z = (
            transform.transform.rotation.z
        )

        request.pose.orientation.w = (
            transform.transform.rotation.w
        )

        self.pending_pose_request = (
            self.pose_client.call_async(request)
        )

        return True

    # ==============================================
    # ATTACH / RELEASE
    # ==============================================

    def grasp_callback(
        self,
        request,
        response
    ):

        # ------------------------------------------
        # ATTACH
        # ------------------------------------------

        if request.data:

            if self.release_in_progress:

                response.success = False
                response.message = (
                    'Release currently in progress.'
                )

                return response

            transform = self.get_grasp_transform()

            if transform is None:

                response.success = False
                response.message = (
                    'Could not get grasp_point transform.'
                )

                return response

            self.attached = True

            self.get_logger().info(
                'BLUE static object ATTACHED.'
            )

            response.success = True
            response.message = (
                'BLUE object ATTACHED to grasp point.'
            )

            return response

        # ------------------------------------------
        # RELEASE
        # ------------------------------------------

        transform = self.get_grasp_transform()

        if transform is None:

            response.success = False
            response.message = (
                'Could not get release pose.'
            )

            return response

        # Stop follower immediately.
        self.attached = False

        if not self.delete_client.service_is_ready():

            response.success = False
            response.message = (
                'Gazebo delete service unavailable.'
            )

            return response

        if not self.spawn_client.service_is_ready():

            response.success = False
            response.message = (
                'Gazebo spawn service unavailable.'
            )

            return response

        self.release_in_progress = True

        self.get_logger().info(
            'RELEASE: deleting STATIC blue object...'
        )

        delete_request = DeleteEntity.Request()

        delete_request.entity.name = 'blue_object'
        delete_request.entity.type = Entity.MODEL

        delete_future = (
            self.delete_client.call_async(
                delete_request
            )
        )

        # When deletion finishes, spawn dynamic cube
        delete_future.add_done_callback(
            lambda future:
            self.delete_finished_callback(
                future,
                transform
            )
        )

        response.success = True

        response.message = (
            'BLUE object RELEASE initiated.'
        )

        return response

    # ==============================================
    # STATIC object deleted
    # ==============================================

    def delete_finished_callback(
        self,
        future,
        transform
    ):

        try:

            result = future.result()

        except Exception as ex:

            self.release_in_progress = False

            self.get_logger().error(
                f'Delete service failed: {ex}'
            )

            return

        if not result.success:

            self.release_in_progress = False

            self.get_logger().error(
                'Could not delete STATIC blue object.'
            )

            return

        self.get_logger().info(
            'STATIC blue object deleted.'
        )

        self.spawn_dynamic_blue(transform)

    # ==============================================
    # Spawn dynamic cube at exact release pose
    # ==============================================

    def spawn_dynamic_blue(
        self,
        transform
    ):

        if not os.path.exists(
            self.dynamic_model_path
        ):

            self.release_in_progress = False

            self.get_logger().error(
                'Dynamic model SDF not found: '
                f'{self.dynamic_model_path}'
            )

            return

        spawn_request = SpawnEntity.Request()

        # Give each rejected BLUE cube a unique name.
        # This leaves the name 'blue_object' free for the
        # next BLUE product generated on the conveyor.
        self.reject_count += 1

        self.current_rejected_name = (
            f'rejected_blue_{self.reject_count:03d}'
        )

        spawn_request.entity_factory.name = (
            self.current_rejected_name
        )

        spawn_request.entity_factory.allow_renaming = (
            False
        )

        spawn_request.entity_factory.sdf_filename = (
            self.dynamic_model_path
        )

        spawn_request.entity_factory.relative_to = (
            'world'
        )

        # Spawn exactly where the cube was held
        spawn_request.entity_factory.pose.position.x = (
            transform.transform.translation.x
        )

        spawn_request.entity_factory.pose.position.y = (
            transform.transform.translation.y
        )

        spawn_request.entity_factory.pose.position.z = (
            transform.transform.translation.z
        )

        spawn_request.entity_factory.pose.orientation.x = (
            transform.transform.rotation.x
        )

        spawn_request.entity_factory.pose.orientation.y = (
            transform.transform.rotation.y
        )

        spawn_request.entity_factory.pose.orientation.z = (
            transform.transform.rotation.z
        )

        spawn_request.entity_factory.pose.orientation.w = (
            transform.transform.rotation.w
        )

        self.get_logger().info(
            f'Spawning {self.current_rejected_name} '
            'as DYNAMIC reject...'
        )

        spawn_future = (
            self.spawn_client.call_async(
                spawn_request
            )
        )

        spawn_future.add_done_callback(
            self.spawn_finished_callback
        )

    # ==============================================
    # Dynamic spawn finished
    # ==============================================

    def spawn_finished_callback(
        self,
        future
    ):

        try:

            result = future.result()

        except Exception as ex:

            self.release_in_progress = False

            self.get_logger().error(
                f'Spawn service failed: {ex}'
            )

            return

        self.release_in_progress = False

        if result.success:

            self.get_logger().info(
                f'{self.current_rejected_name} spawned.'
            )

            self.get_logger().info(
                'Gravity now controls the rejected cube.'
            )

        else:

            self.get_logger().error(
                'Could not spawn dynamic blue object.'
            )

    # ==============================================
    # Follow gripper while STATIC object attached
    # ==============================================

    def update_object_pose(self):

        if not self.attached:
            return

        if (
            self.pending_pose_request is not None
            and not self.pending_pose_request.done()
        ):
            return

        if not self.pose_client.service_is_ready():

            return

        transform = self.get_grasp_transform()

        if transform is None:
            return

        self.set_blue_pose_from_transform(
            transform
        )


def main(args=None):

    rclpy.init(args=args)

    node = GraspController()

    node.get_logger().info(
        'Waiting for Gazebo services...'
    )

    while not node.pose_client.wait_for_service(
        timeout_sec=1.0
    ):

        node.get_logger().info(
            'Waiting for set_pose...'
        )

    while not node.delete_client.wait_for_service(
        timeout_sec=1.0
    ):

        node.get_logger().info(
            'Waiting for remove...'
        )

    while not node.spawn_client.wait_for_service(
        timeout_sec=1.0
    ):

        node.get_logger().info(
            'Waiting for create...'
        )

    node.get_logger().info(
        'All Gazebo services connected.'
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
