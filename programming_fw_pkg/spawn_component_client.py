import rclpy
from spawn_object_interfaces.srv import SpawnObject

class SpawnComponentClient:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('spawn_component_client')
        self.client = self.node.create_client(SpawnObject, '/object_manager/spawn_object')

    def spawn_component(self, obj_name, parent_frame, translation, rotation, cad_data):
        request = SpawnObject.Request()
        request.obj_name = obj_name
        request.parent_frame = parent_frame
        request.translation = translation
        request.rotation = rotation
        request.cad_data = cad_data

        if self.client.wait_for_service(timeout_sec=2.0):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None and future.result().success:
                self.node.get_logger().info('Component spawned successfully')
            else:
                self.node.get_logger().error('Failed to spawn Component')
        else:
            self.node.get_logger().warn("Service not available. Unable to spawn Component.")

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()

# object_spawner_manager/launch/object_spawner.launch.py
# ros2 launch object_spawner_manager object_spawner.launch.py

# ros2 service call /object_manager/spawn_object spawn_object_interfaces/srv/SpawnObject "{obj_name: LEON_TEST_OBJECT, parent_frame: Gonio_Right_Part_Origin , translation:[0.0,0.0,0.0], rotation:[0.0,0.0,0.0,1.0], cad_data: /home/leonkoch/Distrobox/Ubuntu22/ros2_ws/src/pm_robot_programming_framework/programming_fw_pkg/programming_fw_pkg/S_UFC.STL}"
