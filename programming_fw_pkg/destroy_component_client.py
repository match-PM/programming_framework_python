import rclpy
from spawn_object_interfaces.srv import DestroyObject
from rclpy.node import Node

### SERVICE IS NOT WORKING!
class DestroyComponentClient(Node):
    def __init__(self):
        rclpy.init() 
        self.node = rclpy.create_node('destroy_component_client')
        self.client = self.node.create_client(DestroyObject, '/object_manager/destroy_object')

    def destroy_component(self, obj_name):
        request = DestroyObject.Request()
        request.obj_name = obj_name

        if self.client.wait_for_service(timeout_sec=2.0):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None and future.result().success:
                self.node.get_logger().info('Component destroyed successfully')
            else:
                self.node.get_logger().error('Failed to destroy Component')
        else:
            self.node.get_logger().warn("Service not available. Unable to destroy Component.")

def main(args=None):
    rclpy.init(args=args)

    destroy_object_client = DestroyComponentClient()

    destroy_object_client.destroy_node()

    try:
        rclpy.spin(destroy_object_client)
    finally:
        destroy_object_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# object_spawner_manager/launch/object_spawner.launch.py
# ros2 service call /object_manager/destroy_object spawn_object_interfaces/srv/DestroyObject "{obj_name: Test_Component}"