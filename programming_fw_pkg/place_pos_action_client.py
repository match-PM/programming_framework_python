import rclpy
from spawn_object_interfaces.srv import CreateRefFrame
import geometry_msgs.msg

class PlacePosActionClient:
    def __init__(self):
        rclpy.init() 
        self.node = rclpy.create_node('place_pos_action_client')
        self.client = self.node.create_client(CreateRefFrame, '/object_manager/create_ref_frame')

    def send_create_ref_frame_request(self, target_component):
        request = self.create_request_for_create_ref_frame(target_component)

        if self.client.wait_for_service(timeout_sec=2.0):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None and future.result().success:
                self.node.get_logger().info('Reference frame created successfully')
            else:
                self.node.get_logger().error('Failed to create reference frame')
        else:
            self.node.get_logger().warn("Service not available. Unable to create reference frame.")

    def create_request_for_create_ref_frame(self, target_component):
        request = CreateRefFrame.Request()
        request.frame_name = "Drop_Position_Gonio_Left"
        request.parent_frame = target_component
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.098 # was 0.097
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        request.pose = pose
        return request

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()
