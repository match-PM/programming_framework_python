import rclpy
from rclpy.node import Node

from pm_moveit_interfaces.srv import MoveToolTcpTo

# not needed here:
# from pm_moveit_interfaces.srv import MoveCam1TcpTo
# from pm_moveit_interfaces.srv import MoveLaserTcpTo

class CallServiceNode(Node):

    def __init__(self):
        super().__init__('call_service_node')
        self.call_service()
       
    # Call service means sending request to server
    def call_service(self):

        # 1. Create Client
        self.client = self.create_client(MoveToolTcpTo, '/pm_moveit_server/move_tool_to_frame')
        # self.client = self.create_client(<Service_Type>, '<Service_Name>')
        # pm_moveit_interfaces/srv/MoveToolTcpTo -- Servie Type

        # 2. Wait for server to response
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again ...')

        # 3. Create a Request
        request = MoveToolTcpTo.Request()
        request.frame_name = 'Camera_Station_TCP'
        request.move_to_pose.position.x = 0.530
        request.move_to_pose.position.y = 0.40
        request.move_to_pose.position.z = 1.3
        request.move_to_pose.orientation.w = 1.0
        request.translation.x = 0.0
        request.translation.y = 0.0
        request.translation.z = 0.015
        request.exec_wait_for_user_input = False
        request.execute = True

        # 4. call the server async, send request - we get a future
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_call_service)
        #rclpy.spin_until_future_complete(self, future)

    #process the result in the callback    
    def callback_call_service(self, future):
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {response.success}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = CallServiceNode()
    #node.call_service()
    rclpy.spin
    rclpy.shutdown()

if __name__ == '__main__':
    main()
