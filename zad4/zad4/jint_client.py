import sys
import rclpy
from rclpy.node import Node
from zad4_srv.srv import RobotInterpolation


class jint_client(Node):

    def __init__(self):
        super().__init__('jint')
        self.cli = self.create_client(RobotInterpolation, 'jint_control_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RobotInterpolation.Request()

    def send_request(self):
        try:
            self.req.joint1_pose = float(sys.argv[1])
            self.req.joint2_pose = float(sys.argv[2])
            self.req.joint3_pose = float(sys.argv[3])
            self.req.move_time = float(sys.argv[4])
            self.req.interpolation_method = sys.argv[5]
            self.future = self.cli.call_async(self.req)
        except ValueError:
            self.get_logger().info('ValueError while passing parameters ')
            self.req.joint1_pose = -1
            self.req.joint2_pose = -1
            self.req.joint3_pose = -1
            self.req.move_time = f-1
            self.req.interpolation_method = sys.argv[5]
            self.future = self.client.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)
    print(sys.argv)

    client = jint_client()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                client.get_logger().info(response.output)
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
