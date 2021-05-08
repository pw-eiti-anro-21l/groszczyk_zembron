import sys
import rclpy
from rclpy.node import Node
from zad4_srv.srv import OintInterpolation


class oint_client(Node):

    def __init__(self):
        super().__init__('oint')

        self.cli = self.create_client(OintInterpolation, 'oint_control_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OintInterpolation.Request()

    def send_request(self):
        try:
            self.req.x_pose = float(sys.argv[1])
            self.req.y_pose = float(sys.argv[2])
            self.req.z_pose = float(sys.argv[3])
            self.req.roll_pose = float(sys.argv[4])
            self.req.pitch_pose = float(sys.argv[5])
            self.req.yaw_pose = float(sys.argv[6])
            self.req.move_time = float(sys.argv[7])
            self.req.interpolation_method = sys.argv[8]
            self.future = self.cli.call_async(self.req)
        except ValueError:
            self.get_logger().info('ValueError while passing parameters ')
            self.req.x_pose  = -1
            self.req.y_pose = -1
            self.req.z_pose = -1
            self.req.roll_pose = -1
            self.req.pitch_pose = -1
            self.req.yaw_pose = -1
            self.req.move_time = -1


            self.req.interpolation_method = sys.argv[8]
            self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    print(sys.argv)

    client = oint_client()
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
