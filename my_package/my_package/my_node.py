<<<<<<< HEAD

=======
>>>>>>> 9a303fc19de5fb596ced76e4e8bc6a84fc3d2512
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
import turtlesim



class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 3.14
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)



def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
