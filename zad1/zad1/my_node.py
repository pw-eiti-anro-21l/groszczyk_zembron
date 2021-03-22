import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from curtsies import Input
from math import pi


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')


        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.01  # seconds

        # time.sleep(0.1)
        self.timer = self.create_timer(timer_period, self.turtle_control)

        self.declare_parameter('forward', 'w')
        self.declare_parameter('back', 'x')
        self.declare_parameter('left', 'a')
        self.declare_parameter('right', 'd')

        self.get_logger().info("Welcome! \n Default settings: |  FORWARD: W  |  BACK: X  | LEFT:  A|  RIGHT:  D  |")

        self.linear= 0.0
        self.angular= 0.0

    def set_velocity(self, lin, ang):
    	self.linear= float(lin)
    	self.angular= float(ang)


    def turtle_control(self):
    	forward = self.get_parameter('forward').get_parameter_value().string_value
    	back= self.get_parameter('back').get_parameter_value().string_value
    	left= self.get_parameter('left').get_parameter_value().string_value
    	right= self.get_parameter('right').get_parameter_value().string_value

    	lin_const=3
    	ang_const=pi/2

    	with Input(keynames='curtsies') as input_generator:

    		e=input_generator.send(0.1)
    		e=str(e)
    		self.set_velocity(0, 0)
    		if(e==forward):
    			self.set_velocity(lin_const, 0)

    		elif(e==back):
    			self.set_velocity(-lin_const, 0)

    		elif(e==left):
    			self.set_velocity(0, ang_const)

    		elif(e==right):
    			self.set_velocity(0, -ang_const)

    		msg= Twist()
    		msg.linear.x= self.linear
    		msg.angular.z= self.angular
    		self.publisher_.publish(msg)


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
