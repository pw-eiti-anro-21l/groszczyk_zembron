from rclpy.node import Node
import rclpy

class jint_node(Node):
    def __init__(self):
        super().__init__('jint_node')
        self.positon = [0,0,0]
        self.subscription = self.create_subscription(JointState, 'joint_states',self.start_position_callback, 10)
        self.srv = self.create_service(JintControlSrv,'jint_control_srv', self.jint_control_callback)

    def start_position_callback(self, msg):
        #pobieranie aktualnych położeń stawów
        self.positon[0] = msg.position[0]
        self.positon[1] = msg.position[1]
        self.positon[2] = msg.position[2]

    def jint_control_callback(self, request, response ):




def main(args=None):
    rclpy.init(args=args)

    jint_node = jint_node()

    rclpy.spin(jint_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
