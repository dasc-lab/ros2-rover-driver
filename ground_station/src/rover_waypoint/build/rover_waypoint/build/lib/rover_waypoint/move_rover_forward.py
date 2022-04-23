import rclpy
import numpy
from rclpy.node import Node


from std_msgs.msg import String
from vicon_receiver.msg import Position

class waypoint_controller(Node):

    def __init__(self):
        super().__init__('waypoint_controller')
        self.subscription = self.create_subscription(
            Position,
            'vicon/rover7/rover7',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # self.publisher_ = self.create_publisher("/cmd_vel", Twist, 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
 
    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        print(msg.data)
        

    # def timer_callback(self):
    #     self.i = 0
    #     # msg = String()
    #     # msg.data = 'Hello World: %d' % self.i
    #     # self.publisher_.publish(msg)
    #     # self.get_logger().info('Publishing: "%s"' % msg.data)
    #     # self.i += 1
    
    # def control(self,msg):
    #     #take in msg.data as position points
        
    #     #command it to  move 1m forward
    #     msg.data.x_trans

    #     #run publish of /cmd_vel needed to make it happen

def main(args=None):
    rclpy.init(args=args)

    listener = waypoint_controller()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

