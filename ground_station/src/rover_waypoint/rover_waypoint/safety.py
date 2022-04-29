from argparse import ArgumentDefaultsHelpFormatter
from cv2 import remap
from matplotlib.pyplot import cool
import rclpy
import numpy
from rclpy.node import Node

FORWARD_AXIS_INDEX = 1
TURN_AXIS_INDEX = 3
ARM_BUTTON_INDEX = 2
CMD_MODE_BUTTON_INDEX = 0
MAX_ARM_TIMEOUT = 0.5

armed = False
autonomous = False
LIN_V_MAX = -2.0
ANG_V_MAX = -4.0


class JoystickSafety(Node):
    
    def __init__(self): 
        super().__init__('joystick_safety')
        self.desVelSub = self.create_subscription(
            Twist,
            'des_vel',
            des_vel_callback,
            10
        )

        self.joySub = self.create_subscription(
            Joy,
            'joy',
            joy_sub_callback,
            10
        )
        
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step = 0.01, jump_threshold = 0.0)

        self.cmdVelPub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.desVelSub
        self.joySub
        self.cmdVelPub
    
    def des_vel_callback(self,msg):
        if not armed:
            return
        
        if not autonomous:
            return
        
        if not armIsRecent:
            armed = False
            return
        self.cmdVelPub.publish(msg)


    def joy_sub_callback(self,msg):
        armed =  True if msg.buttons[ARM_BUTTON_INDEX] == 1 else False
        autonomous  = True if msg.buttons[CMD_MODE_BUTTON_INDEX] == 0 else False
        lastArmMsg = self.get_clock().now()
        if not armed: return
        if not autonomous:
            lin_vel = remap(msg.axes[FORWARD_AXIS_INDEX], LIN_V_MAX)
            ang_vel = remap(msg.axes[TURN_AXIS_INDEX], ANG_V_MAX)

            self.cmdVelPub.publsih(lin_vel, ang_vel)
            
            return
    
    def remap(val, max):
        return val * max
            

    LIN_V_MAX = -2.0

    rclpy.init(args=args)

    safety = JoystickSafety()

    rclpy.spin(safety)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    

    
    
