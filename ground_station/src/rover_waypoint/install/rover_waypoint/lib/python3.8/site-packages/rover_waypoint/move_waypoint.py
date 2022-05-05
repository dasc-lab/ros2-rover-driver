import rclpy
import numpy as np
# import tf_transformations
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist


from vicon_receiver.msg import Position

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.yaw = 0

        self.state = "idle"
        self.target_waypoint = None
        self.waypoints = []

        self.viconSub = self.create_subscription(
            Position,
            'vicon/rover7/rover7',
            self.vicon_callback,
            10
        )

        self.desWaypointSub = self.create_subscription(
            Pose,
            'des_wp',
            self.des_wp_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.desVelPub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
 
    def vicon_callback(self, msg):   
        self.x_pos = msg.x_trans / 1000 # TODO: confirm vicon gives in mm
        self.y_pos = msg.y_trans / 1000
        self.z_pos = msg.z_trans / 1000
        self.yaw, pitch, roll = R.from_quat([msg.x_rot, msg.y_rot, msg.z_rot, msg.w]).as_euler(seq='zyx')
        # self.yaw, roll, pitch = tf_transformations.euler_from_quaternion([msg.w, msg.x_rot, msg.y_rot, msg.z_rot])[2]
        #print(f"X: {self.x_pos}, Y: {self.y_pos}, Z: {self.z_pos}") 
        #print(f"Roll: {roll*180.0/np.pi}, Pitch: {pitch*180.0/np.pi}, Yaw: {self.yaw*180.0/np.pi}") 

    def des_wp_callback(self,msg):
        self.waypoints.append(msg)

    def timer_callback(self):
        self.run_state_machine()

    def run_state_machine(self):
        if self.state == "idle":
            self.send_command(0.,0.)
            if len(self.waypoints) > 0:
                self.target_waypoint = self.waypoints.pop(0)
                self.set_next_state("rotate_1")
            return
        
        if self.state == "rotate_1":
            self.run_rotate_1()
            return

        if self.state == "straight":
            self.run_straight()
            return

        if self.state == "rotate_2":
            self.run_rotate_2()
            return

        # should never get here
        print("SHOULD NOT BE HERE")
        self.set_next_state("idle")
        self.target_waypoint = None
        return

    def set_next_state(self, new_state):
        print(f"SETTING NEW STATE TO {new_state}")
        self.state = new_state

    def send_command(self, lin_vel, ang_vel):
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        #print(f"lin: {msg.linear.x}, ang: {msg.angular.z}") 
        self.desVelPub.publish(msg)

    def run_rotate_1(self, k_omega = 0.1, threshold=5*np.pi/180):
        # uses the current state and the next target waypoint to determine where to go next

        if self.target_waypoint is None:
            self.set_next_state("idle")
            return

        del_x = self.target_waypoint.position.x - self.x_pos
        del_y = self.target_waypoint.position.y - self.y_pos


        target_yaw = np.arctan2(del_y, del_x)
        
        yaw_error = self.yaw - target_yaw

        if np.abs(yaw_error) > threshold:
            omega = - k_omega * yaw_error
            self.send_command(0.0, omega)
            # self.set_next_state("rotate_1")
            return
        else:
            omega = 0.0
            self.send_command(0.0, omega)
            self.set_next_state("straight")
            return

    def run_straight(self, k_omega = 0.1, k_d = 0.1, threshold_ang=5*np.pi/180, threshold_lin=0.1):
        if  self.target_waypoint is None:
            self.set_next_state("idle")
            return

        del_x = self.target_waypoint.position.x - self.x_pos
        del_y = self.target_waypoint.position.y - self.y_pos


        target_yaw = np.arctan2(del_y, del_x)

        dist_error = np.sqrt(del_x**2 + del_y**2)
        
        yaw_error = self.yaw - target_yaw

        if np.abs(yaw_error) > threshold_ang or np.abs(distance_error) > threshold_lin:
            omega = - k_omega * yaw_error
            d = k_d * dist_error
            self.send_command(d, omega)
            # self.set_next_state("rotate_1")
            return
        else:
            omega = 0.0
            d = 0.0
            send_command(d, omega)
            self.set_next_state("rotate_2")
            return 


        


    def run_rotate_2(self, k_omega = 0.1, threshold=5*np.pi/180):
        if self.target_waypoint is None:
                    self.set_next_state("idle")
                    return

        del_x = self.target_waypoint.position.x - self.x_pos
        del_y = self.target_waypoint.position.y - self.y_pos


        target_yaw = np.arctan2(del_y, del_x)
        
        yaw_error = self.yaw - target_yaw

        if np.abs(yaw_error) > threshold:
            omega = - k_omega * yaw_error
            self.send_command(0.0, omega)
            # self.set_next_state("rotate_1")
            return
        else:
            omega = 0.0
            send_command(0.0, omega)
            self.set_next_state("idle")
            return 

def main(args=None):
    rclpy.init(args=args)

    node = WaypointController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

