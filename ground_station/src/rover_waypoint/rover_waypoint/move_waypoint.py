import rclpy
import numpy as np
# import tf_transformations
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node



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
            'cmd_vel'
        )
 
    def vicon_callback(self, msg):   
        self.x_pos = msg.x_trans # TODO: some line to assign msg values to variable here
        self.y_pos = msg.y_trans
        self.z_pos = msg.z_trans
        self.yaw, pitch, roll = R.from_quat([msg.x_rot, msg.y_rot, msg.z_rot, msg.w]).as_euler(seq='zyx')
        # self.yaw, roll, pitch = tf_transformations.euler_from_quaternion([msg.w, msg.x_rot, msg.y_rot, msg.z_rot])[2]
        print(f"Roll: {roll*180.0/np.pi}, Pitch: {pitch*180.0/np.pi}, Yaw: {self.yaw*180.0/np.pi}") 

    def des_wp_callback(self,msg):
        self.waypoints.append(msg)

    def timer_callback(self):
        self.run_state_machine()

    # def rtr_controller(self,args):
    #     #initial values
    #     del_x0 = xg-x
    #     del_y0 = yg-y
    #     alpha0 = np.atan2(del_y0,del_)

    #     theta = q #theta is current rover angle in global
    #     #Rotate until in line with waypoint
    #     control(self,x,y,theta,True) #run with argument of just angle of vector beween start and waypoint
    #     #TODO: how to iterate it to decide to go between modes

    #     #move until distance to waypoint is zero
    #     control(self,x,y,theta,False) #run with just distance as argument and angle as zero

    #     #align with waypoint angle
    #     control(self,x,y,theta,True) # run with zero distance and just angle waypoint originally had


    # def control(self,del_x,del_y,theta,rotate): # TODO: include vicon assigned variable, and desired waypoint
    #     Kv = 1
    #     Kw = 1 
    #     d = np.sqrt(del_x**2 + del_y**2)
    #     alpha = np.atan2(del_y,del_x) - theta
    #     w_des = Kw*alpha 
    #     v_des = 0 if rotate else Kv*d
    #     u_des = [v_des, w_des]
    #     self.desVelPub.publish(u_des) #TODO: rewrite into proper format
    #     return u_des

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
        raise NotImplementedError
        return

    def run_rotate_1(self, k_omega = 1.0, threshold=5*np.pi/180):
        # uses the current state and the next target waypoint to determine where to go next

        if self.target_waypoint is None:
            self.set_next_state("idle")
            return

        del_x = self.target_waypoint.x - self.x_pos
        del_y = self.target_waypoint.y - self.y_pos


        target_yaw = np.arctan2(del_y, del_x)
        
        yaw_error = self.yaw - target_yaw

        if np.abs(yaw_error) > threshold:
            omega = - k_omega * yaw_error
            self.send_command(0, omega)
            # self.set_next_state("rotate_1")
            return
        else:
            omega = 0.0
            send_command(0.0, omega)
            self.set_next_state("straight")
            return "straight"

    



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

