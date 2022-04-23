from re import T, sub
from tkinter import Y
import rclpy
import numpy as np
import scipy #import the transform library specifically?
from rclpy.node import Node


from vicon_receiver.msg import Position

class WaypointController(Node):
    def __init__(self):
        position = [] #TODO: Initialize properly
        super().__init__('waypoint_controller')
        self.viconSub = self.create_subscription(
            Position,
            'vicon/rover7/rover7',
            self.vicon_callback,
            10
        )
        self.viconSub
        
        self.desWaypointSub = self.create_subscription(
            Pose,
            'des_wp',
            self.des_wp_callback,
            10
        )

        # self.desTrajSub = self.create(
            
        # )

        self.desVelPub = self.create_publisher(
            Twist,
            'des_vel'
        )
 
    def vicon_callback(self, msg):   
        position = msg # TODO: some line to assign msg values to variable here
        print(msg) 

    def des_wp_callback(self,msg):
        print(msg)
        rtr_controller(msg)

    def rtr_controller(self,args):
        #initial values
        del_x0 = xg-x
        del_y0 = yg-y
        alpha0 = np.atan2(del_y0,del_x0)

        theta = q #theta is current rover angle in global
        #Rotate until in line with waypoint
        control(self,x,y,theta,True) #run with argument of just angle of vector beween start and waypoint
        #TODO: how to iterate it to decide to go between modes

        #move until distance to waypoint is zero
        control(self,x,y,theta,False) #run with just distance as argument and angle as zero

        #align with waypoint angle
        control(self,x,y,theta,True) # run with zero distance and just angle waypoint originally had


    def control(self,del_x,del_y,theta,rotate): # TODO: include vicon assigned variable, and desired waypoint
        Kv = 1
        Kw = 1 
        d = np.sqrt(del_x**2 + del_y**2)
        alpha = np.atan2(del_y,del_x) - theta
        w_des = Kw*alpha 
        v_des = 0 if rotate else Kv*d
        u_des = [v_des, w_des]
        self.desVelPub.publish(u_des) #TODO: rewrite into proper format
        return u_des


def main(args=None):
    rclpy.init(args=args)

    listener = WaypointController()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

