#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os

import pandas as pd
from robot_control import robot
from sensor_msgs.msg import JointState

import time
import copy
import numpy as np

class FanucRosInterface(Node):

    def __init__(self):
        super().__init__('fanuc_ros_inteface')
        
        
        self.cartesian_motion = False
        # traj_name="trajs/scan_traj"
        # traj_name="trajs/arc_traj"
        # traj_name="trajs/s_traj"
        traj_name="trj_smooth"
        
        self.control_time = 0.025
        robot_ip = "10.11.31.131"
        
        self.r = robot(robot_ip)
        
        self.j23_factor = 1
        self.r.allow_motion(False)        
        cp = self.r.get_current_cart_pos()
        self.r.write_cartesian_position(cp)
        self.r.allow_motion(True)

        self.get_logger().info("starting motion allowed")

        self.fb_publisher_ = self.create_publisher(JointState, 'fb_pos', 1)
        self.cmd_publisher_ = self.create_publisher(JointState, 'cmd_pos', 1)
        
        self.joint_limits=np.array([360, 360, 540, 380, 360, 450])/2.005
        
        cur_path = os.path.dirname(os.path.realpath(__file__))
        
        if self.cartesian_motion:
            fol="cart"
        else:
            fol="joints"
        
        #  RECORDED TRAJS
        dir_path = cur_path + "/" + traj_name+".csv"

        demo = pd.read_csv(dir_path, delimiter=",")
        q = demo[['0', '1', '2', '3', '4', '5']].to_numpy()


        if self.cartesian_motion:
            print("Cartesian")
            self.r.write_cartesian_position(q[0])
        else:
            print("Joints")
            
            cmd_j_pos = copy.deepcopy(q[0])
            self.adjust_j3_pos(j_pos=cmd_j_pos, j23_factor=-self.j23_factor)
            self.r.write_joint_pose(np.rad2deg(cmd_j_pos))
        
        print(q[0])
        input("press enter to execute the next trajectory")
        self.send_command(q)
        
        
    def adjust_j3_pos(self,j_pos,j23_factor):
        j_pos[2] += j23_factor * ( j_pos[1])
        
    def send_command(self,q):
        count=0
        for j in q:
            count+=1
            msg = JointState()

            if self.cartesian_motion:
                self.r.write_cartesian_position(j)
                msg.name = ["x", "y", "z", "Rx", "Ry", "Rz" ]
                cp = self.r.get_current_cart_pos()
            else:
                self.adjust_j3_pos(j_pos=j, j23_factor=-self.j23_factor)
                j=np.rad2deg(j)
                # Clip the joint values to the limits in degrees
                j = np.clip(j, -self.joint_limits, self.joint_limits)
                self.r.write_joint_pose(j)

                msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
                cp = self.r.get_current_joint_pos()
                self.adjust_j3_pos(j_pos=cp, j23_factor=self.j23_factor)
    
            msg.header.stamp = self.get_clock().now().to_msg()
    
            msg.position = j.tolist()
            self.cmd_publisher_.publish(msg)
            msg.position = cp
            self.fb_publisher_.publish(msg)
            # print(cp)
            time.sleep(self.control_time)
            print(j)
            print(count)
            input()


    def destroy_node(self):
        super().destroy_node()
        self.r.allow_motion(False)
        
    def print_mm(self):
        self.get_logger().info("quii")
        self.get_logger().info("quii")
        self.get_logger().info("quii")
        self.get_logger().info("quii")
        

def main(args=None):
    rclpy.init(args=args)
    
    cm = FanucRosInterface()

    rclpy.spin(cm)

    cm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





