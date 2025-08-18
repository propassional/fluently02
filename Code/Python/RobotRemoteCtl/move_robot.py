#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
import os

import pandas as pd
from robot_control import robot
# from sensor_msgs.msg import JointState

import time
import copy
import numpy as np

MANUAL_CONTROL = False # If not timed, user input is requested
CONTROL_TIME = 1.0 # 0.2 # [sec]

#dir_path = r"D:\ScanCartesian.csv" # Complete file, multiple tours
#dir_path = r"D:\scan_yaw.csv"
#dir_path = r"D:\ScanCartesianStartToPirouette.csv"
#dir_path = r"D:\ScanCartesianPirouetteToStart.csv"
#dir_path = r"D:\ScanCartesianPirouetteToEnd.csv"
dir_path = r"D:\Banfi\Github\Fluently\Code\Python\knowledge-transfer-fleuntly-main\trajs\vincenzo_all_scan_positions.csv"

class FanucRosInterface():

    def __init__(self):
        # super().__init__('fanuc_ros_inteface')

        self.cartesian_motion = True
        traj_name="scan"
        
        CONTROL_TIME = 0.2
        robot_ip = "10.11.31.131"
        
        self.r = robot(robot_ip)
        
        self.j23_factor = 1
        self.r.allow_motion(False)        
        cp = self.r.get_current_cart_pos()
        self.r.write_cartesian_position(cp)
        self.r.allow_motion(True)

        # self.get_logger().info("starting motion allowed")
        # self.fb_publisher_ = self.create_publisher(JointState, 'fb_pos', 1)
        # self.cmd_publisher_ = self.create_publisher(JointState, 'cmd_pos', 1)
        
        #cur_path = os.path.dirname(os.path.realpath(__file__))
        # print(cur_path)
        # exit()

        if self.cartesian_motion:
            fol="cart"
        else:
            fol="joints"
        
        #  RECORDED TRAJS
        ##dir_path = cur_path +"/../trajs/"+fol+"/"+traj_name+".csv" # Original linux code
        #dir_path = cur_path + "\..\trajs\" + fol + "\\" + traj_name + ".csv" # Windows

        demo = pd.read_csv(dir_path, delimiter=",")
        q = demo[['0', '1', '2', '3', '4', '5']].to_numpy()

        #if self.cartesian_motion:
        if dir_path.__contains__("Cartesian"):
            print("Cartesian")
            self.r.write_cartesian_position(q[0])
        elif True: # dir_path.__contains__("Joints"):
            print("Joints")
            cmd_j_pos = copy.deepcopy(q[0])
            self.adjust_j3_pos(j_pos=cmd_j_pos, j23_factor=-self.j23_factor)
            self.r.write_joint_pose(np.rad2deg(cmd_j_pos))
        else:
            print("No Cartesian nor Joints string read: Error!")
            exit(-1)
        
        # Original code
        print(q[0])
        input("press enter to execute the next trajectory")
        self.send_command(q)

        # for i in range(len(q)):
        #     print(f"[{i}] = {q[i]}")
        #     #input("press enter to execute the next trajectory")
        #     self.send_command(q[i])
        
    def adjust_j3_pos(self,j_pos,j23_factor):
        j_pos[2] += j23_factor * ( j_pos[1])
        
    def send_command(self,q):
        
        for index, j in enumerate(q):
            # msg = JointState()

            if self.cartesian_motion:
                # if index < 56:
                #     j[5] = j[5] + 360
                self.r.write_cartesian_position(j)
                # msg.name = ["x", "y", "z", "Rx", "Ry", "Rz" ]
                cp = self.r.get_current_cart_pos()
            else:
                self.adjust_j3_pos(j_pos=j, j23_factor=-self.j23_factor)
                j=np.rad2deg(j)
                self.r.write_joint_pose(j)

                # msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
                cp = self.r.get_current_joint_pos()
                self.adjust_j3_pos(j_pos=cp, j23_factor=self.j23_factor)
    
            # msg.header.stamp = self.get_clock().now().to_msg()
    
            # msg.position = j.tolist()
            # self.cmd_publisher_.publish(msg)
            # msg.position = cp
            # self.fb_publisher_.publish(msg)
            # print(cp)

            #now
            row_output = f"Moved to [{index}] = "
            for sub_index, value in enumerate(j):
                row_output += f"  {value:.2f}"
            print(row_output)
            if MANUAL_CONTROL:
                #print("MB: press a key for next point")
                input()
            else:
                time.sleep(CONTROL_TIME)

    # def destroy_node(self):
    #     super().destroy_node()
    #     self.r.allow_motion(False)
        
    # def print_mm(self):
    #     self.get_logger().info("quii")
    #     self.get_logger().info("quii")
    #     self.get_logger().info("quii")
    #     self.get_logger().info("quii")
        

def main(args=None):
    # rclpy.init(args=args)
    
    cm = FanucRosInterface()

    # rclpy.spin(cm)

    # cm.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()





