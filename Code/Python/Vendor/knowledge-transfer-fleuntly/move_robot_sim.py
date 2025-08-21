#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os

import pandas as pd
from std_msgs.msg import Float64MultiArray
import time
import numpy as np

class FanucRosInterface(Node):

    def __init__(self):
        super().__init__('fanuc_ros_inteface')
        
        self.sim_robot = True
        
        traj_name="trajs/scan_traj"
        traj_name="trajs/arc_traj"
        traj_name="trajs/s_traj"
        
        self.sim_cmd_publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)
        
        cur_path = os.path.dirname(os.path.realpath(__file__))
        
        #  RECORDED TRAJS
        dir_path = cur_path + "/" + traj_name+".csv"

        demo = pd.read_csv(dir_path, delimiter=",")
        q = demo[['0', '1', '2', '3', '4', '5']].to_numpy()

        print(q[0])
        input("press enter to execute the next trajectory")
        self.send_command(q)
        
        
        
    def send_command(self,q):
        time.sleep(2.0)
            
        for j in q:
            cmd_msg = Float64MultiArray()
            cmd_msg.data=j.tolist()
            self.sim_cmd_publisher_.publish(cmd_msg)
            time.sleep(0.5)
            print(j)
            input()
            
                

    def destroy_node(self):
        super().destroy_node()
        self.r.allow_motion(False)
        

def main(args=None):
    rclpy.init(args=args)
    
    cm = FanucRosInterface()

    rclpy.spin(cm)

    cm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





