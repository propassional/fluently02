# MB Run this program for moving the cobot, no need to ROS2-source it, since it writes directly into the cobot memory
# Start REMOTE_CTRL program on teach pendant, setup override as indicated below, according to the csv it is used

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

MANUAL_CONTROL = False  # If not timed, user input is requested

# Never use 0.00001, since the cobot receives too many points and will cut some of them, running into collision potential
# An example of collision trajectory in this case is in all_scan_positions.csv, when joint4 passes near over the propeller: pass near becomes hit!
CONTROL_TIME = 0.005  # 0.2 # [sec]

# MB Get the directory of the current script
script_dir = os.path.dirname(__file__)

# MB Vincenzo top scan, starts from top, works
dir_path = os.path.join(script_dir, 'trajs', 'vincenzo_top_scan_all_positions.csv')

# MB Vincenzo scan all position, starts from top
#dir_path = os.path.join(script_dir, 'trajs', 'all_scan_positions.csv') # CONTROL_TIME = 0.005, override 100%

# MB Select Slice vs Snake scan
# MB slice scan, starts from above
#dir_path = os.path.join(script_dir, 'trajs', 'arc_traj.csv')

# MB snake scan, starts from bottom
#dir_path = os.path.join(script_dir, 'trajs', 's_traj.csv')

#dir_path = r"D:\arc_traj.csv"
#dir_path = r"D:\ScanCartesian.csv"  # Complete file, multiple tours
# dir_path = r"D:\scan_yaw.csv"
# dir_path = r"D:\ScanCartesianStartToPirouette.csv"
# dir_path = r"D:\ScanCartesianPirouetteToStart.csv"
# dir_path = r"D:\ScanCartesianPirouetteToEnd.csv"

q = None

class FanucRosInterface():
    index_start_obj = None
    index_end_obj = None
    control_time_obj = 1

    # See example of use in ArtecStudio__FullScan__2024_08_05__14_57_32.py
    def __init__(self, dir_path, index_start, index_end, control_time):
        global q
        # super().__init__('fanuc_ros_inteface')
        self.index_start_obj = index_start
        self.index_end_obj = index_end
        self.control_time_obj = control_time

        self.cartesian_motion = False
        #self.cartesian_motion = True # MB
        traj_name = "scan"

        #CONTROL_TIME = 0.2
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

        cur_path = os.path.dirname(os.path.realpath(__file__))
        # print(cur_path)
        # exit()

        if self.cartesian_motion:
            fol = "cart"
        else:
            fol = "joints"

        #  RECORDED TRAJS
        ##dir_path = cur_path +"/../trajs/"+fol+"/"+traj_name+".csv" # Original linux code
        # dir_path = cur_path + "\..\trajs\" + fol + "\\" + traj_name + ".csv" # Windows

        demo = pd.read_csv(dir_path, delimiter=",")
        q = demo[['0', '1', '2', '3', '4', '5']].to_numpy()

        # if self.cartesian_motion:
        # if dir_path.__contains__("Cartesian"):
        #     print("Cartesian")
        #     self.r.write_cartesian_position(q[0])
        # elif dir_path.__contains__("Joints"):
        print("Joints")
        cmd_j_pos = copy.deepcopy(q[0])
        self.adjust_j3_pos(j_pos=cmd_j_pos, j23_factor=-self.j23_factor)
        #self.r.write_joint_pose(np.rad2deg(cmd_j_pos)) # Cobot moves here!
        # else:
        #     print("No Cartesian nor Joints string read: Error!")
        #     exit(-1)

        # Original code
        print(q[0])
        #input("press enter to execute the next trajectory")
        #self.send_command(q)

        # for i in range(len(q)):
        #     print(f"[{i}] = {q[i]}")
        #     #input("press enter to execute the next trajectory")
        #     self.send_command(q[i])

    def goto_positions(self):
        global q
        #input("press enter to execute the next trajectory")
        last_index = self.send_command_part1(q)
        return last_index

    def adjust_j3_pos(self, j_pos, j23_factor):
        j_pos[2] += j23_factor * (j_pos[1])

    def send_command_part1(self, q):
        index = -1
        #for index, j in enumerate(q): # Original
        #for index, j in enumerate(q[self.index_start_obj:self.index_end_obj], start=self.index_start_obj):
        if self.index_end_obj >= self.index_start_obj:
            # Normal slicing when end index is greater than or equal to start index
            for index, j in enumerate(q[self.index_start_obj:self.index_end_obj], start=self.index_start_obj):
                # index: This variable represents the current index in the enumeration
                # j: This variable represents the current element from the sliced list
                print(f"index: {index}, j: {j}")
                self.send_command_part2(index, j)
        else:
            # Reverse slicing when end index is less than start index
            for index, j in enumerate(q[self.index_end_obj:self.index_start_obj][::-1], start=self.index_end_obj):
                self.send_command_part2(index, j)
        return index

    def send_command_part2(self, index, j):
        # msg = JointState()

        if self.cartesian_motion:
            # if index < 56:
            #     j[5] = j[5] + 360
            self.r.write_cartesian_position(j)
            # msg.name = ["x", "y", "z", "Rx", "Ry", "Rz" ]
            cp = self.r.get_current_cart_pos()
        else:
            self.adjust_j3_pos(j_pos=j, j23_factor=-self.j23_factor)
            j = np.rad2deg(j)
            self.r.write_joint_pose(j) # Moves cobot

            # msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
            cp = self.r.get_current_joint_pos()
            self.adjust_j3_pos(j_pos=cp, j23_factor=self.j23_factor)

        # msg.header.stamp = self.get_clock().now().to_msg()

        # msg.position = j.tolist()
        # self.cmd_publisher_.publish(msg)
        # msg.position = cp
        # self.fb_publisher_.publish(msg)
        # print(cp)

        # now
        row_output = f"Moved to [{index}] = "
        for sub_index, value in enumerate(j):
            row_output += f"  {value:.2f}"
        print(row_output)
        if MANUAL_CONTROL:
            # print("MB: press a key for next point")
            input()
        else:
            #time.sleep(CONTROL_TIME)
            time.sleep(self.control_time_obj)

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

# #!/usr/bin/env python3
#
# import rclpy
# from rclpy.node import Node
# import os
#
# import pandas as pd
# from robot_control import robot
# from sensor_msgs.msg import JointState
#
# import time
# import copy
# import numpy as np
#
# class FanucRosInterface(Node):
#
#     def __init__(self):
#         super().__init__('fanuc_ros_inteface')
#
#
#         self.cartesian_motion = False
#         traj_name="trajs/scan_traj"
#         traj_name="trajs/arc_traj"
#         # traj_name="trajs/s_traj"
#
#         self.control_time = 0.025
#         robot_ip = "10.11.31.131"
#
#         self.r = robot(robot_ip)
#
#         self.j23_factor = 1
#         self.r.allow_motion(False)
#         cp = self.r.get_current_cart_pos()
#         self.r.write_cartesian_position(cp)
#         self.r.allow_motion(True)
#
#         self.get_logger().info("starting motion allowed")
#
#         self.fb_publisher_ = self.create_publisher(JointState, 'fb_pos', 1)
#         self.cmd_publisher_ = self.create_publisher(JointState, 'cmd_pos', 1)
#
#         cur_path = os.path.dirname(os.path.realpath(__file__))
#
#         if self.cartesian_motion:
#             fol="cart"
#         else:
#             fol="joints"
#
#         #  RECORDED TRAJS
#         dir_path = cur_path + "/" + traj_name+".csv"
#
#         demo = pd.read_csv(dir_path, delimiter=",")
#         q = demo[['0', '1', '2', '3', '4', '5']].to_numpy()
#
#
#         if self.cartesian_motion:
#             print("Cartesian")
#             self.r.write_cartesian_position(q[0])
#         else:
#             print("Joints")
#
#             cmd_j_pos = copy.deepcopy(q[0])
#             self.adjust_j3_pos(j_pos=cmd_j_pos, j23_factor=-self.j23_factor)
#             self.r.write_joint_pose(np.rad2deg(cmd_j_pos))
#
#         print(q[0])
#         input("press enter to execute the next trajectory")
#         self.send_command(q)
#
#
#     def adjust_j3_pos(self,j_pos,j23_factor):
#         j_pos[2] += j23_factor * ( j_pos[1])
#
#     def send_command(self,q):
#
#         for j in q:
#             msg = JointState()
#
#             if self.cartesian_motion:
#                 self.r.write_cartesian_position(j)
#                 msg.name = ["x", "y", "z", "Rx", "Ry", "Rz" ]
#                 cp = self.r.get_current_cart_pos()
#             else:
#                 self.adjust_j3_pos(j_pos=j, j23_factor=-self.j23_factor)
#                 j=np.rad2deg(j)
#                 self.r.write_joint_pose(j)
#
#                 msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
#                 cp = self.r.get_current_joint_pos()
#                 self.adjust_j3_pos(j_pos=cp, j23_factor=self.j23_factor)
#
#             msg.header.stamp = self.get_clock().now().to_msg()
#
#             msg.position = j.tolist()
#             self.cmd_publisher_.publish(msg)
#             msg.position = cp
#             self.fb_publisher_.publish(msg)
#             # print(cp)
#             time.sleep(self.control_time)
#             print(j)
#             input()
#
#
#     def destroy_node(self):
#         super().destroy_node()
#         self.r.allow_motion(False)
#
#     def print_mm(self):
#         self.get_logger().info("quii")
#         self.get_logger().info("quii")
#         self.get_logger().info("quii")
#         self.get_logger().info("quii")
#
#
# def main(args=None):
#     rclpy.init(args=args)
#
#     cm = FanucRosInterface()
#
#     rclpy.spin(cm)
#
#     cm.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
#
#
#
#
#
