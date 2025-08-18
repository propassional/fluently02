from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.backends import PyPlot
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from collections import deque
from numpy import ndarray
import spatialmath as sm
import numpy as np
import time
import copy
import csv
import os
import sys
import json
from scipy.signal import savgol_filter
import pandas as pd
from pose_array_pub import PoseArrayPublisher
from joint_publisher import JointPublisher
import rclpy

class Sphere():
    """Geometrical representation of a sphere
    """
    def __init__(self, centre: ndarray, radius: float) -> None:
        """Constructor

        Args:
            centre (ndarray): 3d centre
            radius (float)
        """
        self.centre = centre
        self.radius = radius

    def point_is_inside(self, point: ndarray):
        """Return true if point is inside or on the sphere

        Args:
            point (ndarray): 3d point

        Returns:
            bool
        """
        return (self.centre[0] - point[0])**2 + (self.centre[1] - point[1])**2 + (self.centre[2] - point[2])**2 <= self.radius**2

    def plot(self, env):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = (self.radius * np.cos(u) * np.sin(v)) + self.centre[0]    # Sphere's x coordinates
        y = (self.radius * np.sin(u) * np.sin(v)) + self.centre[1]    # Sphere's y coordinates
        z = (self.radius * np.cos(v)) + self.centre[2]               # Sphere's z coordinates
        env.ax.plot_surface(x, y, z, color='r', alpha=0.5)

class Box():
    def __init__(self, centre: tuple[float, float], length:float, width: float, heigth: float):
        self.c = centre
        self.l = length
        self.w = width
        self.h = heigth
        
    def point_is_inside(self, point: ndarray):
        return ((self.c[0] - self.l/2 <= point[0] <= self.c[0] + self.l/2) and 
                (self.c[1] - self.w/2 <= point[1] <= self.c[1] + self.w/2) and 
                (self.c[2] - self.h/2 <= point[2] <= self.c[2] + self.h/2))

    def plot(self, env):
        edges = []
        u = np.array([1, -1, -1,  1])
        v = np.array([1,  1, -1, -1])
        
        x = u * self.l/2 + self.c[0]
        y = v * self.w/2 + self.c[1]
        z = np.full(4, -self.h/2 + self.c[2])
        edges.append([list(zip(x, y, z))])
        z = np.full(4,  self.h/2 + self.c[2])
        edges.append([list(zip(x, y, z))])
        
        x = u * self.l/2 + self.c[0]
        z = v * self.h/2 + self.c[2]
        y = np.full(4, -self.w/2 + self.c[1])
        edges.append([list(zip(x, y, z))])
        y = np.full(4,  self.w/2 + self.c[1])
        edges.append([list(zip(x, y, z))])

        y = u * self.w/2 + self.c[1]
        z = v * self.h/2 + self.c[2]
        x = np.full(4, -self.l/2 + self.c[0])
        edges.append([list(zip(x, y, z))])
        x = np.full(4,  self.l/2 + self.c[0])
        edges.append([list(zip(x, y, z))])

        for edge in edges:
            env.ax.add_collection3d(Poly3DCollection(edge, alpha=0.5))
        # for i, p in enumerate(verts):
            # print(p)
            # env.ax.scatter(p[0], p[1], p[2])
            # env.ax.text(p[0], p[1], p[2], str(i))
        
        # env.ax.voxels(data)
       
class SimRobotBackend(ERobot):
    """Implementation of a robot trough a urdf file using rtb, usable standalone for debugging purpose, used by the SimRobot
    class for computations, inherit from roboticstoolbox.robot.Erobot.Erobot
    """
    def __init__(self, urdf_file:str, tcp_frame_urdf:str = None, 
                 x_free_space: tuple = (float('-inf'), float('inf')), 
                 y_free_space: tuple = (float('-inf'), float('inf')), 
                 z_free_space: tuple = (float('-inf'), float('inf')),
                 home_position:ndarray=np.array([1.17, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2]),
                 tcp_frame_transf: sm.SE3 = sm.SE3(np.eye(4)),
                 robot_base=sm.SE3(np.eye(4)))-> None:
        """Constructor

        Args:
            urdf_file (str): configuration file
            tcp_frame (str, optional): String with name of tcp frame . Defaults to None.
            x_free_space (tuple, optional): space limits in the axis. Defaults to (float('-inf'), float('inf')).
            y_free_space (tuple, optional): space limits in the axis. Defaults to (float('-inf'), float('inf')).
            z_free_space (tuple, optional): space limits in the axis. Defaults to (float('-inf'), float('inf')).
            home_position (ndarray, optional): Defaults to np.array([1.17, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2]).
        """
        super().__init__(ERobot.URDF(urdf_file, gripper=tcp_frame_urdf))
        self.name = urdf_file.replace(".urdf", "").split("/")[-1]
        self.q = home_position
        self.home_position = home_position
        self.collision_objs = []
        self.x_free_space = x_free_space
        self.y_free_space = y_free_space
        self.z_free_space = z_free_space
        self.check_collision = True
        self.tcp_frame_transf = tcp_frame_transf
        self.env = None
        self.robot_base = robot_base

        self.use_j_limit = False
        if self.qlim is None: # if the limits are not specified in the urdf
            self.qlim = np.array([[float('-inf'), float('-inf'), float('-inf'), float('-inf'), float('-inf'), float('-inf')],
                              [ float('inf'),  float('inf'),  float('inf'),  float('inf'),  float('inf'),  float('inf')]])

    def set_joints_limit(self, limits: ndarray):
        self.qlim = copy.deepcopy(limits)
        self.use_j_limit = True

    def set_joint_limit(self, joint_index: int, limit: tuple[float, float]):
        qlim = copy.deepcopy(self.qlim)
        qlim[0][joint_index] = limit[0]
        qlim[1][joint_index] = limit[1]
        self.qlim = copy.deepcopy(qlim)

    def set_joint_limits_usage(self, value: bool):
        self.use_j_limit = value

    def set_geom_free_space(self, axis: chr, free_space: tuple[float, float]):
        if axis == 'x':
            self.x_free_space = free_space
        if axis == 'y':
            self.y_free_space = free_space
        if axis == 'z':
            self.z_free_space = free_space

    def get_tcp_pose(self, q: ndarray = None) -> sm.SE3:
        """Return tcp pose in base frame

        Args:
            q (ndarray): joint configuration, if none the current one in the backend will be used. Defaults to None.

        Returns:
            sm.SE3: TCP pose
        """
        if q is not None:
            return self.fkine(q) * self.tcp_frame_transf
        else:
            return self.fkine(self.q) * self.tcp_frame_transf

    def add_collision_sphere(self, centre: ndarray, radius: float):
        """Add a collision sphere to the simulated environment

        Args:
            centre (ndarray): 3d centre of the sphere
            radius (float): radius of the sphere
        """
        s = Sphere(centre=centre, radius=radius)
        self.collision_objs.append(s)
        if self.env is not None:
            s.plot(self.env)
        return s
                
    def add_collision_box(self, centre: ndarray, length: float,  width:float, heigth: float):
        """Add a collision box to the simulated environmen
        """
        b = Box(centre=centre, length=length, width=width, heigth=heigth)
        self.collision_objs.append(b)     
        if self.env is not None:
            b.plot(self.env)
        return b
           
    def check_joint_collisions(self, q: ndarray):
        """Check if the joint configuration passed is in collision with the limits posed or the added spheres

        Args:
            q (ndarray): joint configuration

        Returns:
            bool: True if in collision, False otherwise
        """
        for i, t in enumerate(self.fkine_all(q).t):
            if ((self.x_free_space[0] >= t[0] or t[0] >= self.x_free_space[1]) or # check the the joint is in the free space
                (self.y_free_space[0] >= t[1] or t[1] >= self.y_free_space[1]) or 
                (self.z_free_space[0] >= t[2] or t[2] >= self.z_free_space[1])):
                # print(f"Configuration {q} is in collision with space limits in joint {i}, t: {t}, x_free_space: {self.x_free_space}, y_free_space: {self.y_free_space}, z_free_space: {self.z_free_space}")
                return True
            for obj_id, obj in enumerate(self.collision_objs): # check every sphere added
                if obj.point_is_inside(t):
                    # print(f"Collision with {type(obj)} {obj_id} limits in joint {i}")
                    return True
        return False
    
    def check_pose_collisions(self, T: sm.SE3):
        """Check if the pose T is in collision

        Args:
            T (sm.SE3): target pose

        Returns:
            bool: True if in collision, False otherwise
        """
        if ((self.x_free_space[0] >= T.t[0] or T.t[0] >= self.x_free_space[1]) or # check the point is in free space
            (self.y_free_space[0] >= T.t[1] or T.t[1] >= self.y_free_space[1]) or 
            (self.z_free_space[0] >= T.t[2] or T.t[2] >= self.z_free_space[1])):
            print(f"Pose in collision with space limits; T pos: {T.t}, x_free_space: {self.x_free_space}, y_free_space: {self.y_free_space}, z_free_space: {self.z_free_space}")
            return True
        for obj_id, obj in enumerate(self.collision_objs): # check every sphere added
            if obj.point_is_inside(T.t):
                print(f"Pose in collision with {type(obj)} {obj_id}")
                return True
        return False

    def generate_q_traj(self, q0: ndarray, qf: ndarray, t: int = 200):
        """Generate a trajectory between q0 and qf of t steps

        Args:
            q0 (ndarray): initial configuration
            qf (ndarray): final configuration
            t (int, optional): legth of the trajectory. Defaults to 200.

        Returns:
            roboticstoolbox.trajectory: trajectory
        """
        traj = rtb.jtraj(q0=q0, qf=qf, t=t)
        return traj

    def ik_collision_free(self, Tep: sm.SE3, n_trials: int = 50, q0: ndarray = None, mask=None, tol=1e-6) -> tuple[bool, ndarray]:
        """Generate a inverse kinematics collision fre solution for the pose Tep

        Args:
            Tep (sm.SE3): target Tep in base frame
            n_trials (int, optional): # of trials to find the solution. Defaults to 10.

        Returns:
            bool, ndarray: whether or not the solution is valid and the solution
        """
        start = time.time()
        T = copy.deepcopy(Tep)
        perturbation = 0.2 * (np.pi - (-np.pi))
        sol_valid = False
        trial = 0
        T = (T * self.tcp_frame_transf.inv())
        q0 = self.q if q0 is None else q0
        starting_q = q0
        if not self.check_pose_collisions(T): # if the point itself is in collision then we don't even try
            while not sol_valid and trial < n_trials: # otherwise we try n_trials time
                sol = self.ik_LM(T, q0=starting_q, joint_limits=self.use_j_limit, tol=tol, mask=mask) # generating the solution
                if sol[1]: # if there is an ik solution
                    sol_valid = not self.check_joint_collisions(sol[0]) # we check it, if it's not in collision sol_valid become true and we exit the loop 
                else: # ik_LM is already looping to find a solution
                    pass
                    # print("No solution for inverse kinematics found, T:")
                    # print(T)
                    # break
                starting_q = q0 + np.random.uniform(-perturbation, perturbation, size=q0.shape)
                trial += 1
        else:
            print("The pose is in collision, impossible for the robot to reach the pose")
        if not sol_valid:
            sol = [np.full(6, np.nan)]
        # print(f"trial: {trial}, success: {sol_valid}, time: {time.time()-start}", end=' ')
        return sol_valid, sol[0]

    def find_all_ik_solutions(self, Tep: sm.SE3, n_sols=10, tfms:list[sm.SE3]=[sm.SE3([0,0,0])], tol=1e-6, mask=None):
        T = copy.deepcopy(Tep)
        T = (T * self.tcp_frame_transf.inv())
        solutions = []
        if  self.check_pose_collisions(T): # if the point itself is in collision then we don't even try
            # print("pose in collision")
            return solutions
        for tfm in tfms:
            T_var = T * tfm
            for _ in range(n_sols):
                starting_q = np.empty(self.q.shape)
                for idx in range(len(self.q)):
                    min_q = self.qlim[0][idx]
                    max_q = self.qlim[1][idx]
                    starting_q[idx] = np.random.uniform(min_q, max_q)
                    # print("starting_q:", starting_q)
                sol = self.ik_LM(T_var, q0=starting_q, joint_limits=True, tol=tol, mask=mask) # generating the solution
                if sol[1]: # if there is an ik solution
                    q_sol = np.round(sol[0], decimals=6)
                    if not any(np.allclose(q_sol, q, atol=1e-2) for q in solutions):
                        solutions.append(sol[0])
        return solutions

    def world_T_robot(self, T: sm.SE3) -> sm.SE3:
        """
        given a pose in world fram in form of 4x4 matrix gives back the pose in base frame of the robot
        """
        return self.robot_base.inv() * T
    
    def robot_T_world(self, T: sm.SE3) -> sm.SE3:
        """
        given a pose in robot base frame in form of 4x4 matrix gives back the pose in base frame of the robot
        """
        return  self.robot_base * T

    def generate_continous_trajectory_jtraj(self, poses: list[ndarray], q0=None, speed=0.05, t_step=200, remove_spikes=False) -> tuple[bool, ndarray[ndarray], tuple[int, int]]:
        j_confs = []
        traj_q = []
        success = False
        q0 = self.q # for smoother transtion between configuration
        qd0 = np.full(6, 0)
        qdf = np.full(6, speed)
        start_correction_traj, end_correction_traj = -1, -1
        for pose in poses:
            success, sol = self.ik_collision_free(pose, q0=q0)
            if success:
                if remove_spikes and (abs(sol[0] - q0))[-1] > 0.1:
                    start_correction_traj = len(j_confs)
                    print("spike in last joint, unravel tcp")
                    spike_correction_traj = rtb.jtraj(q0=q0, qf=sol[0], t=200)
                    for q in spike_correction_traj.q:
                        j_confs.append(q)
                    j_confs.extend([spike_correction_traj.q[-1]]*5) # stand still
                    end_correction_traj = len(j_confs)
                j_confs.append(sol[0])
                q0 = sol[0]
         
        for i, next_j_conf in enumerate(j_confs[1:]):
            # j_confs[1:] iterate from the second elemtn to the last, i start from 0 and get to len(j_confs) - 1
            if i == len(j_confs) - 1:
                qdf = np.full(6, 0)
            traj = rtb.jtraj(q0=j_confs[i], qf=next_j_conf, qd0=qd0, qd1=qdf, t=2)
            
            traj_q.extend(traj.q[1:])
            qd0 = qdf
            success = True
        return success, np.array(traj_q), (start_correction_traj, end_correction_traj)

    def init_plot_env(self) -> None:
        """init the pyplot environment for debugging purpose
        """
        self.env = PyPlot.PyPlot()
        self.env.launch()
        self.env.add(self)

    def plot_q(self, q:ndarray = None, hold=True) -> None:
        """Plot a joint configuration in pyplot

        Args:
            q (ndarray, optional): the target q. Defaults to None.
        """
        if q is not None:
            self.q = q
        self.env.step()
        if hold:
            self.env.hold()

    def plot_traj(self, traj: list[ndarray], loop=False, hold=True) -> None:
        """Plot a trajectory in pyplot

        Args:
            traj (Trajectory): The trajectory to be plotted
        """
        if loop:
            ans = ''
            while loop and ans == '':
                for q in traj:
                    self.q = q
                    self.env.step()
                ans = input(">>>")
        else:
            for q in traj:
                self.q = q
                self.env.step()
                time.sleep(0.2)
            if hold:
                self.env.hold()

    def move_away_from_pos_trj(self, P_obj: ndarray, distance: float, step_size: float=0.002):
        trj = []
        for i in np.arange(0, distance, step_size):
            T_TCP = self.get_tcp_pose()
            P_TCP = T_TCP.t
            R_TCP = T_TCP.R
            direction = P_TCP - P_obj
            unit_direction = direction / np.linalg.norm(direction)
            unit_direction[2] = 0 # we keep z as the same
            P_new = P_TCP + (unit_direction * step_size)
            new_T = sm.SE3.Rt(sm.SO3(R_TCP), P_new)
            suc, q = self.ik_collision_free(new_T, q0=robot.q)
            if suc:
                trj.append(q)
                print("appended new")
                robot.q = q
            print(i)
        plot_joint_traj(trj)
        return trj


    def move_up(self, distance: float):
        pass

def plot_joint_traj(qs: list[ndarray], title="joint_trj", hold=True):
    qs_copy = np.array(qs)
    t = range(len(qs_copy))
    
    fig, axs = plt.subplots(6, sharex=True)
    fig.suptitle(title)
    for i, ax in enumerate(axs):
        ax.plot(t, qs_copy[:, i])
        ax.set_title(f"Joint_{i}")
        
    plt.grid()
    if hold:
        plt.show()

def plot_cart_traj(poses: list[sm.SE3], title="poses_traj", conversion='rpy', convert=True, hold=True):
    if conversion == 'rpy':
        n_subfigures = 6
        tcp_var = ["x", "y", "z", "r", "p", "y"]
    elif conversion == 'q':
        n_subfigures = 7
        tcp_var = ["x", "y", "z", "q1", "q2", "q3", "q4"]

    traj_tcp_cart = np.empty((len(poses), n_subfigures))
    fig, axs = plt.subplots(n_subfigures, sharex=True)
    fig.suptitle(title)
    if convert:
        for i, pose in enumerate(poses):
            T = sm.SE3(pose)
            if conversion == 'rpy':
                traj_tcp_cart[i, :] = np.hstack((T.t, sm.SO3.eul(sm.SO3(T.R))))
            elif conversion == 'q':
                traj_tcp_cart[i, :] = np.hstack((T.t, sm.SO3.UnitQuaternion(sm.SO3(T.R))))
    else:
        traj_tcp_cart = poses
        
    cs = ["red", "green", "blue"]
    for i, ax in enumerate(axs):
        ax.plot(range(traj_tcp_cart.shape[0]), list(traj_tcp_cart[:, i]))
        ax.set_title(tcp_var[i])
    plt.grid()
    if hold:
        plt.show()

def gen_rings_poses(obj_pose:sm.SE3, radius, h_res=8, v_res=5, 
                    x_stretch=1, y_stretch=1, z_stretch=1,
                    starting_angle_v=np.pi/3, ending_angle_v=np.pi-np.pi/3, 
                    starting_angle_h=0, ending_angle_h=2*np.pi):
    u = np.linspace(starting_angle_h, ending_angle_h, h_res+1)[1:] # the +1 is to get to the actual number
    v = np.linspace(starting_angle_v, ending_angle_v, v_res)
    poses = deque()
    rings = []
    for phi in v:
        ring = []
        for theta in u: 
            x = (np.sin(phi) * np.cos(theta)) * radius * x_stretch # to "flatten" the ellipsoid increase this
            y = (np.sin(phi) * np.sin(theta)) * radius * y_stretch # to "flatten" the ellipsoid increase this
            z = (np.cos(phi)) * radius * z_stretch
            direction_vector = np.array([-x, -y, -z])
            direction_vector /= np.linalg.norm(direction_vector)
            z_angle = np.arctan2(direction_vector[1], direction_vector[0])
            y_angle = np.arctan2(np.sqrt(direction_vector[0]**2 + direction_vector[1]**2), direction_vector[2])
            Rot_mat = sm.SO3.Rz(z_angle) * sm.SO3.Ry(y_angle) * sm.SO3.Rx(0) * sm.SO3.Rz(0)
            # Rot_mat=Rot_mat*sm.SO3.Rz(np.pi/2)
            T = sm.SE3().Rt(Rot_mat, np.array([x, y, z]))
            T = obj_pose * T
            ring.append(T)
            poses.append(T)
            # if phi == 0 or phi == np.pi:
            #     break
        rings.append(ring)
        u = u[::-1]
    return rings

def gen_sphere(obj_pose:sm.SE3, radius:float, stretch:float, res:float):
    angle_step = res / radius
    v = np.arange(0, np.pi, angle_step)
    poses = deque()
    for phi in v:
        curr_r = radius * np.sin(phi) * stretch
        if curr_r < 1e-6:
            angle_step = np.pi*2
        else:
            angle_step = res / curr_r
        u = np.arange(0, np.pi*2, angle_step)
        for theta in u: 
            x = (np.sin(phi) * np.cos(theta)) * radius * stretch # to "flatten" the ellipsoid increase this
            y = (np.sin(phi) * np.sin(theta)) * radius * stretch # to "flatten" the ellipsoid increase this
            z = (np.cos(phi)) * radius
            direction_vector = np.array([-x, -y, -z])
            direction_vector /= np.linalg.norm(direction_vector)
            z_angle = np.arctan2(direction_vector[1], direction_vector[0])
            y_angle = np.arctan2(np.sqrt(direction_vector[0]**2 + direction_vector[1]**2), direction_vector[2])
            Rot_mat = sm.SO3.Rz(z_angle) * sm.SO3.Ry(y_angle) * sm.SO3.Rx(0)
            T = obj_pose * sm.SE3().Rt(Rot_mat, np.array([x, y, z]))
            poses.append(T)
    return poses

def gen_arc_poses(obj_pose:sm.SE3, radius, h_res=1, v_res=10):
    theta = 0
    v = np.linspace(np.pi/6, np.pi-np.pi/6, v_res+2)
    arc = []
    for phi in v:
        if abs(phi) == np.pi : # we don't want to hit the pole we remove pi
            continue
        x = ((np.sin(phi) * np.cos(theta)) * radius)
        y = ((np.sin(phi) * np.sin(theta)) * radius)
        z = ((np.cos(phi)) * radius)
        direction_vector = np.array([-x, -y, -z])
        direction_vector /= np.linalg.norm(direction_vector)

        z_angle = np.arctan2(direction_vector[1], direction_vector[0])
        y_angle = np.arctan2(np.sqrt(direction_vector[0]**2 + direction_vector[1]**2), direction_vector[2])
        Rot_mat = sm.SO3.Rz(z_angle)* sm.SO3.Ry(y_angle) * sm.SO3.Rx(0)
        Rot_mat = Rot_mat * sm.SO3.Rz(np.pi/2) # to flip x and y axis for the scanner
        T = sm.SE3().Rt(Rot_mat, np.array([x, y, z]))
        T = obj_pose * T 
        arc.append(T)
    return arc

def gen_s_poses(obj_pose:sm.SE3, radius, num_points: int=25):
    ts = np.linspace(-2*np.pi, 2*np.pi, num_points)
    s = []
    i = 0
    for t in ts:
        z = 0.8*t/(2*np.pi) * radius
        y = np.sin(t)*0.5 * radius
        x = abs(np.sqrt(radius**2 - z**2 - y**2))
        direction_vector = np.array([-x, -y, -z])
        direction_vector /= np.linalg.norm(direction_vector)

        z_angle = np.arctan2(direction_vector[1], direction_vector[0])
        y_angle = np.arctan2(np.sqrt(direction_vector[0]**2 + direction_vector[1]**2), direction_vector[2])
        Rot_mat = sm.SO3.Rz(z_angle)* sm.SO3.Ry(y_angle) * sm.SO3.Rx(0)
        Rot_mat = Rot_mat * sm.SO3.Rz(np.pi/2) # to flip x and y axis for the scanner
        T = sm.SE3().Rt(Rot_mat, np.array([x, y, z]))
        T = obj_pose * T 
        s.append(T)
    return s

def connect_rings(rings, connection_t=20, r=8):
    """connect the rings together smoothly

    Args:
        rings (_type_): the rings to connect
        connection_t (int, optional): This defines the number of intermediate steps used to interpolate the transition between two rings. Defaults to 20.
        r (int, optional): This determines how many poses at the start and end of each ring are excluded from the transition connection. 
                           The function trims r poses from both ends of each ring except for the first and last ones, ensuring smoother transitions between rings.. 
                           Defaults to 8.

    Returns:
        list[ndarray]: the connected rings
    """
    flat_rings, i = [], 1
    while i < len(rings):
        if i == 1: # on the first we add all the poses
            flat_rings.extend(rings[i-1][:-r])
        else: # after that it's be added in the connection path
            flat_rings.extend(rings[i-1][r:-r])
        midpoint = rtb.ctraj(rings[i-1][-1], rings[i][0], t=3)[1]
        connection = list(rtb.ctraj(rings[i-1][-r], midpoint, t=connection_t//2)) + list(rtb.ctraj(midpoint, rings[i][r], t=connection_t//2)[1:]) # remove the midpoint as it is already in the first connection
        polished_ts = savgol_filter([T.t for T in connection], window_length=connection_t//3, polyorder=3, axis=0) # we interpolate the positions but leave the orientation
        for T, t in zip(connection, polished_ts):
            flat_rings.append(sm.SE3.Rt(sm.SO3(T.R), t))
        i += 1
    flat_rings.extend(rings[i-1][r:])
    return flat_rings

def setup_env():
    obj_pose = sm.SE3([-0.04, 0.8, 1.28]) * sm.SE3.Rz(np.pi/3)

    robot_base = sm.SE3.Rt(np.eye(3), [0, 0, 1])
    # robot = SimRobotBackend(urdf_file=os.getcwd() + "/crx20ia_l.urdf", tcp_frame_urdf="tool_tcp", robot_base=robot_base)
    robot = SimRobotBackend(urdf_file=os.getcwd() + "/prima_additive.urdf", tcp_frame_urdf="tool_tcp", robot_base=robot_base)
    robot.set_joint_limit(1, [-np.pi/2, np.pi/2])

    obj_pose_ROBOT = robot.world_T_robot(obj_pose)
    robot.set_joint_limits_usage(True)

    robot.add_collision_sphere(obj_pose_ROBOT.t, radius=0.15)
    robot.add_collision_box([obj_pose_ROBOT.t[0], obj_pose_ROBOT.t[1], -0.5], 0.76, 1.02, 0.94)
    robot.add_collision_box([obj_pose_ROBOT.t[0], obj_pose_ROBOT.t[1], obj_pose_ROBOT.t[2]-0.15-0.18], 0.04, 0.04, 0.36)
    
    # robot.init_plot_env()
    return robot, obj_pose_ROBOT

def read_trj_cart(filepath, skip_header=1):
    return np.genfromtxt(filepath, delimiter=',', skip_header=skip_header)

def write_trj_csv(filepath, data):
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(data)

class JconfTree:
    class _Node:
        def __init__(self, cart_pos, solutions):
            self.cart_pos = cart_pos
            self.solutions = solutions
            # self.J = Jacobian
        
        def __repr__(self):
            return str(self.solutions)

    def __init__(self, robot: SimRobotBackend, filename=None, poses=[], transformations=[], J_threshold=0.01, n_confs_per_pose=10, ik_tol=1e-6, ik_mask=None):
        start = time.time()
        print(f"Tree init started")
        self.robot = robot
        self.graph = {}
        self.res = 0.001
        self.max_dist_neigh = 0.01
        if filename is not None:
            print(f"Reading file")
            self.load_from_file(filename=filename)
        else:
            self.transformations = transformations
            self.J_threshold = J_threshold
            self.n_confs_per_pose = n_confs_per_pose
            self.ik_tol = ik_tol
            self.ik_mask = ik_mask
            self.add_poses(poses=poses)
            # self.graph = sorted(self.graph, key=lambda d_key: (d_key[2], d_key[1], d_key[0]))
            print()
        print(f"Tree initialized {time.time()-start:05.2f} s")

    def create_dict_key(self, x, y, z):
        # convert into mm
        key_x = int(x * 1/self.res)
        key_y = int(y * 1/self.res)
        key_z = int(z * 1/self.res)
        return (key_x, key_y, key_z)

    def add_poses(self, poses):
        start = time.time()
        print("start adding poses")
        for idx, pose in enumerate(poses):
            print(f"Generating solution and variation for pose {idx+1:03d}/{len(poses):03d}", end='\r')
            self.add_node(pose)
        print(f"Poses added {time.time()-start:05.2f} s")

    def update_transformations(self, transformations):
        self.transformations = transformations

    def add_node(self, T):
        solutions = []
        all_sols = self.robot.find_all_ik_solutions(T, n_sols=self.n_confs_per_pose, tfms=self.transformations, tol=self.ik_tol, mask=self.ik_mask)
        for sol in all_sols:
            J = self.robot.jacob0(sol)
            if np.min(J) < self.J_threshold:
                solutions.append(sol)
        node = JconfTree._Node(T.t, solutions)
        self.graph[self.create_dict_key(T.t[0], T.t[1], T.t[2])] = node

    def generate_R(self, t):
        pass

    def get_node(self, x, y, z):
        try:
            return self.graph[self.create_dict_key(x, y, z)]
        except KeyError:
            # print()
            # print("old key:", x, y, z)
            for i in np.arange(0, self.max_dist_neigh, self.res): # we look at maximum 1 cm away and with a res of 1 mm because of how we setup the tree
                dist = i + self.res  
                kernel = [-dist, 0, dist]
                for dx in kernel:
                    for dy in kernel:
                        for dz in kernel:
                            new_key = self.create_dict_key(x+dx, y+dy, z+dz)
                            if new_key in self.graph:
                                # print("new_key:", new_key)
                                return self.graph[new_key]
            raise KeyError('No neighbours found within max_dist_neigh defined in JConfTree class')
   
    def gen_path(self, positions, jump_threshold=50):
        start = time.time()
        best_cost, best_trj = -1, None
        print("Start trj generation from tree")
        # default_stdout = sys.stdout
        # f = open('path_planner.log', 'w')
        # sys.stdout = f
        # costs = [0] * len(positions) # we need to track the costs between jumps (a simple cost tracked with max search does not work because of backtrack)
        trjs = []
        for n_q0, q0 in enumerate(self.get_node(*positions[0]).solutions):
            trj = [q0]
            selected_idxs = [0] * len(positions) # we need to track the solution that we used in position n to not try it again when we backtrack
            i = 1
            max_cost = 0
            while i < len(positions): # when we covered all the poses we are done
                print(f"Trying with q0 {n_q0:03d} / {len(self.get_node(*positions[0]).solutions)-1:03d}, positions[{i}]", end="\r")
                # print(f"{i:03d} / {len(positions):03d}", end='\r')
                # print("="*20, i, "="*20)
                node = self.get_node(*positions[i])
                if i==0:
                    # print("BACKTRACK TO 0")
                    break
                previous_q = trj[i-1]

                candidates = sorted(node.solutions, key=lambda q: np.max(abs(q-previous_q)))
                while selected_idxs[i] < len(candidates): # for the pose we are adding we look at all the solutions
                    max_jump = np.max(np.abs(previous_q - candidates[selected_idxs[i]]))
                    # print(selected_idxs[i], ":", max_jump)
                    
                    # if max_jump < jump_threshold: # if we are below the threshold we are moving slightly
                        # costs[i] = max_jump
                    max_cost = max(max_jump, max_cost)
                    trj.append(candidates[selected_idxs[i]])
                    # print("ADDED")
                    selected_idxs[i] += 1 # we add one so that if we are coming back we start from the next one
                    break
                    # else: # being sorted if candidates[i] does not meet the criteria neither will 1,2,3,4,...
                        # trj.insert(i, previous_q)
                        # print("SKIP")
                        # break
                if len(trj) > i: # we added a solution in this step
                    # print("i + 1")
                    i += 1 # next step
                else:
                    # print("BACKTRACK")
                    selected_idxs[i] = 0
                    i -= 1 # backtrack
                    trj.pop()
            if len(trj) == len(positions):
                trjs.append((max_cost, trj))
        # sys.stdout = default_stdout
        print()
        print(f"Trajectory ready {time.time()-start:05.2f} s")
        # print(selected_idxs)
        if len(trjs) != 0:
            best_trj, best_cost = sorted(trjs, key= lambda item: item[0])[0]
        return best_cost, best_trj

    def get_all_poses(self):
        var_cart_poses = []
        for solutions in self.solutions:
            for sol in solutions:
                var_cart_poses.append(sol.cart_pose)

    def save(self, filename: str):
        # print(self.robot)
        ik_mask = list(self.ik_mask) if self.ik_mask is not None else None
        data = {
                    "transformations": [T.A.tolist() for T in self.transformations],
                    "robot_name": robot.name,
                    "J_threshold": self.J_threshold,
                    "n_confs_per_pose": self.n_confs_per_pose,
                    "ik_mask": ik_mask,
                    "ik_tol": self.ik_tol,
                    "graph": {str(pos): [list(q) for q in self.graph[pos].solutions] for pos in self.graph}
        }
        with open(filename, "w") as f:
            json.dump(data, f)

    def load_from_file(self, filename):
        with open(filename, "r") as f:
            data = json.load(f)
            self.transformations = [sm.SE3(T) for T in data['transformations']]
            robot.name = data["robot_name"]
            self.J_threshold = data["J_threshold"]
            self.n_confs_per_pose = data["n_confs_per_pose"]
            self.ik_mask = data["ik_mask"]
            self.ik_tol = data["ik_tol"]
            for pos_str in data["graph"]:
                pos = tuple([int(coord) for coord in pos_str.strip("()").split(", ")])
                self.graph[pos] = JconfTree._Node(pos, [np.array(q_arr) for q_arr in data["graph"][pos_str]])

def smooth_trj(og_trj, w_len=20, order=2):
    """Apply savgol filter to a trajectory
        Rely to documentation for in deth explanation:
        https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html
    Args:
        og_trj (_type_): trajectory to apply the filter on
        w_len (int, optional): The length of the filter window. Defaults to 20.
        order (int, optional): The order of the polynomial used to fit the samples. Defaults to 2.

    Returns:
        _type_: _description_
    """
    trj = np.empty_like(og_trj)
    for j in range(og_trj[0].shape[0]):
        joint_trj = np.array(og_trj)[:, j]
        polished = savgol_filter(joint_trj, window_length=w_len, polyorder=order)
        trj[:, j] = polished
    return trj

def find_trj_cost(trj):
    cost, i = 0, 1
    while i < len(trj):
        cost = max(cost, np.max(np.abs(trj[i-1] - trj[i])))
        i += 1
    return cost

def smooth_trj(og_trj, w_len=20, order=2):
    trj = np.empty_like(og_trj)
    for j in range(og_trj[0].shape[0]):
        joint_trj = np.array(og_trj)[:, j]
        polished = savgol_filter(joint_trj, window_length=w_len, polyorder=order)
        trj[:, j] = polished
    return trj

if __name__ == "__main__":
    """Setup env"""
    obj_pose = sm.SE3([-0.04, 0.8, 1.28]) * sm.SE3.Rz(np.pi/3)

    robot_base = sm.SE3.Rt(np.eye(3), [0, 0, 1])
    # robot = SimRobotBackend(urdf_file=os.getcwd() + "/crx20ia_l.urdf", tcp_frame_urdf="tool_tcp", robot_base=robot_base)
    robot = SimRobotBackend(urdf_file=os.getcwd() + "/crx20ia_l.urdf", tcp_frame_urdf="tool_tcp", robot_base=robot_base)
    
    robot.set_joint_limit(1, [-np.pi/2, np.pi/2]) # the robot has limited movement on the first joint to prevent shoulder down conf
    robot.set_joint_limits_usage(True)

    obj_pose_ROBOT = robot.world_T_robot(obj_pose)

    robot.add_collision_sphere(obj_pose_ROBOT.t, radius=0.15)
    robot.add_collision_box([obj_pose_ROBOT.t[0], obj_pose_ROBOT.t[1], -0.5], 0.76, 1.02, 0.94)
    robot.add_collision_box([obj_pose_ROBOT.t[0], obj_pose_ROBOT.t[1], obj_pose_ROBOT.t[2]-0.15-0.18], 0.04, 0.04, 0.36)
    
    transformations = [
                        # sm.SE3(np.eye(4)),
                        # sm.SE3.Rz(1 * np.pi / 6),
                        # sm.SE3.Rz(1 * np.pi / 4 ),
                        # sm.SE3.Rz(1 * np.pi / 3 ),
                        sm.SE3.Rz(1 * np.pi / 2 ),
                        sm.SE3.Rz(2 * np.pi / 3 ),
                        sm.SE3.Rz(3 * np.pi / 4 ),
                        sm.SE3.Rz(5 * np.pi / 6 ),
                        sm.SE3.Rz(1 * np.pi / 1 ),
                        sm.SE3.Rz(7 * np.pi / 6 ),
                        sm.SE3.Rz(5 * np.pi / 4 ),
                        sm.SE3.Rz(4 * np.pi / 3 ),
                        sm.SE3.Rz(3 * np.pi / 2 ),
                        # sm.SE3.Rz(5 * np.pi / 3 ),
                        # sm.SE3.Rz(7 * np.pi / 4 ),
                        # sm.SE3.Rz(11 * np.pi / 6),
                        # sm.SE3.Rx(np.pi/12),
                        # sm.SE3.Rx(-np.pi/12),
                        # sm.SE3.Ry(np.pi/12),
                        # sm.SE3.Ry(-np.pi/12),
                        ]
    
    """Generate the rings and connect them"""
    h_res, v_res, radius = 250, 6, 0.3
    x_stretch, y_stretch, z_stretch = 1, 1, 0.5
    starting_angle_v, ending_angle_v = np.pi/6, np.pi-np.pi/6
    starting_angle_h, ending_angle_h = 0, 2*np.pi
    rings = gen_rings_poses(h_res=h_res, v_res=v_res, obj_pose=obj_pose_ROBOT, radius=radius, 
                            x_stretch=x_stretch, y_stretch=y_stretch, z_stretch=z_stretch, 
                            starting_angle_v=starting_angle_v, ending_angle_v=ending_angle_v, 
                            starting_angle_h=starting_angle_h, ending_angle_h=ending_angle_h)
    flat_rings = connect_rings(rings, connection_t=30, r=15)
    # sphere = gen_sphere(obj_pose=obj_pose_ROBOT, radius=0.15, stretch=2, res=0.001)
    
    """Init the tree from the poses we have and then save it"""
    # jconf_tree = JconfTree(robot=robot, poses=rings[0], transformations=transformations, n_confs_per_pose=20)
    # jconf_tree.add_poses(rings[1])
    # jconf_tree.save("test.json")
    """Once the tree has been stored in a file we can start it from there"""
    # jconf_tree = JconfTree(robot=robot, filename="test.json")

    # trj, cost = jconf_tree.gen_path([T.t for T in flat_rings])
    # trj_smooth = smooth_trj(trj, w_len=50, order=5)
    # df = pd.DataFrame(trj_smooth)
    # df.to_csv('trj.csv', index=False)

    trj = []
    with open('trj.csv', mode='r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)
        for row in csv_reader:
            trj.append(np.array([float(value) for value in row]))
    
    cost = find_trj_cost(trj)
    # plot_joint_traj(trj, title=str(cost), hold=True)
    # plot_joint_traj(trj_smooth, title=str(cost))
        
    """ROS SETUP"""
    rclpy.init(args=None)
    jpub = JointPublisher(topic_name='joint_states')
    ppub = PoseArrayPublisher(topic_name='poses')
    # ppub.send_poses([])
    # jpub.send_joint([0,0,0,0,0,0])    
    ppub.send_poses(robot.robot_T_world(T) for T in flat_rings)
    # jpub.send_traj(trj)
    
    # jpub.send_traj([trj[-1]])
    # robot.q = trj[-1]
    # gen_go_back_trj = gen_go_back_trj(robot=robot, poses=flat_rings, v_res=v_res, h_res=h_res)
    # input(">>>")
    # jpub.send_traj(gen_go_back_trj)
    # now_theta = np.linspace(0, np.pi*2, 250+1)[1:][-1] # the +1 is to get to the actual number
    # now_phi = np.linspace(starting_angle, ending_angle, v_res)[-1]


    # go_up_trj = robot.move_up(distance=0.2)

    
    # for z_step in np.arange()
    # robot.ik_collision_free()    
    
    # # df.to_csv('trj_smooth.csv', mode='a', index=False, header=not os.path.exists('trj_smooth.csv'))
    # df.to_csv('trj_smooth.csv', mode='w', index=False, header=True)
