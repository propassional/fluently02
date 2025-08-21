from roboticstoolbox.robot.ERobot import ERobot
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import Trajectory
import numpy as np
import os
import copy
import spatialmath as sm
import matplotlib.pyplot as plt
from numpy import ndarray
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from roboticstoolbox.backends import PyPlot
import time
from collections import deque
from scipy.interpolate import CubicSpline

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

class _RRT():
    class Node:
        def __init__(self, q, nearest_q_idx):
            self.q = q 
            self.nearest_q_idx = nearest_q_idx

    def __init__(self, robot_backend, q0, qf, step_size=0.1, n_trials=1000):
        self.tree = [_RRT.Node(np.array(q0), -1)]
        self.qf = np.array(qf)
        self.step_size = step_size
        self.n_trials = n_trials
        self.robot = robot_backend

    def plan(self):
        self.traj = None
        self.converged = False
        trial = 0
        while trial < self.n_trials:
            q_rand = None
            if np.random.rand() < 0.1:
                q_rand = self.qf
            else:
                q_rand = np.random.uniform(-2*np.pi, 2*np.pi, 6)
        
            distances = []
            for node in self.tree:
                distances.append(np.linalg.norm(node.q - q_rand))
            nearest_idx = np.argmin(distances)
            q_nearest = self.tree[nearest_idx].q
            
            # steps = int(np.linalg.norm(q_rand-q_nearest)/self.step_size+1)
            if np.linalg.norm(q_rand - q_nearest) == 0:
                continue
            direction = (q_rand - q_nearest) / np.linalg.norm(q_rand - q_nearest)
            q_new = q_nearest + self.step_size * direction
            # q_new = np.linspace(q_nearest, q_rand, steps)[1]

            if not self.robot.check_joint_collisions(q_new):
                self.tree.append(_RRT.Node(q_new, nearest_idx))
                if np.linalg.norm(q_new-self.qf) < self.step_size:
                    self.tree.append(_RRT.Node(self.qf, len(self.tree)-1))
                    self.traj = self._generate_traj()
                    self.converged = True
            trial += 1
        return self.converged

    def _generate_traj(self):
        traj = []
        node_idx = len(self.tree) - 1
        while node_idx != -1:
            traj.append(self.tree[node_idx].q)
            node_idx = self.tree[node_idx].nearest_q_idx
        return np.array(traj[::-1])
    
    def spline_smooth_traj(self, num_points=200):
        t = np.arange(len(self.traj))  # Time steps, assuming uniform time between waypoints

        # Create cubic splines for each joint (assuming 6DOF robot)
        splines = [CubicSpline(t, self.traj[:, i]) for i in range(6)]  # One spline per joint

        # Generate more points along the trajectory using spline interpolation
        t_smooth = np.linspace(0, len(self.traj) - 1, num=num_points)
        self.traj = np.array([spline(t_smooth) for spline in splines]).T

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
        self.qlim[0][joint_index] = limit[0]
        self.qlim[1][joint_index] = limit[1]

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

    def ik_collision_free(self, Tep: sm.SE3, n_trials: int = 50000, q0: ndarray = None) -> tuple[bool, ndarray]:
        """Generate a inverse kinematics collision fre solution for the pose Tep

        Args:
            Tep (sm.SE3): target Tep in base frame
            n_trials (int, optional): # of trials to find the solution. Defaults to 10.

        Returns:
            bool, ndarray: whether or not the solution is valid and the solution
        """
        start = time.time()
        perturbation = 0.2 * (np.pi - (-np.pi))
        sol_valid = False
        trial = 0
        T = Tep * self.tcp_frame_transf.inv()
        q0 = self.q if q0 is None else q0
        starting_q = q0
        if not self.check_pose_collisions(T): # if the point itself is in collision then we don't even try
            while not sol_valid and trial < n_trials: # otherwise we try n_trials time
                sol = self.ik_LM(T, q0=starting_q, joint_limits=self.use_j_limit) # generating the solution
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
            sol = np.full(6, np.nan)
        # print(f"trial: {trial}, success: {sol_valid}, time: {time.time()-start}", end=' ')
        return sol_valid, sol

    def generate_traj_rrt(self, q0, qf, step_size=0.05, n_trials=1000, plan_tries=10):
        tries = 0
        while tries < plan_tries: 
            print(f"rrt tries: {tries+1}/{plan_tries}", end='\r')
            rrt = _RRT(robot_backend=self, q0=q0, qf=qf, step_size=step_size, n_trials=n_trials)
            if rrt.plan():
                break
            tries += 1
        rrt.spline_smooth_traj()
        # print()
        # for node in rrt.tree:
        #     print(node.q, node.nearest_q_idx)
        return rrt.traj
        
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
            if hold:
                self.env.hold()

def gen_rings_poses(obj_pose:sm.SE3, radius, h_res=8, v_res=5, stretch=1):
    u = np.linspace(0, np.pi*2, h_res+1)[1:] # the +1 is to get to the actual number
    v = np.linspace(np.pi/6, np.pi-np.pi/6, v_res)
    poses = deque()
    for phi in v:
        for theta in u: 
            x = (np.sin(phi) * np.cos(theta)) * radius * stretch # to "flatten" the ellipsoid increase this
            y = (np.sin(phi) * np.sin(theta)) * radius * stretch # to "flatten" the ellipsoid increase this
            z = (np.cos(phi)) * radius
            direction_vector = np.array([-x, -y, -z])
            direction_vector /= np.linalg.norm(direction_vector)
            z_angle = np.arctan2(direction_vector[1], direction_vector[0])
            y_angle = np.arctan2(np.sqrt(direction_vector[0]**2 + direction_vector[1]**2), direction_vector[2])
            Rot_mat = sm.SO3.Rz(z_angle) * sm.SO3.Ry(y_angle) * sm.SO3.Rx(0) * sm.SO3.Rz(np.pi)
            Rot_mat=Rot_mat*sm.SO3.Rz(np.pi/2)
            T = sm.SE3().Rt(Rot_mat, np.array([x, y, z]))
            T = obj_pose * T 
            poses.append(T)
        u = u[::-1]
    return poses

def gen_arc_poses(obj_pose:sm.SE3, radius, h_res=1, v_res=10, stretch=1):
    theta = 0
    v = np.linspace(np.pi/6, np.pi-np.pi/6, v_res+2)
    arc = []
    for phi in v:
        if abs(phi) == np.pi : # we don't want to hit the pole we remove pi
            continue
        x = ((np.sin(phi) * np.cos(theta)) * radius * stretch)
        y = ((np.sin(phi) * np.sin(theta)) * radius * stretch)
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

def gen_s_poses(obj_pose:sm.SE3, radius, num_points: int=25, stretch=1):
    ts = np.linspace(-2*np.pi, 2*np.pi, num_points)
    s = []
    i = 0
    for t in ts:
        z = 0.8*t/(2*np.pi) * radius
        y = np.sin(t)*0.5 * radius * stretch
        x = abs(np.sqrt(radius**2 - z**2 - y**2)) * stretch
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
