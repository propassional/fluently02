# Genera i csv

from robot_backend import SimRobotBackend
import spatialmath as sm # pip install spatialmath-python, since conda package is missing
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd # conda install pandasconda info --envs
import copy
import os

# Questo urdf non va bene, dovresti utilizzare un urdf che descriva solo il robot, te lo includo in questa cartella
urdf_path = os.getcwd() + "/crx20ia_l.urdf"

demo_path = './full_scan_bottom_joints.csv'
demo = pd.read_csv(demo_path)
b_seed = demo[['0', '1', '2', '3', '4', '5']].to_numpy()

demo_path = './full_scan_side_joints.csv'
demo = pd.read_csv(demo_path)
s_seed = demo[['0', '1', '2', '3', '4', '5']].to_numpy()

demo_path = './full_scan_top_joints.csv'
demo = pd.read_csv(demo_path)
t_seed = demo[['0', '1', '2', '3', '4', '5']].to_numpy()

plt.plot(np.vstack((b_seed,s_seed,t_seed)))
plt.legend(['j1', 'j2', 'j3','j4', 'j5', 'j6'])
plt.show()

obj_pose = sm.SE3([-0.04, 0.8, 1.28]) * sm.SE3.Rz(np.pi/3)

# la robot base é importante per l'uso della funzione world_T_robot che trasforma una posa specificata nel frame world nel frame robot, va specificat nel frame world e aggiunta in fase di creazione del robot backend
robot_base = sm.SE3.Rt(np.eye(3), [0, 0, 1])
robot = SimRobotBackend(urdf_file=urdf_path, tcp_frame_urdf="tool_tcp", robot_base=robot_base)

# questa variabile é la posa dell oggetto nel frame del robot, la useremo un paio di volte quindi l'ho definita
obj_pose_ROBOT = robot.world_T_robot(obj_pose)
# il free space del robot va specificato nel frame del robot ed é sul top del tavolo (0.92) ma appunto nel frame del robot, per questo la sottrazione
z_table = 0.92
robot.set_geom_free_space('z', (z_table-robot.robot_base.t[2], float('inf')))

# funzione di visualizzazione dell'ambiente del backend, purtroppo c'é u problema che disegna una linea tra il primo e l'ultimo joint a prescindere
robot.init_plot_env() 

# la collision sphere va aggiunta nel frame del robot, ho impostato il raggio per essere 0.15 per andare in coppia con il 0.2 delle pose, se aumenti il raggio delle pose potresti dover aumentare questo a sua volta
# se il plot env del robot é stato inizializzato verrá disegnata
robot.add_collision_sphere(obj_pose_ROBOT.t, radius=0.15)

# utilizzando direttamente la posa nel frame del robot ci evitiamo di dover trasformare poi le pose che ci escono fuori
rings = robot.gen_rings_poses(obj_pose=obj_pose_ROBOT, radius=0.2)
arc = robot.gen_arc_poses(obj_pose=obj_pose_ROBOT, radius=0.2) # Slice path
s = robot.gen_s_poses(obj_pose=obj_pose_ROBOT, radius=0.2)

c=0
seed_configuration=t_seed[0]
j_conf = []
for pose in rings:
    c+=1
    suc, q = robot.ik_collision_free((pose),q0=seed_configuration, n_trials=5000)
    print(c)
    print(q[0])
    print(seed_configuration)
    if suc:
        seed_configuration = copy.deepcopy(q[0])
        j_conf.append(q[0])
    # input()
    pose.plot(length=0.01)
robot.env.hold() # una volta inizializzato l'environment grafico utilizza quello anche per visualizzare le pose

stacked_arrays = np.vstack(j_conf)

plt.plot(stacked_arrays)
plt.legend(['j1', 'j2', 'j3','j4', 'j5', 'j6'])
plt.title("scan traj")
plt.show()

df = pd.DataFrame(stacked_arrays)
path= "trajs/scan_traj.csv"
df.to_csv(path, index=None)


seed_configuration=t_seed[0]
j_conf = []
for pose in arc:
    suc, q = robot.ik_collision_free(pose,q0=seed_configuration)
    if suc:
        seed_configuration = copy.deepcopy(q[0])
        j_conf.append(q[0])

stacked_arrays = np.vstack(j_conf)

plt.plot(stacked_arrays)
plt.legend(['j1', 'j2', 'j3','j4', 'j5', 'j6'])
plt.title("arc traj")
plt.show()

df = pd.DataFrame(stacked_arrays)
path= "trajs/arc_traj.csv"
df.to_csv(path, index=None)

j_conf = []
for pose in s:
    suc, q = robot.ik_collision_free(pose,q0=seed_configuration)
    if suc:
        seed_configuration = copy.deepcopy(q[0])
        j_conf.append(q[0])

stacked_arrays = np.vstack(j_conf)

plt.plot(stacked_arrays)
plt.legend(['j1', 'j2', 'j3','j4', 'j5', 'j6'])
plt.title("snake traj")
plt.show()

df = pd.DataFrame(stacked_arrays)
path= "trajs/s_traj.csv"
df.to_csv(path, index=None)