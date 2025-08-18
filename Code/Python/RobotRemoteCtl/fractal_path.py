import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import copy

class FractalPath:
    def __init__(self, csv_file) -> None:
        
        df = pd.read_csv(csv_file, delimiter=",")
        self.demo = df[['0', '1', '2', '3', '4', '5']].to_numpy()
        self.demo_ds = self.downsample_demo(round(len(self.demo)/10))
        
        fig = plt.figure(figsize=(10, 10))
        self.ax = fig.add_subplot(111, projection='3d')
        
        
        
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        
        x = self.demo[:,0]
        y = self.demo[:,1]
        z = self.demo[:,2]
            
        max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max()

        mid_x = (x.max() + x.min()) * 0.5
        mid_y = (y.max() + y.min()) * 0.5
        mid_z = (z.max() + z.min()) * 0.5

        self.ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        self.ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        self.ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
        
        
    def downsample_demo(self, target_length):
        current_length = self.demo.shape[0]
        target_indices = np.linspace(0, current_length - 1, target_length)
        idx=np.rint(target_indices).astype(int)
        return self.demo[idx]
    
    
    def get_rotated_axis(self,Rx, Ry, Rz, rotation_vectors):        
        rotation = Rotation.from_euler('xyz', [Rx, Ry, Rz], degrees=True)                    
        return rotation.apply(rotation_vectors)
    
    
    def scale(self,positions,scale):
        
        centroid = np.mean(positions, axis=0)

        translated_poses = positions - centroid

        scaled_poses = translated_poses * scale
        scaled_poses += centroid
        
        return scaled_poses
    
    
    def translate(self,positions,translation):
        return positions+translation
    
    
    def rotate_poses(self,rotations,delta):
        
        current_rotation = Rotation.from_euler('xyz', rotations, degrees=True)  
        rotation_vector =  Rotation.from_euler('xyz', np.radians(delta) ).as_rotvec()
        new_rotation = Rotation.from_rotvec(rotation_vector)

        return (current_rotation*new_rotation).as_euler('xyz', degrees=True)
    
    
    def rotate_path(self, path, global_rot):
        
        g_rot_vec = Rotation.from_euler('xyz', np.radians(global_rot) ).as_rotvec()
        global_rot = Rotation.from_rotvec(g_rot_vec)
        
        rot_point = np.mean(path[:,:3], axis=0)
        
        transformed_poses = []
        for pose in path:
            pos = pose[:3]
            rot = np.radians(pose[3:])

            rot_vec = Rotation.from_euler('xyz', rot).as_rotvec()

            new_p = global_rot.apply(pos - rot_point)
            new_pos = new_p + rot_point

            local_rot = Rotation.from_rotvec(rot_vec)
            
            transformed_rotation = global_rot * local_rot
            new_rot = np.degrees(transformed_rotation.as_euler('xyz'))

            transformed_pose = np.concatenate([new_pos, new_rot])
            transformed_poses.append(transformed_pose)

        return np.array(transformed_poses)
    
    
    def visualize_poses(self, path):

        unit_vec= np.array([[1, 0, 0],
                            [0, 1, 0], 
                            [0, 0, 1]])
        for p in path:
            x, y, z = p[0],p[1],p[2]
            Rx, Ry, Rz = p[3],p[4],p[5]
            
            rotated_axes = self.get_rotated_axis(Rx, Ry, Rz,unit_vec)

            self.ax.quiver(x, y, z, rotated_axes[0, 0], rotated_axes[0, 1], rotated_axes[0, 2], color="red"  , length=25)
            self.ax.quiver(x, y, z, rotated_axes[1, 0], rotated_axes[1, 1], rotated_axes[1, 2], color="green", length=25)
            self.ax.quiver(x, y, z, rotated_axes[2, 0], rotated_axes[2, 1], rotated_axes[2, 2], color="blue" , length=25)
            
            # ax.scatter(x, y, z, color="black",s=3)

        # plt.show()


if __name__=="__main__":
    # f = FractalPath('recorded_trajs/cart/fractal_circular.csv')
    f = FractalPath('recorded_trajs/cart/demo_scan.csv')
    traj_save_name="scan.csv"
    
    delta_scale = 0.9
    delta_translation = np.array([0,0,50.0])
    delta_global_rotation = np.array([0, 0, 0])
    
    delta_frame_rotation = np.array([0, -20, 0])
    
    new_pos = copy.deepcopy(f.demo_ds[:,:3])
    new_rot = copy.deepcopy(f.demo_ds[:,3:])
    
    fractal_path = np.hstack((new_pos,new_rot))
    
    f.visualize_poses(fractal_path)
    
    n_fractal_iter = 2
    
    for i in range(0,n_fractal_iter):
        
        new_pos = f.scale(new_pos,delta_scale)
        new_pos = f.translate(new_pos,delta_translation)
        new_rot = f.rotate_poses(new_rot,delta_frame_rotation)
        
        new_path = np.hstack((new_pos,new_rot))
        
        new_path = f.rotate_path(new_path, delta_global_rotation*i)
        
        f.visualize_poses(new_path)
        fractal_path = np.vstack([fractal_path, new_path])
    
    df = pd.DataFrame(fractal_path)
    df.to_csv(traj_save_name, index=None)
    
    plt.show()
