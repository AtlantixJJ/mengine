import os
import numpy as np
import open3d as o3d
import mengine as m

# NOTE: First install open3d using: 'python3 -m pip install open3d'
# On Mac you also need 'brew install libomp'

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create mustard bottle
mustard = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mustard.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[0, 0, 0.85], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)

# Let the object settle on the table
m.step_simulation(steps=100)

# Create a camera
camera = m.Camera(camera_pos=[0, -0.25, 1], look_at_pos=mustard.get_base_pos_orient()[0], fov=60, camera_width=1920//4, camera_height=1080//4)
# Capture a point cloud from the camera
pc, rgba = camera.get_point_cloud(body=mustard)
# Visualize the point cloud
m.DebugPoints(pc, points_rgb=rgba[:, :3], size=10)
# Hide the mustard bottle
mustard.change_visual(link=mustard.base, rgba=[1, 1, 1, 0])

# Create open3d point cloud from array of points
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pc)
pcd.colors = o3d.utility.Vector3dVector(rgba[:, :3])

# Estimate normals for each point
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
print('First normal:', pcd.normals[0])

# Visualize point cloud. Press 'n' to see normals
o3d.visualization.draw_geometries([pcd], front=[-0.28, -0.76, 0.59], lookat=[0, -0.01, 0.85], up=[0.18, 0.56, 0.81], zoom=0.7)

# Visualize voxelization
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.005)
o3d.visualization.draw_geometries([voxel_grid], front=[-0.28, -0.76, 0.59], lookat=[0, -0.01, 0.85], up=[0.18, 0.56, 0.81], zoom=0.7)

# Visualize downsampled point cloud
downpcd = pcd.voxel_down_sample(voxel_size=0.005)
o3d.visualization.draw_geometries([downpcd], front=[-0.28, -0.76, 0.59], lookat=[0, -0.01, 0.85], up=[0.18, 0.56, 0.81], zoom=0.7)

m.step_simulation(steps=10000, realtime=True)
