import numpy as np 

def o3d_display_point(verts):
  import open3d

  pcd = open3d.geometry.PointCloud()

  pcd.points = open3d.utility.Vector3dVector(verts)
  mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  open3d.visualization.draw_geometries([pcd, mesh_frame])

def show_pointcloud_from_depth(depth, camera_angle_x=None, focal=None, T_=np.eye(4)):

    import open3d as o3d 
    import matplotlib.pyplot as plt

    H,W = depth.shape

    if camera_angle_x is not None:
        focal = .5 * W / np.tan(.5 * camera_angle_x)

    K = np.array([
        [focal, 0, 0.5*W],
        [0, focal, 0.5*H],
        [0, 0, 1]
    ])

    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]

    rows, cols = depth.shape
    yy,xx = np.meshgrid(np.arange(0, rows), np.arange(0, cols), indexing='ij')

    xx = xx.flatten()
    yy = yy.flatten()
    zz = depth.flatten()
    x = (xx - cx) * zz / fx
    y = (yy - cy) * zz / fy

    mask = zz > 0
    x = x[mask]
    y = y[mask]
    z = zz[mask]

    points = np.stack([x,y,z], axis=1)

    points = points @ T_[:3, :3].T + T_[:3, 3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    a = o3d.geometry.TriangleMesh.create_coordinate_frame(1)
    o3d.visualization.draw_geometries([pcd, a])