#!/usr/bin/env python3
import os
import json
import open3d as o3d
import numpy as np
import re
from os.path import join
# print(o3d.ColorMapOptimizationOption)
def sorted_alphanum(file_list_ordered):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(file_list_ordered, key=alphanum_key)


def get_file_list(path, extension=None):
    if extension is None:
        file_list = [
            path + f for f in os.listdir(path) if os.path.isfile(join(path, f))
        ]
    else:
        file_list = [
            path + f
            for f in os.listdir(path)
            if os.path.isfile(os.path.join(path, f)) and
            os.path.splitext(f)[1] == extension
        ]
    file_list = sorted_alphanum(file_list)
    return file_list

def my_data_loader():
    # path = o3dtut.download_fountain_dataset()
    path = "/home/nitesh/Downloads/rgbd-scenes/desk/desk_3/"
    debug_mode = True

    rgbd_images = []
    # depth_image_path = get_file_list(os.path.join(path, "depth/"), extension=".png")
    # color_image_path = get_file_list(os.path.join(path, "image/"), extension=".jpg")
    all_images = get_file_list(path, extension=".png")
    depth_image_path = []
    color_image_path = []
    for image in all_images:
        if "depth" in image:
            depth_image_path.append(image)
        else:
            color_image_path.append(image)
    # color_image_path = get_file_list(path, extension=".png")
    # print((depth_image_path))
    assert (len(depth_image_path) == len(color_image_path))
    for i in range(len(depth_image_path)):
        depth = o3d.io.read_image(os.path.join(depth_image_path[i]))
        color = o3d.io.read_image(os.path.join(color_image_path[i]))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False)
        if debug_mode:
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image,
                o3d.camera.PinholeCameraIntrinsic(
                    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        rgbd_images.append(rgbd_image)
    # o3d.visualization.draw_geometries([pcd])
    return rgbd_images

is_ci = False

def load_fountain_dataset():
    rgbd_images = []
    fountain_rgbd_dataset = o3d.data.SampleFountainRGBDImages()
    for i in range(len(fountain_rgbd_dataset.depth_paths)):
        depth = o3d.io.read_image(fountain_rgbd_dataset.depth_paths[i])
        color = o3d.io.read_image(fountain_rgbd_dataset.color_paths[i])
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False)
        rgbd_images.append(rgbd_image)

    camera_trajectory = o3d.io.read_pinhole_camera_trajectory(
        fountain_rgbd_dataset.keyframe_poses_log_path)
    mesh = o3d.io.read_triangle_mesh(
        fountain_rgbd_dataset.reconstruction_path)

    return mesh, rgbd_images, camera_trajectory


def run():
    rgbd_images = my_data_loader()
    camera_trajectory = o3d.io.read_pinhole_camera_trajectory("traj.log")
    mesh = o3d.io.read_triangle_mesh("possion_mesh.ply")
        
    mesh, camera_trajectory = o3d.pipelines.color_map.run_rigid_optimizer(
    mesh, rgbd_images, camera_trajectory,
    o3d.pipelines.color_map.RigidOptimizerOption(maximum_iteration=0))

    o3d.visualization.draw_geometries([mesh])

    print("rigid optimization")
    maximum_iteration = 100 if is_ci else 300
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, camera_trajectory = o3d.pipelines.color_map.run_rigid_optimizer(
            mesh, rgbd_images, camera_trajectory,
            o3d.pipelines.color_map.RigidOptimizerOption(
                maximum_iteration=maximum_iteration))
    o3d.visualization.draw_geometries([mesh])

    # Save mesh
    mesh_path = "possion_mesh_rigid" + '.ply'
    o3d.io.write_triangle_mesh(mesh_path, mesh)

    print("non-rigid optimization")
    maximum_iteration = 100 if is_ci else 300
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, camera_trajectory = o3d.pipelines.color_map.run_non_rigid_optimizer(
            mesh, rgbd_images, camera_trajectory,
            o3d.pipelines.color_map.NonRigidOptimizerOption(
                maximum_iteration=maximum_iteration))
        
    # Save mesh
    mesh_path = "possion_mesh_nonrigid" + '.ply'
    o3d.io.write_triangle_mesh(mesh_path, mesh)
        
    o3d.visualization.draw_geometries([mesh])

def quaternion_to_rotation_matrix(q):
    """Convert quaternion to rotation matrix.
    Args:
        q: (4,) array, quaternion.
    Returns:
        R: (3, 3) array, rotation matrix.
    """
    q = np.array(q)
    assert q.shape == (4,), "q must be (4,)"
    q /= np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([[1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w,
                   2 * x * z + 2 * y * w],
                  [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2,
                   2 * y * z - 2 * x * w],
                  [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w,
                   1 - 2 * x**2 - 2 * y**2]])
    return R

def generate_camera_trajectory():
    path = "/home/nitesh/programming/take_home/reconstruction/data/rgbd-scenes_aligned/desk/desk_3/desk_3.pose"
    f = open(path, "r")
    traj = open("traj.log", "w")
    lines = f.readlines()
    extrinsics = []
    for line in lines:
        line = line.split(" ")
        line = [float(i) for i in line[2:]]
        trans = np.eye(4)
        trans[0, 3] = line[4]
        trans[1, 3] = line[5]
        trans[2, 3] = line[6]
        rot = quaternion_to_rotation_matrix(line[:4])
        rot = np.vstack((rot, np.array([0, 0, 0])))
        rot = np.hstack((rot, np.array([[line[4]], [line[5]], [line[6]], [1]])))

        traj.write(f"0 0 0\n"
                   f"{rot[0, 0]} {rot[0, 1]} {rot[0, 2]} {rot[0, 3]}\n"
                   f"{rot[1, 0]} {rot[1, 1]} {rot[1, 2]} {rot[1, 3]}\n"
                   f"{rot[2, 0]} {rot[2, 1]} {rot[2, 2]} {rot[2, 3]}\n"
                   f"{rot[3, 0]} {rot[3, 1]} {rot[3, 2]} {rot[3, 3]}\n"
                   )
    my_data = {"extrinsics": extrinsics}
    jsonFile = open("data.json", "w")
    jsonFile.write(json.dumps(my_data))

# my_data_loader()
# generate_camera_trajectory()
run()
# mesh = o3d.io.read_triangle_mesh("possion_mesh.ply")
# o3d.visualization.draw_geometries([mesh])