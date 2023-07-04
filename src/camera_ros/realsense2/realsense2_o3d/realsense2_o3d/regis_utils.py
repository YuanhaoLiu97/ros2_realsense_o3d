import open3d as o3d
import copy
import numpy as np
'''
Point clouds Global Registration RANSAC based on features with o3d.
Fine Registration ICP with o3d
'''

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])


def preprocess_point_cloud(pcd, voxel_size, uniform_rate):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    # pcd_down = pcd.voxel_down_sample(voxel_size)

    # print(":: Downsample with a uniform rate %.3f." %  uniform_rate)

    pcd_down = pcd.uniform_down_sample(every_k_points=int(1 / uniform_rate))
    radius_normal = voxel_size * 1
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 2
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, transformation, distance_threshold):
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result






def regis_wrap(template, fusion_cloud):
    voxel_size = 0.01

    # source_filtered.estimate_normals()
    template.estimate_normals()
    print('::normal estimated')

    # down sampling both clouds and extract fpfh features
    source_down, source_fpfh = preprocess_point_cloud(template, voxel_size, uniform_rate=1)
    target_down, target_fpfh = preprocess_point_cloud(fusion_cloud, voxel_size, uniform_rate=1)

    # global registration with RANSAC
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    # print(result_ransac.transformation)
    distance_threshold = voxel_size
    result_icp = refine_registration(template, fusion_cloud, source_fpfh, target_fpfh,
                                     voxel_size, result_ransac.transformation, distance_threshold)
    # print(result_icp.transformation)

    # 将仿射变换分解成平移向量和旋转矩阵
    translation = result_icp.transformation[:3, 3]
    rotation = result_icp.transformation[:3, :3]
    pose = result_icp.transformation

    # 复制点云对象
    clone = o3d.geometry.PointCloud(template)
    clone.transform(pose)
    # 计算源点云中每个点到目标点云的最近距离
    distances = np.asarray(fusion_cloud.compute_point_cloud_distance(clone))

    # 设置阈值，将最接近的点移除
    threshold = 0.02
    outlier_indices = distances < threshold
    outlier_indices_list = np.nonzero(outlier_indices)[0].tolist()
    removal_count = len(outlier_indices_list)
    fusion_cloud_new = fusion_cloud.select_by_index(outlier_indices_list, invert=True)

    return pose, removal_count, fusion_cloud_new


def adjust_object_pose(world_pose):
    # 提取物体的旋转矩阵R
    R = world_pose[:3, :3]

    # 提取世界坐标系的z轴方向
    world_z_axis = np.array([0, 0, 1])

    # 计算物体坐标系中各个轴与世界坐标系z轴的夹角
    angles = np.arccos(np.abs(np.dot(R.T, world_z_axis)))
    print(angles)
    # 找到夹角最小的轴的索引
    min_angle_idx = np.argmin(angles)

    # 根据夹角最小的轴重新定位物体的z轴
    old_x_axis = R[:, 0]
    old_y_axis = R[:, 1]
    old_z_axis = R[:, 2]
    if min_angle_idx == 0:
        # 物体x轴与世界坐标系z轴夹角最小
        new_z_axis = old_x_axis
        new_y_axis = old_z_axis
        new_x_axis = old_y_axis
    elif min_angle_idx == 1:
        # 物体y轴与世界坐标系z轴夹角最小
        new_z_axis = old_y_axis
        new_y_axis = old_x_axis
        new_x_axis = old_z_axis
    else:
        # 物体z轴与世界坐标系z轴夹角最小
        new_z_axis = old_z_axis
        new_y_axis = old_y_axis
        new_x_axis = old_x_axis

    if new_z_axis[2] >= 0:
        pass
    elif new_z_axis[2] < 0:
        temp_z = -new_z_axis
        temp_y = -new_x_axis
        temp_x = -new_y_axis
        new_z_axis = temp_z
        new_y_axis = temp_y
        new_x_axis = temp_x



    # 构造调整后的旋转矩阵
    new_R = np.column_stack((new_x_axis, new_y_axis, new_z_axis))

    # 构造调整后的物体姿态矩阵
    adjusted_pose = np.eye(4)
    adjusted_pose[:3, :3] = new_R
    adjusted_pose[:3, 3] = world_pose[:3, 3]

    return adjusted_pose








def regis_once_main(template, fusion_cloud, adjust_pose=False):
    removal_count_list = []
    pose_list = []
    remaining_list = []
    for i in range(3):
        pose, removal_count, remaining = regis_wrap(template, fusion_cloud)
        removal_count_list.append(removal_count)
        pose_list.append(pose)
        remaining_list.append(remaining)
    removal_count_list = np.asarray(removal_count_list)
    print(removal_count_list)
    print(pose_list)
    idx = np.argmax(removal_count_list)
    pose = pose_list[idx]
    remaining = remaining_list[idx]
    if adjust_pose:
        pose = adjust_object_pose(pose)
    return pose, remaining


