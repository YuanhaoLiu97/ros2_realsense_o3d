import numpy as np
import open3d as o3d
from pc_registration.pcRegistration_utils import *
import pyrealsense2 as rs
import json, cv2

class Realsense2_o3d_camera():
    def __init__(self, logger):
        self.rscam = o3d.t.io.RealSenseSensor()
        self.rscam.start_capture()
        meta_data = self.rscam.get_metadata()
        self.intrinsic = o3d.core.Tensor(meta_data.intrinsics.intrinsic_matrix)
        self.logger = logger
        self.logger.info("::realsense2 cam initialized.")
        self.fid = 0

    def capture(self):
        self.logger.info("Frame: {}, time: {}s".format(self.fid, self.rscam.get_timestamp() * 1e-6))
        rgbd_frame = self.rscam.capture_frame()
        self.fid += 1
        return rgbd_frame

    def start_capture(self):
        self.fid = 0
        self.rscam.start_capture()
        self.logger.info("::realsense2 cam started.")

    def stop_capture(self):
        self.rscam.stop_capture()
        self.logger.info("::realsense2 cam stopped.")




class Realsense2_camera():
    def __init__(self, logger):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.logger = logger
        self.logger.info("::realsense2 cam initialized.")
        self.fid = 0

    def start_capture(self):
        self.fid = 0
        self.pipeline.start(self.config)
        self.logger.info("::realsense2 cam started.")

    def stop_capture(self):
        self.pipeline.stop()
        self.logger.info("::realsense2 cam stopped.")

    def capture(self):
        self.logger.info("Frame: {}".format(self.fid))
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        rgbd_frame = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_image),
            o3d.geometry.Image(depth_image)
        )
        self.fid += 1
        return rgbd_frame


def get_points_np():
    # 创建相机配置对象
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    # 启动相机
    pipeline = rs.pipeline()
    pipeline.start(config)

    try:
        # 循环读取图像
        while True:
            # 等待新的帧
            frames = pipeline.wait_for_frames()
            # 进行对齐处理
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)

            # 获取对齐后的深度帧和彩色帧
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            # # 获取深度图像和彩色图像
            # depth_frame = frames.get_depth_frame()
            # color_frame = frames.get_color_frame()
            # print(depth_frame.height, depth_frame.width)

            # 创建深度映射
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            
            # 创建点云对象
            point_cloud = rs.pointcloud()
            point_cloud.map_to(color_frame)  # 将点云映射到彩色帧
            
            points = point_cloud.calculate(depth_frame)
            point_cloud_data = np.asarray(points.get_vertices())  # 返回一个nparray，结构为n个numpy.void（[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')]） 
            point_cloud_np = np.stack((point_cloud_data['f0'], point_cloud_data['f1'], point_cloud_data['f2']), axis=-1) # 将其展开为nx3的nparray
            
            
            # points.export_to_ply("point_cloud.ply", depth_frame)
            if not depth_frame or not color_frame:
                continue

            # 在这里添加您的图像处理代码

            # # 将深度图像和彩色图像转换为OpenCV格式
            # depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            # depth_image_uint8 = cv2.convertScaleAbs(depth_image, alpha=0.03)
            # depth_image = cv2.applyColorMap(depth_image_uint8, cv2.COLORMAP_JET)
            # # 保存深度图像和彩色图像
            # cv2.imwrite("depth_image.png", depth_image)
            cv2.imwrite("color_image.png", color_image)
            break
    finally:
        # 停止相机
        pipeline.stop()
        return point_cloud_np, color_image



# 将前面的get_points_np写成类
class RealsenseCamera:
    def __init__(self, logger):
        self.pipeline = None
        self.logger = logger
        self.logger.info("Realsense2 cam initialized.")

    def start_capture(self):
        # 创建相机配置对象
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # 启动相机
        self.pipeline = rs.pipeline()
        self.pipeline.start(config)

    def capture(self):
        if self.pipeline is None:
            raise RuntimeError("Camera is not initialized. Call start_capture() first.")

        try:
            # 等待新的帧
            frames = self.pipeline.wait_for_frames()
            # 进行对齐处理
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)

            # 获取对齐后的深度帧和彩色帧
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # 创建深度映射
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # 创建点云对象
            point_cloud = rs.pointcloud()
            point_cloud.map_to(color_frame)  # 将点云映射到彩色帧

            points = point_cloud.calculate(depth_frame)
            point_cloud_data = np.asarray(points.get_vertices())  # 返回一个nparray，结构为n个numpy.void（[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')]） 
            point_cloud_np = np.stack((point_cloud_data['f0'], point_cloud_data['f1'], point_cloud_data['f2']), axis=-1) # 将其展开为nx3的nparray

            color_image = np.asanyarray(color_frame.get_data())
            
            return point_cloud_np, color_image

        except Exception as e:
            print("Error capturing frames:", e)

    def stop_capture(self):
        if self.pipeline is not None:
            # 停止相机
            self.pipeline.stop()