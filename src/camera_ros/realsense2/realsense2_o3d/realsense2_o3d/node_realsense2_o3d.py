import sys, rclpy, logging, time, os, math
import paho.mqtt.client as mqtt
import open3d as o3d
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, PointField
from geometry_msgs.msg import Transform
from realsense2_o3d.realsense2cam_o3d import Realsense2_o3d_camera, Realsense2_camera, get_points_np, RealsenseCamera
from realsense2_o3d.regis_utils import *
from pc_registration.pcRegistration_utils import xyzrgb_numpyarray_to_pointcloud2, numpy_to_pointcloud2
from pc_registration.readpoints import read_points
from realsense2_o3d.cube_detection import *
from cv_bridge import CvBridge

def eula2mat(pose, is_meter=False, is_radian=True):
        '''
        :param pose: pose in extrinsic XYZRXRYRZ
        :param is_meter: if XYZ in meter or mm
        :return: rotation matrix of pose
        '''
        if not is_radian:
            angle1 = pose[3] * math.pi / 180
            angle2 = pose[4] * math.pi / 180
            angle3 = pose[5] * math.pi / 180
        else:
            angle1 = pose[3]
            angle2 = pose[4]
            angle3 = pose[5]
        if not is_meter:
            posex = pose[0] / 1000
            posey = pose[1] / 1000
            posez = pose[2] / 1000
        else:
            posex = pose[0]
            posey = pose[1]
            posez = pose[2]
        c1 = math.cos(angle1)
        s1 = math.sin(angle1)
        c2 = math.cos(angle2)
        s2 = math.sin(angle2)
        c3 = math.cos(angle3)
        s3 = math.sin(angle3)

        # ZYX
        T = np.array([[c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, posex],
                      [c2 * s3, s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, posey],
                      [-s2, s1 * c2, c1 * c2, posez],
                      [0, 0, 0, 1]])
        return T


class node_realsense2_o3d(Node):

    def __init__(self, logger=None):
        super().__init__('node_realsense2_o3d')
        # define logger
        self.logger = logger if logger else self.get_logger(__name__)
        self.logger.info("Logger initialized")

        # define subscription to /capture_signal
        self.sub_topic = '/capture_signal'  
        self.subscriber = self.create_subscription(String, self.sub_topic, self.trigger_action, 10)

        # define a publisher to publishing pointcloud and image
        self.pub_pc = self.create_publisher(PointCloud2, '/realsense/point_cloud', 10)
        self.logger.info("Topic created {}".format('/realsense/point_cloud'))
        self.pub_img = self.create_publisher(Image, '/realsense/color_image', 10)
        self.logger.info("Topic created {}".format('/realsense/color_image'))

        #创建publisher来发布消息自我触发，这个消息的发布是由外部的mqtt协议触发的。
        self.pub_trigger = self.create_publisher(String, '/capture_signal', 10)
        
        # 相机相关配置
        self.camera = RealsenseCamera(self.logger)
        self.camera.start_capture()
        
        # 读取手眼标定矩阵， todo：写入parser
        self.cali_matrix = np.load('/home/sjl/ros_ws/6dof_ws/src/camera_ros/realsense2/realsense2_o3d/realsense2_o3d/mat_cam_robot_poses/calib_matrix_rs0626.npy')
        self.robot_pose_mat = None
        self.robot_pose_ready = False
        
        # 点云处理及配准相关配置
        self.regis_template = None
        self.bbox_interest = None

        # 创建 MQTT 客户端实例
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.on_message = self.on_message
        # 设置 MQTT 代理服务器的地址和端口号
        broker_address = "192.168.2.150"
        port = 1883
        self.client.username_pw_set("test1", "1")

        # 连接到 MQTT 代理服务器
        self.client.connect(broker_address, port)
        
        # 启动 MQTT 客户端的消息循环
        self.client.loop_start()
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.logger.info("Connected to MQTT broker")   
            self.client.subscribe("/jaka_pose")   
            self.client.subscribe("/CRtrigger")      
        else:
            self.logger.info("Failed to connect, return code: %s ", rc)

    def on_publish(self, client, userdata, mid):
         # 获取发布消息的返回信息
        self.logger.info("Message published, mid: %s", mid)



    # 定义回调函数，处理接收到的pose
    def on_message(self, client, userdata, msg):
        if msg.topic == "/jaka_pose":
            self.logger.info("Received /jaka_pose message")
            pose = np.frombuffer(msg.payload, dtype=np.float32)
            self.robot_pose_mat = eula2mat(pose, is_meter=False, is_radian=False)
            self.robot_pose_ready = True
        if msg.topic =='/CRtrigger':
            self.logger.info("Received /CRtrigger message, payload %s" % msg.payload)
            if msg.payload == b'boxes':
                self.logger.info("Received /boxes_trigger message")
                ros_msg = String()
                ros_msg.data = 'cube'
                self.pub_trigger.publish(ros_msg)
                self.logger.info('Published: %s' % ros_msg.data)
            if msg.payload == b'corridor':
                self.logger.info("Received /corridor_trigger message")
                ros_msg = String()
                ros_msg.data = 'pass'
                self.pub_trigger.publish(ros_msg)
                self.logger.info('Published: %s' % ros_msg.data)



    def publish_data(self):
        # prepare msg data
        try:

            points, image = self.camera.capture()

            # todo: 写进argparser
            # capture_pose = np.load('/home/sjl/ros_ws/6dof_ws/src/camera_ros/realsense2/realsense2_o3d/realsense2_o3d/mat_cam_robot_poses/flange_pose.npy')
            # capture_pose = np.load('/home/sjl/ros_ws/6dof_ws/src/camera_ros/realsense2/realsense2_o3d/realsense2_o3d/mat_cam_robot_poses/flange_pose_low4.npy')
            
            # transform points to robot_base frame
            self.get_jaka_pose()
            capture_pose = self.robot_pose_mat
            
            points = self.cam_2_rbbotbase(points, self.cali_matrix, capture_pose)
            
            # save image and transformed points 
            np.save('/home/sjl/ros_ws/6dof_ws/cr_data_out/points.npy', points)
            np.save("/home/sjl/ros_ws/6dof_ws/cr_data_out/image.npy", image)
            

            pc_msg, image_msg = self.gen_msg(points, image)     

            # 发布消息
            self.pub_pc.publish(pc_msg)
            self.pub_img.publish(image_msg)
            self.logger.info('realsense node msg published.')

        except StopIteration:
            self.camera.stop_capture()
            self.logger.warn("No more frames to capture.")


    def multiview_collect(self, pose_count, topic):
        '''
            manually collect multiview points and images,
            return: combined points, numpy array (N, 3)
        '''
        
        try:
            points_fusion = []
            count = 0
            while True and count < pose_count:
                self.robot_pose_ready = False
                self.get_jaka_pose(topic=topic) # trigger jaka move and return its pose 
                while not self.robot_pose_ready:
                    # wait until robot_pose_received.
                    pass
                
                points, image = self.camera.capture() # get aligned points and image
                capture_pose = self.robot_pose_mat
                points = self.cam_2_rbbotbase(points, self.cali_matrix, capture_pose)
                
                points_fusion.append(points)
                
                count += 1
                '''
                    如果逐步调试请uncomment此段，并去掉上面的count+=1
                
                # key = input('press n for next, q for quit')
                # if key == 'n':
                #     count += 1
                #     continue
                # elif key == 'q':
                #     break
                '''

            points_fusion = np.asarray(points_fusion).reshape(-1,3)
            fusion_cloud = o3d.geometry.PointCloud()
            fusion_cloud.points = o3d.utility.Vector3dVector(points_fusion)
            o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/fusion_cloud.ply', fusion_cloud)
            # np.save('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/points_fused.npy', points_fusion)
        except Exception as e:
            self.logger.error("Exception:", e)
        return fusion_cloud


    def registration(self, fusion_cloud):
        '''
            在这里对fusion point clouds里面的template物体进行配准
        '''
        # 0. Clone template
        template = o3d.geometry.PointCloud(self.regis_template)
        
        # 1. process points_fusion
        # 1.1 创建Open3D点云对象
        # fusion_cloud = o3d.geometry.PointCloud()
        # fusion_cloud.points = o3d.utility.Vector3dVector(points_fusion)
        
        # 1.2 根据已知边框进行裁剪
        # xmin, ymin, zmin = 0.473, -0.410, -0.190
        # xmax, ymax, zmax = 0.837, 0.132, 0.055
        # bbox_min = np.array([xmin, ymin, zmin])  # 边界框最小点坐标
        # bbox_max = np.array([xmax, ymax, zmax])  # 边界框最大点坐标
        # bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=bbox_min, max_bound=bbox_max) # 创建边界框对象
        fusion_cloud_cropped = fusion_cloud.crop(self.bbox_interest) # 在边界框内保留点云

        # 1.3 预处理裁剪后的点云
        fusion_cloud_ds = fusion_cloud_cropped.voxel_down_sample(voxel_size=0.005) # 降采样
        
        self.logger.info("原始点云大小：%d" % len(fusion_cloud_cropped.points)) # 输出采样后的点云信息
        self.logger.info("下采样后的点云大小：%d" % len(fusion_cloud_ds.points))
        if len(fusion_cloud_ds.points) < 800:
            self.logger.info("点云信息太少，无法识别到物体。")
            return 
        
        fusion_cloud_filtered, _ = fusion_cloud_ds.remove_statistical_outlier(nb_neighbors=150, std_ratio=4) # 统计离群值移除
        fusion_cloud_filtered, _ = fusion_cloud_filtered.remove_statistical_outlier(nb_neighbors=20, std_ratio=1)

        fusion_cloud_filtered.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=20)) # 法线计算
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/fusion_cloud_cropped.ply', fusion_cloud_cropped) 
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/fusion_cloud_ds.ply', fusion_cloud_ds)
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/processed.ply', fusion_cloud_filtered)
        # 2. do registration, return one pose.
        # if there exist over 800 points higher than horizontal level (zmin + 0.12), then repeat registraton until position is above the level.
        
        high_zmin = zmin + 0.12
        
        bbox_min = np.array([xmin, ymin, high_zmin])  # 边界框最小点坐标
        bbox_max = np.array([xmax, ymax, zmax])  # 边界框最大点坐标
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=bbox_min, max_bound=bbox_max) # 创建边界框对象
        high_filtered = fusion_cloud_filtered.crop(bbox) # 在边界框内保留点云
        high_point_num = len(high_filtered.points)
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/high_filtered.ply', high_filtered)
        self.logger.info('Points number higher than level is % s' % str(high_point_num))
        t_z = high_zmin - 1
        if high_point_num > 800:
            high_filtered = high_filtered.voxel_down_sample(voxel_size=0.005)
            self.logger.info('there exist stack objects')
            while t_z < high_zmin:
                pose, _ =  regis_once_main(template, high_filtered, adjust_pose=True)
                t_z = pose[2,3]
                self.logger.info('object center is at level %s' % str(t_z) )
        else:
            fusion_cloud_filtered = fusion_cloud_filtered.voxel_down_sample(voxel_size=0.005)
            pose, _ =  regis_once_main(template, fusion_cloud_filtered, adjust_pose=True)


        # 3. apply transformation to control flange. 300mm from cube top.
        # matrix = [[ 7.07106781e-01, -7.07106781e-01 , 0.00000000e+00,  3.53553391e-03],
        # [-7.07106781e-01, -7.07106781e-01, -1.22464680e-16, -3.53553391e-03],
        # [ 8.65956056e-17,  8.65956056e-17, -1.00000000e+00,  3.00000000e-01],
        # [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]] 
        # flange_pose = np.dot(pose, matrix)
        self.logger.info('the registered pose is %s' , pose)
        # self.logger.info('the Gripping flange pose is %s' , flange_pose)
        
        return pose


    def registration_pass(self, fusion_cloud):
        '''
            在这里对fusion point clouds里面的template物体进行配准
        '''
        # 0. Clone template
        template = o3d.geometry.PointCloud(self.regis_template)
        
        # 1. process points_fusion
        # 1.1 创建Open3D点云对象
        # fusion_cloud = o3d.geometry.PointCloud()
        # fusion_cloud.points = o3d.utility.Vector3dVector(points_fusion)
        
        # 1.2 根据已知边框进行裁剪
        # xmin, ymin, zmin = 0.473, -0.410, -0.190
        # xmax, ymax, zmax = 0.837, 0.132, 0.055
        # bbox_min = np.array([xmin, ymin, zmin])  # 边界框最小点坐标
        # bbox_max = np.array([xmax, ymax, zmax])  # 边界框最大点坐标
        # bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=bbox_min, max_bound=bbox_max) # 创建边界框对象
        fusion_cloud_cropped = fusion_cloud.crop(self.bbox_interest) # 在边界框内保留点云

        # 1.3 预处理裁剪后的点云
        fusion_cloud_ds = fusion_cloud_cropped.voxel_down_sample(voxel_size=0.005) # 降采样
        
        self.logger.info("原始点云大小：%d" % len(fusion_cloud_cropped.points)) # 输出采样后的点云信息
        self.logger.info("下采样后的点云大小：%d" % len(fusion_cloud_ds.points))
        if len(fusion_cloud_ds.points) < 800:
            self.logger.info("点云信息太少，无法识别到物体。")
            return 
        
        fusion_cloud_filtered, _ = fusion_cloud_ds.remove_statistical_outlier(nb_neighbors=150, std_ratio=4) # 统计离群值移除
        fusion_cloud_filtered, _ = fusion_cloud_filtered.remove_statistical_outlier(nb_neighbors=20, std_ratio=1)

        fusion_cloud_filtered.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=20)) # 法线计算
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/fusion_cloud_cropped.ply', fusion_cloud_cropped) 
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/fusion_cloud_ds.ply', fusion_cloud_ds)
        o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/processed.ply', fusion_cloud_filtered)
        # 2. do registrations, return two poses.
        pose_list = []
        fusion_cloud_filtered = fusion_cloud_filtered.voxel_down_sample(voxel_size=0.005)
        for i in range(2):
            pose, fusion_cloud_filtered =  regis_once_main(template, fusion_cloud_filtered, adjust_pose=False)
            remaining = fusion_cloud_filtered
            pose_list.append(pose)
            self.logger.info('the registered pose is %s' , pose)
            o3d.io.write_point_cloud('/home/sjl/ros_ws/6dof_ws/cr_data_out/multiview/remaining%d.ply' % i, fusion_cloud_filtered) 

        # 3. apply transformation to control flange. 300mm from cube top.
        # matrix = [[ 7.07106781e-01, -7.07106781e-01 , 0.00000000e+00,  3.53553391e-03],
        # [-7.07106781e-01, -7.07106781e-01, -1.22464680e-16, -3.53553391e-03],
        # [ 8.65956056e-17,  8.65956056e-17, -1.00000000e+00,  3.00000000e-01],
        # [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]] 
        # flange_pose = np.dot(pose, matrix)
        # self.logger.info('the Gripping flange pose is %s' , flange_pose)
        
        return pose_list, remaining


    def mqtt_send(self, pose, topic = ""):
        # 4. 生成mqtt msg 并以/boxes的topic传出去
        mqtt_msg = np.asarray(pose, dtype=np.float32)
        mqtt_msg = bytearray(mqtt_msg.tobytes())
        self.client.publish(topic, mqtt_msg)
        self.logger.info('Target pose send out, continued')

    def get_jaka_pose(self, topic=''):
        '''
            每次返回一个pose
        '''
        try:
            assert topic != ''
            if topic == '/trigger_topic/pass':
                trigger_topic = '/trigger_topic/pass'
                self.client.publish(trigger_topic, payload='trigger')
            elif topic == '/trigger_topic/cube':
                trigger_topic = '/trigger_topic/cube'
                self.client.publish(trigger_topic, payload='trigger')

        except Exception as e:
            self.logger.error("Exception:", e)


    def gen_msg(self, points, image):
        '''
            generate mqtt msg for points and image
        '''
        # prepare ros2 msgs, in PC2 and Image
        pc_msg = numpy_to_pointcloud2(points, None, None) # nparray to ros2 PointCloud2
        image_msg = self.cvimg_to_ros2img(image)
        return pc_msg, image_msg

    def collect_calibration_data(self, num=15):
        '''
            for calibration data collection, num of [img, points_map]default 15
        '''
        
        # 此method可以用来进行标定图像和点云的采集
        
        for i in range(num):
            points, image = self.camera.capture()
            np.save('/home/sjl/ros_ws/6dof_ws/cr_data_out/calibration_data_collection/points%d.npy' % i, points)
            np.save("/home/sjl/ros_ws/6dof_ws/cr_data_out/calibration_data_collection/image%d.npy" % i, image)
            input('%d finished----press any buttom' % i)
            continue
         

    def convert_to_pointcloud2(self, rgbd_frame):
        # 将 RGBD 数据转换为点云
        o3d_cloud = o3d.t.geometry.PointCloud.create_from_rgbd_image(rgbd_frame, self.camera.intrinsic).to_legacy()
        o3d_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        # 获取点云数据
        points = o3d_cloud.points
        colors = o3d_cloud.colors

        # 构造 PointCloud2 消息
        msg = PointCloud2()
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True

        # 填充点云数据
        buffer = []
        for i in range(len(points)):
            buffer.append(points[i][0])
            buffer.append(points[i][1])
            buffer.append(points[i][2])
            buffer.append(colors[i][0])
            buffer.append(colors[i][1])
            buffer.append(colors[i][2])

        buffer = np.asarray(buffer, dtype=np.uint8).tolist()
        msg.data = bytes(buffer)
        msg.data = bytes(buffer)

        return msg

    def trigger_action(self, msg):
        '''
            this is a callback trigger, switch to different methods for different purposes.
        '''
        self.logger.info("Received trigger message: %s" % msg.data)
        # self.publish_data()
        '''
            cube识别
        '''
        if msg.data == 'cube':
            self.regis_template = o3d.io.read_point_cloud("/home/sjl/ros_ws/6dof_ws/cr_data_out/template/cube_translated_voxled.ply")
            fusion_cloud = self.multiview_collect(6, '/trigger_topic/cube')
            global xmin, ymin, zmin, xmax, ymax, zmax
            xmin, ymin, zmin = 0.473, -0.410, -0.190
            xmax, ymax, zmax = 0.837, 0.132, 0.055
            bbox_min = np.array([xmin, ymin, zmin])  # 边界框最小点坐标
            bbox_max = np.array([xmax, ymax, zmax])  # 边界框最大点坐标
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=bbox_min, max_bound=bbox_max) # 创建边界框对象
            self.bbox_interest = bbox
            
            # template is self.regis_template
            pose = self.registration(fusion_cloud)
            self.mqtt_send(pose, topic="/boxes")

        '''
            通过性认证
        '''
        if msg.data == 'pass':
            self.regis_template = o3d.io.read_point_cloud("/home/sjl/ros_ws/6dof_ws/cr_data_out/template/plane.ply")
            fusion_cloud = self.multiview_collect(6, '/trigger_topic/pass')
            
            xmin, ymin, zmin = -0.30, -1.050, -0.07
            xmax, ymax, zmax = 0.25, -0.62, 0.05
            bbox_min = np.array([xmin, ymin, zmin])  # 边界框最小点坐标
            bbox_max = np.array([xmax, ymax, zmax])  # 边界框最大点坐标
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=bbox_min, max_bound=bbox_max) # 创建边界框对象
            self.bbox_interest = bbox

            # template is self.regis_template
            poses, remaining = self.registration_pass(fusion_cloud)
            self.mqtt_send(poses, topic="/corridor")
        '''
            calibration
        '''
        # self.collect_calibration_data()

    def run(self):
        # Publish data once per second                                                                                                                                                                                                  
        # timer_period = 0.033 hardware limitation is 30Hz
        # timer_period = 0.5
        # self.timer_ = self.create_timer(timer_period, self.publish_data)
        while rclpy.ok():
            rclpy.spin_once(self)
        self.camera.stop_capture()

    def cam_2_rbbotbase(self, points, cali_matrix, capture_pose):
        '''
            转齐次坐标，读取标定矩阵和当前拍摄位置，并应用变换矩阵 r_T_g(capture_pose) * g_T_c(cali_matrix) * P_c(homogeneous_coords)
            return: numpy array[n, 3]
        '''
        ## 进行手眼标定的变换，将rs坐标系的点位转为机械臂坐标点位

        # 添加齐次坐标
        n = points.shape[0]
        homogeneous_coords = np.hstack((points, np.ones((n, 1))))
        
        # 应用变换矩阵 r_T_g * g_T_c(cali_matrix) * P_c(homogeneous_coords)
        r_T_g = capture_pose
        transformed_coords = np.dot(r_T_g, np.dot(cali_matrix, homogeneous_coords.T)).T
        
        # 去除齐次坐标
        points = transformed_coords[:, :3]
        return points

    def cvimg_to_ros2img(self, image):
        # 将 OpenCV 图像转换为 ROS 2 的 Image 消息
        cv_bridge = CvBridge()
        image_cv = np.asarray(image)
        image_msg = cv_bridge.cv2_to_imgmsg(image_cv)
        image_msg.header.stamp = rclpy.time.Time(seconds=int(time.time()), nanoseconds=0).to_msg()
        return image_msg


def main(args=None):
    
    # define logger
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # std out logger handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    # file_handler = logging.FileHandler('{}.log'.format(__name__))
    # file_handler.setFormatter(formatter)
    # file_handler.setLevel(logging.DEBUG)
    # logger.addHandler(file_handler)
    
    
    logger.info('node_realsense2_o3d is starting')

    rclpy.init(args=args)

    while True:
        node = node_realsense2_o3d(logger)

        node.run()

        # 断开mqtt连接
        node.client.disconnect()

        # 停止 MQTT 客户端的消息循环
        node.client.loop_stop()
        
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()



if __name__ == '__main__':
    main()
