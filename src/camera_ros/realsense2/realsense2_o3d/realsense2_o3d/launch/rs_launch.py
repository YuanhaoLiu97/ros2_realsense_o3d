from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    ld_library_path = '/home/sjl/ros_ws/6dof_ws/src/camera_ros/realsense2/realsense2_o3d/realsense2_o3d'  # 第三方库的路径
    ld_library_action = SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path)

    node = Node(
        package='realsense2_o3d',
        executable='node_realsense2_o3d',
        # 其他节点配置参数
    )

    return LaunchDescription([
        ld_library_action,
        node,
    ])