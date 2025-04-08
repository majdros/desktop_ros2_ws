from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Available Launch Arguments
    camera_index = DeclareLaunchArgument(name = 'camera_index', default_value = '0')
    frame_rate = DeclareLaunchArgument(name = 'frame_rate', default_value = '30.0')
    jpeg_quality_value = DeclareLaunchArgument(name = 'jpeg_quality_value', default_value = '70')
    use_raw_image_publisher = DeclareLaunchArgument(name = 'use_raw_image_publisher', default_value = 'false')
    use_compressed_image_publisher = DeclareLaunchArgument(name = 'use_compressed_image_publisher', default_value = 'true')

    # Launch Configurations
    camera_index_config = LaunchConfiguration('camera_index')
    frame_rate_config = LaunchConfiguration('frame_rate')
    jpeg_quality_value_config = LaunchConfiguration('jpeg_quality_value')
    use_raw_image_publisher_config = LaunchConfiguration('use_raw_image_publisher')
    use_compressed_image_publisher_config = LaunchConfiguration('use_compressed_image_publisher')


    # Camera Node
    camera_node = Node(
        package = 'my_robot_sensors',
        executable = 'camera_node',
        parameters = [{
            'camera_index' : camera_index_config,
            'frame_rate' : frame_rate_config,
            'jpeg_quality_value' : jpeg_quality_value_config,
            'use_raw_image_publisher' : use_raw_image_publisher_config,
            'use_compressed_image_publisher' : use_compressed_image_publisher_config,
                    }]
    )


    return LaunchDescription([
    camera_index,
    frame_rate,
    jpeg_quality_value,
    use_raw_image_publisher,
    use_compressed_image_publisher,
    camera_node,
    ])