from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration, 
    Command,
    TextSubstitution
)
from launch.actions import DeclareLaunchArgument
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # create launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')    
    base_tf = LaunchConfiguration('base_tf')
    target_tf = LaunchConfiguration('target_tf')
    gps_frame_name = LaunchConfiguration('gps_tf')


    # Setting default launch arguments 
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot'  # huksy1
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )

    

    base_tf_arg = DeclareLaunchArgument(
        'base_tf',
        default_value='map'
    )

    target_tf_arg = DeclareLaunchArgument(
        'target_tf',
        default_value='gps_link'  #Ground_Truth
    )

    gps_frame_name_arg = DeclareLaunchArgument(
        'gps_tf',
        default_value='gps_link'  #husky1_gnss_link
    )

    # config = launch_args['config_file']

    # config_path = os.path.join(
    #     get_package_share_directory('fake_gps'),
    #     'config',
    #     'fake_gps_sim.yaml'
    # )

    # node_parameters = [config_path]



    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,        
        base_tf_arg,
        target_tf_arg,
        gps_frame_name_arg,
        Node(
            package='fake_gps',
            executable='tf2gps_publisher',
            name='tf2gps_publisher',
            output='screen',
            emulate_tty=True,            
            # parameters = [config],
            parameters=[
                {'use_sim_time':use_sim_time},  
                {'base_tf':base_tf},
                {'target_tf':target_tf},
                {'gps_tf':gps_frame_name},      
            ],
            namespace = namespace,
            remappings={
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('/fix','navfix')
            }
        )
    ])