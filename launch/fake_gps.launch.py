from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fake_gps',
            executable='tf2gps_publisher',
            name='tf2gps_publisher',
            output='screen',
            emulate_tty=True,            
            parameters=[
                {'use_sim_time':True},
                {'base_tf':'map'},
                {'target_tf':'base_link_gt'},
                {'gps_tf':'jackal_gnss_link'},      
            ],
            namespace = 'jackal',
            remappings={
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('/fix','navfix')
            }
        )
    ])