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
                {'target_tf':'Ground_Truth'},
                {'gps_tf':'husky1_gnss_link'},      
            ],
            namespace = 'husky1',
            remappings={
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('/fix','navfix')
            }
        )
    ])