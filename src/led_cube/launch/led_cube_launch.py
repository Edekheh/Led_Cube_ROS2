from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackagePrefix(package='led_cube').find('led_cube')
    pkg_share = pkg_share.replace('/install', '/src')
    rviz_arg = PathJoinSubstitution([pkg_share, 'include', 'rviz.rviz'])
    return LaunchDescription([
        Node(
            package='led_cube',
            namespace='led_cube',
            executable='led_cube',
            name='led_cube'
        ),
        Node(
            package='led_cube',
            namespace='led_cube',
            executable='colors_pub',
            name='colors_pub'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_arg]
        )
    ])