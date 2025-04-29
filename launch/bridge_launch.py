from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('namespace', default_value='j100_0860',
                        description='namespace for the robot.'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_viz = get_package_share_directory('clearpath_viz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    # Paths
    gz_sim_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
    rviz_sim_launch = PathJoinSubstitution([pkg_clearpath_viz, 'launch', 'view_navigation.launch.py'])
    localization_launch = PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])

    # launch Descriptions
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[            
                        ('use_sim_time', LaunchConfiguration('use_sim_time')),
                        ('setup_path', LaunchConfiguration('setup_path')),
                        ('world', LaunchConfiguration('world')),
                        ('rviz', LaunchConfiguration('rviz')),
                        ('x', LaunchConfiguration('x')),
                        ('y', LaunchConfiguration('y')),
                        ('z', LaunchConfiguration('z')),
                        ('yaw', LaunchConfiguration('yaw'))]
    )
    
    # rviz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([rviz_sim_launch]),
    #     launch_arguments=[            
    #                     ('use_sim_time', LaunchConfiguration('use_sim_time')),
    #                     ('setup_path', LaunchConfiguration('setup_path')),
    #                     ('world', LaunchConfiguration('world')),
    #                     ('rviz', LaunchConfiguration('rviz')),
    #                     ('namespace', LaunchConfiguration('namespace'))]
    # )
    
    # localization = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([localization_launch]),
    #     launch_arguments=[            
    #                     ('use_sim_time', LaunchConfiguration('use_sim_time')),
    #                     ('setup_path', LaunchConfiguration('setup_path')),
    #                     ('world', LaunchConfiguration('world')),
    #                     ('rviz', LaunchConfiguration('rviz'))]
    # )
    
    # nav2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([nav2_launch]),
    #     launch_arguments=[            
    #                     ('use_sim_time', LaunchConfiguration('use_sim_time')),
    #                     ('setup_path', LaunchConfiguration('setup_path')),
    #                     ('world', LaunchConfiguration('world')),
    #                     ('rviz', LaunchConfiguration('rviz'))]
    # )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    
    # add actions to the launch description
    ld.add_action(gz_sim)
    # ld.add_action(rviz_sim)
    # ld.add_action(localization)
    # ld.add_action(nav2)
    return ld
