import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Declare arguments to be used from command line
    namespace_arg = DeclareLaunchArgument(
        'ns',
        default_value='/hellocm',
        description='Node namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use node with simulation time')

    cycletime_arg = DeclareLaunchArgument(
        'cycletime',
        default_value='10000',
        description='Node cycle time')

    rviz2_arg = DeclareLaunchArgument(
        'rviz2',
        default_value='False',
        description='Launch "rviz2" when launching the "hellocm" node')

    # Include Rviz2 node if required
    #rviz2_include = IncludeLaunchDescription(
     #   PythonLaunchDescriptionSource(
      #      os.path.join(
       #         get_package_share_directory('hellocm_cmnode'),
        #        'launch/ads-dv_rviz2.launch.py'
         #   )
        #),
        #condition=IfCondition(LaunchConfiguration("rviz2"))
    #)

    # Launch the hellocm node
    hellocm_node = Node(
        namespace=LaunchConfiguration('ns'),
        package="hellocm",
        executable='hellocm',
        name='hellocm',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'cycletime': LaunchConfiguration('cycletime')}
        ],
        remappings=[
          ([LaunchConfiguration('ns'), '/cm2ext'], '/carmaker/cm2ext'),
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        cycletime_arg,
        #rviz2_arg,
        hellocm_node,
        #rviz2_include
    ])
