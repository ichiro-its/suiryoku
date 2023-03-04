from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    firstPackage = LaunchConfiguration('firstPackage')
    secondPackage = LaunchConfiguration('secondPackage')
    thirdPackage = LaunchConfiguration('thirdPackage')
    fourthPackage = LaunchConfiguration('fourthPackage')
    mode = LaunchConfiguration('mode')

    firstPackage_launch_arg = DeclareLaunchArgument(
        'firstPackage',
        default_value='suiryoku'
    )
    secondPackage_launch_arg = DeclareLaunchArgument(
        'secondPackage',
        default_value='aruku'
    )
    thirdPackage_launch_arg = DeclareLaunchArgument(
        'thirdPackage',
        default_value='tachimawari'
    )
    fourthPackage_launch_arg = DeclareLaunchArgument(
        'fourthPackage',
        default_value='kansei'
    )
    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        default_value=None,
        description='choose mode between sdk/cm740!',
        choices=['sdk', 'cm740']
    )

    firstNode = Node(
        package=firstPackage,
        executable='main',
        name='suiryoku',
        arguments=['src/suiryoku/data'],
        output='screen'
    )
    secondNode = Node(
        package=secondPackage,
        executable='main',
        name='aruku',
        arguments=['src/aruku/data'],
        output='screen'
    )
    thirdNode = Node(
        package=thirdPackage,
        executable='main',
        name='tachimawari',
        arguments=[mode],
        output='screen'
    )
    fourthNode = Node(
        package=fourthPackage,
        executable='main',
        name='kansei',
        arguments=['src/kansei/data'],
        output='screen'
    )

    return LaunchDescription([
        firstPackage_launch_arg,
        secondPackage_launch_arg,
        thirdPackage_launch_arg,
        fourthPackage_launch_arg,
        mode_launch_arg,
        firstNode,
        secondNode,
        thirdNode,
        fourthNode
    ])
