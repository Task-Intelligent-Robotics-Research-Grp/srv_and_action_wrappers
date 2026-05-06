from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions   import Node

launch_arguments = [
    {
        'name':        'policy',
        'default':     'single',
        'description': 'goal processing policy',
        'choices':     ['single', 'queued', 'multi'],
    },
    {
        'name':        'grouping',
        'default':     'false',
        'description': 'grouping goals by group name',
        'choices':     ['true', 'false', 'True', 'False'],
    },
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    return [
        Node(package='srv_and_action_wrappers',
             executable='test_action_server',
             parameters=[
                 {'policy':   LaunchConfiguration('policy'),
                  'grouping': LaunchConfiguration('grouping')}
             ],
             output='screen'),
        Node(package='srv_and_action_wrappers',
             executable='test_simple_action_client',
             prefix=['xterm -fn 7x14 -e'],
             output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
