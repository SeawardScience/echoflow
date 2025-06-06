# Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# Please consult your device's documentation for supported features
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, EmitEvent
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_driver'
_DEFAULT_PARAMS_FILE = os.path.join(
  get_package_share_directory(_PACKAGE_NAME),
  'microstrain_inertial_driver_common',
  'config',
  'params.yml'
)
_EMPTY_PARAMS_FILE = os.path.join(
  get_package_share_directory(_PACKAGE_NAME),
  'config',
  'empty.yml'
)
_TREVOR_PARAMS_FILE = os.path.join(
  get_package_share_directory("echoflow"),
  'config',
  'microstrain.yaml'
)

def generate_launch_description():
  # Declare arguments with default values
  launch_description = []
  launch_description.append(DeclareLaunchArgument('vessel_ns',   default_value='aura',                description='Vessel namespace'))
  launch_description.append(DeclareLaunchArgument('node_name',   default_value='microstrain',      description='Name to give the Microstrain Inertial Driver node'))
  launch_description.append(DeclareLaunchArgument('configure',   default_value='true',            description='Whether or not to configure the node on startup'))
  launch_description.append(DeclareLaunchArgument('activate',    default_value='true',            description='Whether or not to activate the node on startup'))
  launch_description.append(DeclareLaunchArgument('debug',       default_value='false',            description='Whether or not to log debug information.'))

  # Pass an environment variable to the node to determine if it is in debug or not
  launch_description.append(SetEnvironmentVariable('MICROSTRAIN_INERTIAL_DEBUG', value=LaunchConfiguration('debug')))

  # ******************************************************************
  # Microstrain sensor node
  # ******************************************************************
  microstrain_node = LifecycleNode(
    package    = _PACKAGE_NAME,
    executable = "microstrain_inertial_driver_node",
    name       = LaunchConfiguration('node_name'),
    namespace  = PythonExpression(["'/", LaunchConfiguration('vessel_ns'), "/nav/sensors/mru'"]),
    respawn=True,
    respawn_delay=4,
    parameters = [
      # Load the default params file manually, since this is a ROS params file, we will need to load the file manually
      yaml.safe_load(open(_DEFAULT_PARAMS_FILE, 'r')),

      # If you want to override any settings in the params.yml file, make a new yaml file, and set the value via the params_file arg
      yaml.safe_load(open(_TREVOR_PARAMS_FILE, 'r')),

      # Supported overrides
      {
        "debug" : LaunchConfiguration('debug'),
        # Frame IDs with vessel_ns as prefix
        "mount_frame_id": PythonExpression(["'", LaunchConfiguration('vessel_ns'), "/base_link'"]),
        "gnss1_frame_id": PythonExpression(["'", LaunchConfiguration('vessel_ns'), "/gnss_1_antenna_link'"]),
        "gnss2_frame_id": PythonExpression(["'", LaunchConfiguration('vessel_ns'), "/gnss_2_antenna_link'"]),
        "odometer_frame_id": PythonExpression(["'", LaunchConfiguration('vessel_ns'), "/odometer_link'"]),
      },
    ]
  )

  # Optional configure and activate steps
  config_event = EmitEvent(
    event = ChangeState(
      lifecycle_node_matcher = matches_action(microstrain_node),
      transition_id          = Transition.TRANSITION_CONFIGURE
    ),
    condition = LaunchConfigurationEquals('configure', 'true')
  )
  activate_event = EmitEvent(
    event = ChangeState(
      lifecycle_node_matcher = matches_action(microstrain_node),
      transition_id          = Transition.TRANSITION_ACTIVATE
    ),
    condition = LaunchConfigurationEquals('activate', 'true')
  )

  launch_description.append(microstrain_node)
  launch_description.append(config_event)
  launch_description.append(activate_event)
  return LaunchDescription(launch_description)
