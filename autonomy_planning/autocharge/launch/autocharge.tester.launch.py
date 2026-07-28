"""Launch docking_tester for random or grid (success-rate matrix) runs."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_GRID_LATERAL_OFFSETS_M = [
    0.05,
    -0.05,
    0.10,
    -0.10,
    0.15,
    -0.15,
    0.20,
    -0.20,
    0.25,
    -0.25,
    0.30,
    -0.30,
]
_GRID_REPEATS = 10
_GRID_CASE_COUNT = len(_GRID_LATERAL_OFFSETS_M) * _GRID_REPEATS


def _grid_scenario_params(scenario: str) -> dict:
    if scenario == 'far_50cm':
        return {
            'pose_schedule': 'grid',
            'grid_scenario': 'far_50cm',
            'grid_longitudinal_m': 0.5,
            'max_tests': _GRID_CASE_COUNT,
        }
    if scenario == 'far_70cm':
        return {
            'pose_schedule': 'grid',
            'grid_scenario': 'far_70cm',
            'grid_longitudinal_m': 0.7,
            'max_tests': _GRID_CASE_COUNT,
        }
    return {}


def _launch_setup(context, *args, **kwargs):
  scenario = LaunchConfiguration('test_scenario').perform(context).strip()
  grid_extra = _grid_scenario_params(scenario)

  metrics_topic = LaunchConfiguration('metrics_topic')
  state_topic = LaunchConfiguration('state_topic')
  tester_save_dir = LaunchConfiguration('tester_save_dir')
  plot_enable = LaunchConfiguration('plot_enable')
  max_tests = LaunchConfiguration('max_tests')
  case_timeout = LaunchConfiguration('case_timeout_s')
  success_state = LaunchConfiguration('success_state')
  fail_states = LaunchConfiguration('fail_states')
  auto_stop_when_done = LaunchConfiguration('auto_stop_when_done')
  save_interval = LaunchConfiguration('save_interval_s')
  randomize_pose_before_start = LaunchConfiguration('randomize_pose_before_start')
  initialpose_topic = LaunchConfiguration('initialpose_topic')
  random_pose_x_min = LaunchConfiguration('random_pose_x_min')
  random_pose_x_max = LaunchConfiguration('random_pose_x_max')
  random_pose_y_min = LaunchConfiguration('random_pose_y_min')
  random_pose_y_max = LaunchConfiguration('random_pose_y_max')
  random_pose_yaw_min = LaunchConfiguration('random_pose_yaw_min')
  random_pose_yaw_max = LaunchConfiguration('random_pose_yaw_max')

  params = {
      'metrics_topic': metrics_topic,
      'state_topic': state_topic,
      'save_dir': tester_save_dir,
      'plot_enable': plot_enable,
      'max_tests': max_tests,
      'case_timeout_s': ParameterValue(case_timeout, value_type=float),
      'success_state': success_state,
      'fail_states': fail_states,
      'auto_stop_when_done': auto_stop_when_done,
      'save_interval_s': ParameterValue(save_interval, value_type=float),
      'randomize_pose_before_start': randomize_pose_before_start,
      'initialpose_topic': initialpose_topic,
      'random_pose_x_min': ParameterValue(random_pose_x_min, value_type=float),
      'random_pose_x_max': ParameterValue(random_pose_x_max, value_type=float),
      'random_pose_y_min': ParameterValue(random_pose_y_min, value_type=float),
      'random_pose_y_max': ParameterValue(random_pose_y_max, value_type=float),
      'random_pose_yaw_min': ParameterValue(random_pose_yaw_min, value_type=float),
      'random_pose_yaw_max': ParameterValue(random_pose_yaw_max, value_type=float),
  }

  if grid_extra:
      params.update(
          {
              'pose_schedule': grid_extra['pose_schedule'],
              'grid_scenario': grid_extra['grid_scenario'],
              'grid_longitudinal_m': grid_extra['grid_longitudinal_m'],
              'grid_lateral_offsets_m': _GRID_LATERAL_OFFSETS_M,
              'grid_repeats_per_offset': _GRID_REPEATS,
              'grid_robot_yaw': 0.0,
              'max_tests': grid_extra['max_tests'],
              'random_pose_yaw_min': 0.0,
              'random_pose_yaw_max': 0.0,
              'send_reset_before_test': True,
              'send_start_after_reset': True,
              'reset_after_case': True,
              'start_after_case': True,
              'inter_case_start_delay_s': 1.2,
              'start_delay_s': 1.0,
              'dock_x': 0.5,
              'dock_y': 0.0,
              'dock_yaw': 3.1415926,
              'success_contact_x_m': 0.16,
          }
      )

  return [
      Node(
          package='autocharge',
          executable='docking_tester',
          name='docking_tester',
          output='screen',
          parameters=[params],
      )
  ]


def generate_launch_description() -> LaunchDescription:
    test_scenario_arg = DeclareLaunchArgument(
        'test_scenario',
        default_value='random',
        description=(
            'random | far_50cm (桩前50cm横向矩阵) | far_70cm (桩前70cm横向矩阵)'
        ),
    )
    metrics_topic_arg = DeclareLaunchArgument(
        'metrics_topic',
        default_value='/dock/test_metrics',
        description='Tester metrics topic',
    )
    state_topic_arg = DeclareLaunchArgument(
        'state_topic',
        default_value='/dock/state',
        description='Optional state topic',
    )
    save_dir_arg = DeclareLaunchArgument(
        'tester_save_dir',
        default_value='~/dock_test_logs',
        description='Directory for tester output artifacts',
    )
    plot_enable_arg = DeclareLaunchArgument(
        'plot_enable',
        default_value='true',
        description='Enable matplotlib plotting',
    )
    max_tests_arg = DeclareLaunchArgument(
        'max_tests',
        default_value='50',
        description='How many test cases to run (grid scenario overrides)',
    )
    case_timeout_arg = DeclareLaunchArgument(
        'case_timeout_s',
        default_value='200.0',
        description='Timeout seconds for each test case',
    )
    success_state_arg = DeclareLaunchArgument(
        'success_state',
        default_value='SUCCESS',
        description='State name treated as success',
    )
    fail_states_arg = DeclareLaunchArgument(
        'fail_states',
        default_value='[ABORT, FAIL]',
        description='State list treated as failure',
    )
    auto_stop_arg = DeclareLaunchArgument(
        'auto_stop_when_done',
        default_value='true',
        description='Stop tester automatically after max_tests',
    )
    save_interval_arg = DeclareLaunchArgument(
        'save_interval_s',
        default_value='2.0',
        description='Periodic save interval seconds',
    )
    randomize_pose_before_start_arg = DeclareLaunchArgument(
        'randomize_pose_before_start',
        default_value='true',
        description='Publish initialpose before each start',
    )
    initialpose_topic_arg = DeclareLaunchArgument(
        'initialpose_topic',
        default_value='/initialpose',
        description='Initial pose topic',
    )
    random_pose_x_min_arg = DeclareLaunchArgument(
        'random_pose_x_min',
        default_value='-0.0',
        description='Random start x min (m)',
    )
    random_pose_x_max_arg = DeclareLaunchArgument(
        'random_pose_x_max',
        default_value='0.0',
        description='Random start x max (m)',
    )
    random_pose_y_min_arg = DeclareLaunchArgument(
        'random_pose_y_min',
        default_value='-0.06',
        description='Random start y min (m)',
    )
    random_pose_y_max_arg = DeclareLaunchArgument(
        'random_pose_y_max',
        default_value='0.06',
        description='Random start y max (m)',
    )
    random_pose_yaw_min_arg = DeclareLaunchArgument(
        'random_pose_yaw_min',
        default_value='-0.1',
        description='Random start yaw min (rad)',
    )
    random_pose_yaw_max_arg = DeclareLaunchArgument(
        'random_pose_yaw_max',
        default_value='0.1',
        description='Random start yaw max (rad)',
    )

    return LaunchDescription([
        test_scenario_arg,
        metrics_topic_arg,
        state_topic_arg,
        save_dir_arg,
        plot_enable_arg,
        max_tests_arg,
        case_timeout_arg,
        success_state_arg,
        fail_states_arg,
        auto_stop_arg,
        save_interval_arg,
        randomize_pose_before_start_arg,
        initialpose_topic_arg,
        random_pose_x_min_arg,
        random_pose_x_max_arg,
        random_pose_y_min_arg,
        random_pose_y_max_arg,
        random_pose_yaw_min_arg,
        random_pose_yaw_max_arg,
        OpaqueFunction(function=_launch_setup),
    ])
