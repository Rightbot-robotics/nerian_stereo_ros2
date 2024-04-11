from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    static_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["-0.036639", "0.067622", "1.833859", "-1.568721", "0.013283", "-1.870109", "dc_pillar", "nerian_stereo_right_color_optical_frame"])

    nerian_stereo_node = Node(
                package='nerian_stereo',
                executable='nerian_stereo_node',
                parameters=[
                    {'remote_host':                  '192.168.10.10'}, # old nerian camera
                    {'remote_port':                  '7681'},
                    {'use_tcp':                       True},

                    {'top_level_frame':               'world'},
                    {'internal_frame':                'nerian_stereo_left_color_optical_frame'},
                    {'camera_name':                   'nerian_stereo_left'},
                    {'ros_coordinate_system':         False},
                    {'ros_timestamps':                True},

                    {'max_depth':                     -1},
                    {'point_cloud_intensity_channel', 'mono8'},
                    {'color_code_disparity_map',      ''},
                    {'color_code_legend':             False},

                    {'calibration_file':              '/app/vision_ws/src/sherlock_cv/nerian_stereo_ros2/launch/left_calib_11_4_2024.yaml'},
                    {'q_from_calib_file':             False},
                    {'delay_execution':               0.0},
 
                ]
            )
    ld.add_action(static_tf)
    ld.add_action(nerian_stereo_node)
    return ld

