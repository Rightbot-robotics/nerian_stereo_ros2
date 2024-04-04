from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    static_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["1.091", "-1.004", "0.963", "-1.115", "-0.011", "-1.559", "DC_hinge", "nerian_stereo_right_color_optical_frame"])
                    #    arguments = ["1.162", "-0.831", "1.035", "-1.460", "-0.160", "-1.812", "DC_hinge", "nerian_stereo_right_color_optical_frame"])

    nerian_stereo_node = Node(
                package='nerian_stereo',
                executable='nerian_stereo_node',
                parameters=[
                    {'remote_host':                  '192.168.1.124'}, # new nerian camera
                    {'remote_port':                  '7681'},
                    {'use_tcp':                       False},

                    {'top_level_frame':               'world'},
                    {'internal_frame':                'nerian_stereo_right_color_optical_frame'},
                    {'camera_name':                   'nerian_stereo_right'},
                    {'ros_coordinate_system':         False},
                    {'ros_timestamps':                True},

                    {'max_depth':                     -1},
                    {'point_cloud_intensity_channel', 'mono8'},
                    {'color_code_disparity_map',      ''},
                    {'color_code_legend':             False},

                    {'calibration_file':              '/app/vision_ws/src/sherlock_cv/nerian_stereo_ros2/launch/right_calib.yaml'},
                    {'q_from_calib_file':             False},
                    {'delay_execution':               0.0},
 
                ]
            )
    ld.add_action(static_tf)
    ld.add_action(nerian_stereo_node)
    return ld

