from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    static_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["-0.078", "0.168", "2.013", "-1.565", "-0.013", "-1.918", "dc_pillar", "nerian_stereo_right_color_optical_frame"])
                       #arguments = ["-0.192529", "0.076735", "2.016955", "-1.509586", "0.029778", "-1.903646", "dc_pillar", "nerian_stereo_right_color_optical_frame"])
#The Translation values: (-0.192529, 0.076735, 2.016955)

#The Rotation values (in radians): (-1.903646, -0.029778, -1.509586)
                    #    arguments = ["1.18596", "1.209708", "0.998358", "-2.000644", "0.001864", "-1.636583", "DC_hinge", "nerian_stereo_left_color_optical_frame"])
                    #    arguments = ["1.234", "1.125", "1.037", "-1.841", "0.025", "-1.855", "DC_hinge", "nerian_stereo_left_color_optical_frame"])

    nerian_stereo_node = Node(
                package='nerian_stereo',
                executable='nerian_stereo_node',
                parameters=[
                    {'remote_host':                  '192.168.10.12'}, # old nerian camera
                    {'remote_port':                  '7681'},
                    {'use_tcp':                       True},

                    {'top_level_frame':               'world'},
                    {'internal_frame':                'nerian_stereo_right_color_optical_frame'},
                    {'camera_name':                   'nerian_stereo_right'},
                    {'ros_coordinate_system':         False},
                    {'ros_timestamps':                True},

                    {'max_depth':                     -1},
                    {'point_cloud_intensity_channel', 'mono8'},
                    {'color_code_disparity_map',      ''},
                    {'color_code_legend':             False},

                    {'calibration_file':              '/app/vision_ws/src/sherlock_cv/nerian_stereo_ros2/launch/left_calib_29_4_2024.yaml'},
                    {'q_from_calib_file':             False},
                    {'delay_execution':               0.0},
 
                ]
            )
    ld.add_action(static_tf)
    ld.add_action(nerian_stereo_node)
    return ld

