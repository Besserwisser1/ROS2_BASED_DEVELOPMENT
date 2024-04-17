import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    webcam = launch_ros.actions.Node(
        package="v4l2_camera", executable="v4l2_camera_node",
        parameters=[
            {"image_size": [1280,736]},
        ],
    )

    detector = launch_ros.actions.Node(
        package="detector", executable="detector_node",


    )

    saver = launch_ros.actions.Node(
        package="saver", executable="saver",

    )

    mavros_node =  launch_ros.actions.Node(
        package="mavros", executable="mavros_node" ,
	parameters=[
            {"fcu_url": "/dev/ttyS1:57600"},
            {"gcs_url": ""},
            {"tgt_system": 1},
            {"tgt_component": 1},
            {"log_output": "screen"},
            {"fcu_protocol": "v2.0"},
            {"respawn_mavros": False},
            {"namespace": "mavros"},
        ],
    )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription([
        webcam,
        detector,
        saver,
    ])





