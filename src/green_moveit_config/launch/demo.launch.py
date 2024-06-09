from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os



from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kuka_green_iiwa7", package_name="green_moveit_config")
        .robot_description(
            os.path.join(
                get_package_share_directory("lbr_description"),
                "urdf/kuka_green_iiwa7/green_iiwa7.xacro",
            )
        )
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
