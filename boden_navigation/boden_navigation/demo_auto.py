import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import yaml
import os
import sys
import time

class YamlWaypointParser:
    """
    Parse a set of waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        waypoints = []
        print(self.wps_dict)
        for wp in self.wps_dict["waypoints"]:
            x, y, z = wp["x"], wp["y"], wp["z"]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            waypoints.append(pose)

        return waypoints


class DemoAuto():
    """
    Class to use run demo autonomous code
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        wps = self.wp_parser.get_wps()
        self.navigator.followWaypoints(wps)

        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)

        print("wps completed successfully")


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "boden_navigation"), "config", "waypoints.yml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = DemoAuto(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()