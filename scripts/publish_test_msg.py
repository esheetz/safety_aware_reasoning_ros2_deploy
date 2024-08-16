#!/usr/bin/env python3
"""
Publish Test Messages
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sar_ros2_interfaces.msg import DetectedHazards

#####################
### MAIN FUNCTION ###
#####################

def main(args=None):
    # initialize node
    rclpy.init(args=args)

    # create node
    node = Node("Test_Publisher")


    # multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # create publishers
    node.detected_hazards_pub = node.create_publisher(
        DetectedHazards,
        "/sar_detected_hazards",
        10
    )

    test_msg = DetectedHazards()

    # set message fields for hazard
    # hazard: human in workspace
    test_msg.hazard_names.append("human_enters_workspace")
    test_msg.hazard_detections.append(False)

    # hazard: object fell
    test_msg.hazard_names.append("object_falls")
    test_msg.hazard_detections.append(False)

    # collision during manipulation
    test_msg.hazard_names.append("robot_manipulation_object_collision")
    test_msg.hazard_detections.append(False)

    # collision during manipulation with object in-hand
    test_msg.hazard_names.append("robot_inhand_manipulation_object_collision")
    test_msg.hazard_detections.append(False)

    # navigation collision
    test_msg.hazard_names.append("robot_navigation_object_collision")
    test_msg.hazard_detections.append(False)

    # map collision
    test_msg.hazard_names.append("environment_map_collision")
    test_msg.hazard_detections.append(False)

    node.detected_hazards_pub.publish(test_msg)

    # spin
    executor.spin()

    # destroy node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
