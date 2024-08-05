#!/usr/bin/env python3
"""
Hazard Detector - Main Function
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from safety_aware_reasoning_ros2_deploy.hazard_detector import HazardDetector

#####################
### MAIN FUNCTION ###
#####################

def main(args=None):
    # initialize node
    rclpy.init(args=args)

    # create node
    detector = HazardDetector()

    # multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(detector)

    # spin
    executor.spin()

    # destroy node
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
