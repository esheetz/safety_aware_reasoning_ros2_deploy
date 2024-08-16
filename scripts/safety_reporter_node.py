#!/usr/bin/env python3
"""
Safety Reporter - Main Function
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from safety_aware_reasoning_ros2_deploy.safety_reporter import SafetyReporter

#####################
### MAIN FUNCTION ###
#####################

def main(args=None):
    # initialize node
    rclpy.init(args=args)

    # create node
    reporter = SafetyReporter()

    # multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(reporter)

    # spin
    executor.spin()

    # destroy node
    executor.shutdown()
    reporter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
