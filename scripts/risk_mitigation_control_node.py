#!/usr/bin/env python3
"""
Risk Mitigation Control - Main Function
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from safety_aware_reasoning_ros2_deploy.risk_mitigation_control import RiskMitigationControl

#####################
### MAIN FUNCTION ###
#####################

def main(args=None):
    # initialize node
    rclpy.init(args=args)

    # create node
    control = RiskMitigationControl()

    # multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(control)

    # spin
    executor.spin()

    # destroy node
    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
