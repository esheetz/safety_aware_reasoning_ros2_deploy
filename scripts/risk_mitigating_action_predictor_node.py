#!/usr/bin/env python3
"""
Risk Mitigating Action Predictor - Main Function
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from safety_aware_reasoning_ros2_deploy.risk_mitigating_action_predictor import RiskMitigatingActionPredictor

#####################
### MAIN FUNCTION ###
#####################

def main(args=None):
    # initialize node
    rclpy.init(args=args)

    # create node
    predictor = RiskMitigatingActionPredictor()

    # multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(predictor)

    # spin
    executor.spin()

    # destroy node
    predictor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
