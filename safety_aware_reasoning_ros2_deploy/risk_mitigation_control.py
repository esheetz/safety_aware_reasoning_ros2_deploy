"""
Risk Mitigation Control
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool, Float32
from sar_ros2_interfaces.msg import RiskMitigatingAction

class RiskMitigationControl(Node):
    def __init__(self):
        # initialize parent class
        super().__init__("SAR_Risk_Mitigation_Control")

        # read ROS parameters
        self.declare_parameter("robot", rclpy.Parameter.Type.STRING)
        self.robot = self.get_parameter("robot").value

        # initialize internal parameters
        self.soft_estop_topic = None
        if self.robot == 'val':
            self.soft_estop_topic = "/val_soft_stop/safety_flag"
        elif self.robot == 'clr':
            self.soft_estop_topic = "/drt_soft_stop/safety_flag"
        else:
            self.get_logger().error("Unrecognized robot {}, expect 'val' or 'clr'".format(self.robot))

        # initialize connections
        self.initialize_connections()

        self.get_logger().info("Constructed")

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_connections(self):
        # initialize callback groups
        self.rma_cb_group = ReentrantCallbackGroup()

        # create subscriber
        self.rma_sub = self.create_subscription(
            RiskMitigatingAction,
            "/sar_risk_mitigating_action",
            self.risk_mitigating_action_callback,
            10,
            callback_group=self.rma_cb_group
        )
        self.rma_sub # prevent unused variable warning

        self.get_logger().info("Created subscribers!")

        # create publishers
        self.soft_estop_pub = self.create_publisher(
            Bool,
            self.soft_estop_topic,
            10
        )
        # for lowering joint velocities/torques
        self.velocity_scaling_pub = self.create_publisher(
            Float32,
            "/safety/velocity_scaling",
            10
        )
        self.torque_scaling_pub = self.create_publisher(
            Float32,
            "/safety/acceleration_scaling",
            10
        )

        self.get_logger().info("Created publishers!")

        return

    #################
    ### CALLBACKS ###
    #################

    def risk_mitigating_action_callback(self, msg : RiskMitigatingAction):
        # check risk mitigating action type
        if msg.action == msg.NONE:
            # no action to perform
            return
        if msg.action == msg.HOUSEHOLD_LOWER_JOINT_VELOCITIES_TORQUES:
            # lower joint velocities
            self.perform_lower_joint_velocities_torques()

        # perform soft e-stop
        self.perform_soft_estop()

        return

    #######################################
    ### PERFORM RISK MITIGATING ACTIONS ###
    #######################################

    def perform_soft_estop(self):
        # publish soft e-stop message
        estop_msg = Bool()
        estop_msg.data = True
        self.soft_estop_pub.publish(estop_msg)

        self.get_logger().info("Performing soft e-stop for robot {}".format(self.robot))

        return

    def perform_lower_joint_velocities_torques(self):
        # publish low values for MoveIt velocity and acceleration scaling
        low_scaling_msg = Float32()
        low_scaling_msg.data = 0.5
        self.velocity_scaling_pub.publish(low_scaling_msg)
        self.torque_scaling_pub.publish(low_scaling_msg)

        self.get_logger().info(
            "Lowering joint velocity and torque scaling to {} for robot {}".format(
                low_scaling_msg.data, self.robot
            )
        )

        return
