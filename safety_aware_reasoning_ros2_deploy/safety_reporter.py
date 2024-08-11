"""
Safety Reporter
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sar_ros2_interfaces.msg import DetectedHazards, RiskMitigatingAction

class SafetyReporter(Node):
    def __init__(self):
        # initialize parent class
        super().__init__("SAR_Safety_Reporter")

        # initialize internal parameters
        self.last_risk_mitigating_action_msg = None
        self.last_detected_hazards = {}
        self.last_rma = None
        # string for spacing out reports
        self.spacer_stub = "-"
        self.spacer_size = 40
        # string for tabbing reports
        self.tab_str = " " * 4
        # strings for advanced operator report
        # self.advanced_operator = ["", "CONTROL-LEVEL INFORMATION FOR OPERATOR"]
        # strings for operator suggestions
        self.suggestion = ["", "Suggestion for operator:"]

        # initialize connections
        self.initialize_connections()

        self.get_logger().info("Node constructed! Ready to report!")

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_connections(self):
        # initialize callback groups
        self.rma_cb_group = MutuallyExclusiveCallbackGroup()

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

        return

    #################
    ### CALLBACKS ###
    #################

    def risk_mitigating_action_callback(self, msg : RiskMitigatingAction):
        # store internally and check for differences
        diff = self.check_changes_since_last_message(msg)

        # check for diff
        if not diff:
            return
        # from here can assume that something has changed, which means report is needed

        # create report
        report = []
        report = self.create_safety_report()

        # print report
        self.print_report(report)

        return

    #####################################
    ### CHECK FOR CHANGES IN MESSAGES ###
    #####################################

    def check_changes_since_last_message(self, msg : RiskMitigatingAction):
        # initialize diff flag
        diff = False

        # initialize detected hazards dictionary
        new_detected_hazards = {}

        # check detected hazards
        for i in range(len(msg.hazards.hazard_names)):
            # get hazard name
            hazard = msg.hazards.hazard_names[i]
            # see if hazard does not exist yet
            if hazard not in self.last_detected_hazards.keys():
                diff = True
            # see if hazard exists and has previous detection
            if hazard in self.last_detected_hazards.keys():
                # see if hazard detection has changed
                if msg.hazards.hazard_detections[i] != self.last_detected_hazards[hazard]:
                    diff = True
            # set hazard in dictionary
            new_detected_hazards[hazard] = msg.hazards.hazard_detections[i]

        # check if action is different
        if msg.action != self.last_rma:
            diff = True

        # store internally
        self.last_risk_mitigating_action_msg = msg
        self.last_rma = msg.action
        # update detected hazards and persist past detections
        for haz,det in new_detected_hazards.items():
            self.last_detected_hazards[haz] = det

        return diff

    ############################
    ### CREATE SAFETY REPORT ###
    ############################

    def create_safety_report(self):
        # initialize report
        report = []

        # create report based on hazards
        hazard_report = self.create_hazard_report()

        # create report based on risk mitigating action
        action_report = self.create_action_report()

        # create full report
        report = hazard_report + action_report

        return report

    def create_hazard_report(self):
        # initialize report
        report = []
        report.append("Identified the following hazards:")

        # check if any hazards detected
        if not any(self.last_detected_hazards.values()):
            report.append(self.tab_str + "No hazards detected, safe for tasks!")
        else:
            for hazard in self.last_detected_hazards.keys():
                if self.last_detected_hazards[hazard]:
                    report.append(self.tab_str + hazard)

        report.append("")

        return report

    def create_action_report(self):
        # initialize report
        report = []
        report.append("Predicted the following risk mitigating action:")

        # add information for each action
        if self.last_rma == self.last_risk_mitigating_action_msg.NONE:
            report.append(self.tab_str + "No action needed, safe for tasks!")
        elif self.last_rma == self.last_risk_mitigating_action_msg.HOUSEHOLD_LOWER_JOINT_VELOCITIES_TORQUES:
            report.append(self.tab_str + "Lowering joint velocities and torques")
            report = report + self.suggestion
            report.append(self.tab_str + "There may be a human nearby making it unsafe to move at regular speed.")
            report.append(self.tab_str + "Ensure robot's workspace is clear to avoid damage.")
        elif self.last_rma == self.last_risk_mitigating_action_msg.HOUSEHOLD_SHARED_AUTONOMY_MANIPULATION:
            report.append(self.tab_str + "Requesting shared autonomy to complete manipulation task")
            report = report + self.suggestion
            report.append(self.tab_str + "There may be something in the way.")
            report.append(self.tab_str + "Help the robot maneuver around the object.")
        elif self.last_rma == self.last_risk_mitigating_action_msg.HOUSEHOLD_SHARED_AUTONOMY_NAVIGATION:
            report.append(self.tab_str + "Requesting shared autonomy to complete navigation task")
            report = report + self.suggestion
            report.append(self.tab_str + "There may be something blocking the robot's path.")
            report.append(self.tab_str + "Help the robot navigate around the obstacle.")
        elif (self.last_rma == self.last_risk_mitigating_action_msg.HOUSEHOLD_SUPERVISED_AUTONOMY_BACKUP_NAVIGATION) or \
            (self.last_rma == self.last_risk_mitigating_action_msg.LUNAR_SUPERVISED_AUTONOMY_BACKUP_NAVIGATION):
            report.append(self.tab_str + "Requesting supervised autonomy to complete navigation task")
            report = report + self.suggestion
            report.append(self.tab_str + "There may be a known map collision.")
            report.append(self.tab_str + "Supervise the robot as it attempts to navigate around the obstacle.")
        elif (self.last_rma == self.last_risk_mitigating_action_msg.HOUSEHOLD_TELEOPERATED_MANIPULATION) or \
            (self.last_rma == self.last_risk_mitigating_action_msg.LUNAR_TELEOPERATED_MANIPULATION):
            report.append(self.tab_str + "Requesting teleoperation to complete manipulation task")
            report = report + self.suggestion
            report.append(self.tab_str + "There may be something in the way, making it difficult for the robot to manipulate objects.")
            report.append(self.tab_str + "Take over the task and command the robot to complete manipulation task.")
        elif (self.last_rma == self.last_risk_mitigating_action_msg.HOUSEHOLD_ASK_HUMAN_INTERVENTION_TO_PROCEED) or \
            (self.last_rma == self.last_risk_mitigating_action_msg.LUNAR_ASK_HUMAN_INTERVENTION_TO_PROCEED):
            report.append(self.tab_str + "Unsure how to proceed; requesting human intervention")
            report = report + self.suggestion
            report.append(self.tab_str + "The robot may be expecting an object that it cannot perceive.")
            report.append(self.tab_str + "Help the robot continue the task.")
        elif self.last_rma == self.last_risk_mitigating_action_msg.LUNAR_TELEOPERATED_NAVIGATION:
            report.append(self.tab_str + "Requesting teleoperation to complete navigation task")
            report = report + self.suggestion
            report.append(self.tab_str + "There may be something blocking the robot's path, making it difficult for the robot to navigate.")
            report.append(self.tab_str + "Take over the task and command the robot to complete navigation task.")
        elif self.last_rma == self.last_risk_mitigating_action_msg.LUNAR_ABORT_TASK:
            report.append(self.tab_str + "Aborting task")
            report = report + self.suggestion
            report.append(self.tab_str + "Unsafe to continue task.")
            report.append(self.tab_str + "Address the identified hazards and try the task again.")
        else:
            self.get_logger().error("Unidentified risk mitigating action {}".format(self.last_rma))

        return report

    ###########################
    ### PRINT SAFETY REPORT ###
    ###########################

    def print_report(self, report):
        # check if report is empty
        if (report is None) or (len(report) == 0):
            return

        # print non-empty report
        self.get_logger().info("")
        self.get_logger().info(self.spacer_stub * self.spacer_size)
        self.get_logger().info("")
        for line in report:
            self.get_logger().info(line)
        self.get_logger().info("")
        self.get_logger().info(self.spacer_stub * self.spacer_size)
        self.get_logger().info("")

        return
