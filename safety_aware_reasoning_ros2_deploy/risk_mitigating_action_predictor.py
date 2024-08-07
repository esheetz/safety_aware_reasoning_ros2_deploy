"""
Risk Mitigating Action Predictor
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import String # TODO what is this for?
from sar_ros2_interfaces.msg import DetectedHazards, RiskMitigatingAction
import safety_aware_reasoning_ros2_deploy.model_data_formatting_helpers as MHelpers

from copy import deepcopy
import numpy as np
import pandas as pd
import pickle

class RiskMitigatingActionPredictor(Node):
    def __init__(self):
        # initialize parent class
        super().__init__("SAR_Risk_Mitigating_Action_Predictor")

        # initialize internal parameters
        self.rma = RiskMitigatingAction()
        self.rma.action = self.rma.NONE # current risk mitigating action
        self.detected_hazards = {} # dictionary of detected hazards

        # read ROS parameters
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name = self.get_parameter("robot_name").value
        self.declare_parameter("environment", rclpy.Parameter.Type.STRING)
        self.environment = self.get_parameter("environment").value
        self.declare_parameter("saved_model_file_path", rclpy.Parameter.Type.STRING)
        self.saved_model_file_path = self.get_parameter("saved_model_file_path").value
        self.saved_model_file = self.saved_model_file_path + "/" + \
                                self.robot_name + "_" + self.environment + "_model.sav"
        self.declare_parameter("risky_condition_file_path", rclpy.Parameter.Type.STRING)
        self.risk_file_path = self.get_parameter("risky_condition_file_path").value
        self.risk_file = self.risk_file_path + "/" + \
                                self.robot_name + "/risky_conditions.yaml"

        # get condition risk/consequence scores for robot and environment
        self.hazard_risk_conseqs = MHelpers.get_condition_risk_scores_for_robot_env(
            self.robot_name,
            self.environment,
            self.risk_file
        )
        self.get_logger().debug("Reading risk conditions from file: {}".format(self.risk_file))
        self.get_logger().debug("Read the following risky conditions: {}".format(self.hazard_risk_conseqs))

        # initialize connections
        self.initialize_connections()

        # load in saved model
        self.load_saved_model(self.environment, self.saved_model_file)

        self.get_logger().info("Constructed")

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_connections(self):
        # initialize callback groups
        self.detected_hazards_cb_group = MutuallyExclusiveCallbackGroup()

        # create subscriber
        self.detected_hazards_sub = self.create_subscription(
            DetectedHazards,
            "/sar_detected_hazards",
            self.detected_hazards_callback,
            10,
            callback_group=self.detected_hazards_cb_group
        )
        self.detected_hazards_sub # prevent unused variable warning

        self.get_logger().info("Created subscribers!")

        # create publisher
        self.rma_pub = self.create_publisher(
            RiskMitigatingAction,
            "/sar_risk_mitigating_action",
            10
        )

        self.get_logger().info("Created publishers!")

        return

    #################
    ### CALLBACKS ###
    #################

    def detected_hazards_callback(self, msg):
        # add detected hazards to internal message
        for i in range(len(msg.hazard_names)):
            # set flag for hazard
            self.detected_hazards[msg.hazard_names[i]] = msg.hazard_detections[i]
            # note that this assumes hazards persist unless explicitly detected to be False

        # predict risk mitigating action
        self.predict_risk_mitigating_action()

        return

    ####################################
    ### MODEL HELPERS AND PREDICTION ###
    ####################################

    def risk_mitigating_action_encodings(self, env):
        # initialize dictionary
        self.rma_prediction_to_rma_value = {}

        # set based on environment
        if env == 'lunar_habitat':
            self.rma_prediction_to_rma_value = {
                0 : self.rma.LUNAR_SUPERVISED_AUTONOMY_BACKUP_NAVIGATION,
                1 : self.rma.LUNAR_TELEOPERATED_NAVIGATION,
                2 : self.rma.LUNAR_TELEOPERATED_MANIPULATION,
                3 : self.rma.LUNAR_ASK_HUMAN_INTERVENTION_TO_PROCEED,
                4 : self.rma.LUNAR_ABORT_TASK
            }
        elif env == 'household':
            self.rma_prediction_to_rma_value = {
                0 : self.rma.HOUSEHOLD_LOWER_JOINT_VELOCITIES_TORQUES,
                1 : self.rma.HOUSEHOLD_SHARED_AUTONOMY_MANIPULATION,
                2 : self.rma.HOUSEHOLD_SHARED_AUTONOMY_NAVIGATION,
                3 : self.rma.HOUSEHOLD_SUPERVISED_AUTONOMY_BACKUP_NAVIGATION,
                4 : self.rma.HOUSEHOLD_TELEOPERATED_MANIPULATION,
                5 : self.rma.HOUSEHOLD_ASK_HUMAN_INTERVENTION_TO_PROCEED
            }
        else:
            self.get_logger().error("Unrecognized environment {}".format(env))
            return

        self.get_logger().info(
            "Recognizing {} risk mitigating actions for {} environment".format(
                len(self.rma_prediction_to_rma_value.keys()), env
            )
        )

        return

    def load_saved_model(self, env, saved_model_name):
        # set saved model file name
        self.saved_model_file = saved_model_name
        self.get_logger().info("Loading in model from {}".format(saved_model_name))

        # load in model
        self.rma_prediction_model = pickle.load(open(saved_model_name, 'rb'))

        # get model features
        self.rma_prediction_model_features = deepcopy(self.rma_prediction_model.model.exog_names)
        self.rma_prediction_model_features.remove('Intercept')
        self.get_logger().info(
            "Loaded model will use the following features to predict risk mitigating action: {}".format(
                self.rma_prediction_model_features
            )
        )

        # set up helper dictionary for model states to risk mitigating actions
        self.risk_mitigating_action_encodings(env)

        return

    def get_current_state_information_for_model_input(self):
        # at least one hazard detected, initialize current state info and helper lists
        curr_state_dict = {}
        detected_risk_scores = []
        detected_conseq_scores = []

        # format data for each model feature
        for feature in self.rma_prediction_model_features:
            # initialize value
            curr_state_dict[feature] = [0.0]
            # check feature name
            if MHelpers.check_col_name_for_condition_risk(feature):
                # get hazardous condition name
                hazard = MHelpers.get_condition_name_from_col_name(feature)
                # check if hazard exists and has been detected
                if (hazard in self.detected_hazards.keys()) and (self.detected_hazards[hazard]):
                    # hazard detected, get risk and consequence score
                    hazard_risk = self.hazard_risk_conseqs[hazard]['risk']
                    hazard_conseq = self.hazard_risk_conseqs[hazard]['consequence']
                    # update helpers lists
                    detected_risk_scores.append(hazard_risk)
                    detected_conseq_scores.append(hazard_conseq)
                    # set current state information with risk score
                    curr_state_dict[feature] = [hazard_risk]
                # else hazard not detected, nothing to update
            elif MHelpers.check_col_name_for_state_risk(feature):
                # compute state risk
                state_risk = MHelpers.compute_state_risk_score(detected_risk_scores)
                # set current state information with state risk score
                curr_state_dict[feature] = [state_risk]
            elif MHelpers.check_col_name_for_state_conseq(feature):
                # compute state consequence
                state_conseq = MHelpers.compute_state_consequence_score(detected_conseq_scores)
                # set current state information with state consequence score
                curr_state_dict[feature] = [state_conseq]
            else:
                self.get_logger().error(
                    "Unrecognized model feature {}".format(
                        feature
                    )
                )
                return None

        return curr_state_dict

    def predict_risk_mitigating_action(self):
        # check if no hazards are detected
        if not any(self.detected_hazards.values()):
            self.rma.action = self.rma.NONE
            self.rma_pub.publish(self.rma)
            return

        # at least one hazard detected, get current state info
        curr_state_dict = self.get_current_state_information_for_model_input()
        if curr_state_dict is None:
            self.get_logger().error(
                "Could not get current state info for model prediction"
            )
            return

        # format current state information as data frame
        self.get_logger().info("CURRENT STATE INFO: {}".format(curr_state_dict))
        curr_state = pd.DataFrame(curr_state_dict)

        # get prediction from model based on current state
        rma_pred_prob = self.rma_prediction_model.predict(curr_state)

        # get most likely risk mitigating action
        rma_pred = np.argmax(np.array(rma_pred_prob), axis=1)

        # set risk mitigating action based on prediction
        self.rma.action = self.rma_prediction_to_rma_value[rma_pred[0]]
        # TODO record all actions over probability threshold?

        # publish risk mitigating action
        self.rma_pub.publish(self.rma)

        return
