"""
Hazard Detector
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from dex_ivr_interfaces.srv import BlobCentroid
from sar_ros2_interfaces.msg import DetectedHazards

class HazardDetector(Node):
    def __init__(self):
        # initialize parent class
        super().__init__("SAR_Hazard_Detector")

        # initialize internal parameters
        self.loop_rate = 5.0 # Hz
        self.looking_for_object = False
        self.detected_blob_pose = None
        self.detected_humans = None

        # initialize connections and service clients
        self.initialize_connections()
        self.initialize_service_clients()

        self.get_logger().info("[Hazard Detector] Constructed")

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_connections(self):
        # initialize callback groups
        self.looking_for_object_cb_group = MutuallyExclusiveCallbackGroup()
        self.detected_hazards_cb_group = MutuallyExclusiveCallbackGroup()

        # create subscriber
        self.looking_for_object_sub = self.create_subscription(
            Bool,
            "/robot_looking_for_object",
            self.looking_for_object_callback,
            10,
            callback_group=self.looking_for_object_cb_group
        )
        self.looking_for_object_sub # prevent unused variable warning

        self.get_logger().info("[Hazard Detector] Created subscribers!")

        # create publishers
        self.detected_hazards_pub = self.create_publisher(
            DetectedHazards,
            "/sar_detected_hazards",
            10
        )
        self.unsafe_conditions_pub = self.create_publisher(
            Bool,
            "/sar_unsafe_state",
            10
        )

        # create timer for publisher
        self.detection_timer = self.create_timer(
            1.0 / self.loop_rate, # duration / Hz = period in seconds
            self.detect_hazards_callback,
            callback_group=self.detected_hazards_cb_group
        )

        self.get_logger().info("[Hazard Detector] Created publishers!")

        return

    def initialize_service_clients(self):
        # initialize callback group
        self.color_blob_cb_group = ReentrantCallbackGroup()

        # create client
        self.color_blob_client = self.create_client(
            BlobCentroid,
            '/color_blob_find',
            callback_group=self.color_blob_cb_group
        )

        # wait for service to be advertised and available
        self.color_blob_client.wait_for_service()
        self.get_logger().info("[Hazard Detector] Service /color_blob_find exists!")

        self.get_logger().info("[Hazard Detector] Created service clients!")

        return

    #################
    ### CALLBACKS ###
    #################

    def looking_for_object_callback(self, msg : Bool):
        # set flag
        self.looking_for_object = msg.data
        self.get_logger().info(
            "[Hazard Detector] Robot {} currently looking for an object to interact with".format(
                "IS" if self.looking_for_object else "IS NOT"
            )
        )

        return

    def detect_hazards_callback(self):
        # create empty hazard message
        haz_msg = DetectedHazards()

        # call perception helpers
        self.call_human_detection()
        self.call_color_blob_service()

        # check each detected hazard
        human_present = self.human_detection()
        expected_object_seen = self.expected_object_interaction_detection()
        object_fell = (not expected_object_seen) if self.looking_for_object else False
        unexpected_object_seen = self.unexpected_object_interaction_detection()

        # reset perceived objects
        self.detected_blob_pose = None
        self.detected_humans = None

        # set message fields for hazard
        # hazard: human in workspace
        haz_msg.hazard_names.append("human_enters_workspace")
        haz_msg.hazard_detections.append(human_present)

        # hazard: object fell
        haz_msg.hazard_names.append("object_falls")
        haz_msg.hazard_detections.append(object_fell)

        # collision during manipulation
        haz_msg.hazard_names.append("robot_manipulation_object_collision")
        haz_msg.hazard_detections.append(unexpected_object_seen)

        # collision during manipulation with object in-hand
        haz_msg.hazard_names.append("robot_inhand_manipulation_object_collision")
        haz_msg.hazard_detections.append(unexpected_object_seen)

        # publish all detected hazards message
        self.detected_hazards_pub.publish(haz_msg)

        # publish collective unsafe state message
        unsafe_state = Bool()
        unsafe_state.data = any(haz_msg.hazard_detections)
        self.unsafe_conditions_pub.publish(unsafe_state)

        return

    ###############################
    ### SERVICE CLIENT CALLBACK ###
    ###############################

    def color_blob_detection_done_callback(self, response):
        # got response!
        self.get_logger().debug("[Hazard Detector] Service call successful!")

        # check response
        if self.check_color_blobs_detected(response):
            self.get_logger().info("[Hazard Detector] Color blob detector found something")
            self.detected_blob_pose = response.result().centroid_pose
        else:
            self.get_logger().info("[Hazard Detector] Color blob detector did not find anything")
            self.detected_blob_pose = None

        return

    ########################################
    ### HUMAN DETECTION HELPER FUNCTIONS ###
    ########################################

    def call_human_detection(self):
        # TODO pending Jodi's help with human detection
        return

    #############################################
    ### COLOR BLOB DETECTION HELPER FUNCTIONS ###
    #############################################

    def check_color_blobs_detected(self, response):
        # check if color blobs detected
        return (response.result().centroid_pose.header.frame_id != "")

    def call_color_blob_service(self, color="red"):
        # create request
        request = BlobCentroid.Request()

        # set the color, use default for everything else
        request.color = color

        # call service
        future = self.color_blob_client.call_async(request)
        future.add_done_callback(self.color_blob_detection_done_callback)

    #########################################
    ### HAZARD DETECTION HELPER FUNCTIONS ###
    #########################################

    def human_detection(self) -> bool:
        if self.detected_humans:
            # TODO may need to do some thresholding here
            return True
        else: # self.detected_humans is None
            return False

    def expected_object_interaction_detection(self) -> bool:
        if self.looking_for_object and self.detected_blob_pose:
            return True
        else:
            # not looking for object or no detected object
            return False

    def unexpected_object_interaction_detection(self) -> bool:
        if (not self.looking_for_object) and self.detected_blob_pose:
            return True
        else:
            # looking for object or no detected object
            return False
