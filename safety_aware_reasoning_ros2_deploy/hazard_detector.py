"""
Hazard Detector
Emily Sheetz, NSTGRO VTE, Summer 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Bool
from vision_msgs.msg import ObjectHypothesis
from geometry_msgs.msg import PoseStamped
from dex_ivr_interfaces.srv import BlobCentroid
from sar_ros2_interfaces.msg import DetectedHazards

class HazardDetector(Node):
    def __init__(self):
        # initialize parent class
        super().__init__("SAR_Hazard_Detector")

        # initialize internal parameters
        self.loop_rate = 1.0 # Hz
        self.min_blob_size = 25.0 # look for bigger blobs to reduce noise
        self.looking_for_object = False
        self.manipulation_blob_color = "red"
        self.navigation_blob_color = "green" # TODO color?
        self.detected_blob_pose = {
            self.manipulation_blob_color : None,
            self.navigation_blob_color : None
        }
        self.detected_humans = None

        # initialize connections and service clients
        self.initialize_connections()
        self.initialize_service_clients()

        self.get_logger().info("Constructed")

    ######################
    ### INITIALIZATION ###
    ######################

    def initialize_connections(self):
        # initialize callback groups
        self.looking_for_object_cb_group = MutuallyExclusiveCallbackGroup()
        self.human_detection_cb_group = MutuallyExclusiveCallbackGroup()
        self.detected_hazards_cb_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.looking_for_object_sub = self.create_subscription(
            Bool,
            "/robot_looking_for_object",
            self.looking_for_object_callback,
            10,
            callback_group=self.looking_for_object_cb_group
        )
        self.looking_for_object_sub # prevent unused variable warning

        self.human_detection_sub = self.create_subscription(
            ObjectHypothesis,
            "/yolo/detections",
            self.human_detection_callback,
            10,
            callback_group=self.human_detection_cb_group
        )
        self.human_detection_sub # prevent unused variable warning

        self.get_logger().info("Created subscribers!")

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

        self.get_logger().info("Created publishers!")

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
        self.get_logger().info("Service /color_blob_find exists!")

        self.get_logger().info("Created service clients!")

        return

    #################
    ### CALLBACKS ###
    #################

    def looking_for_object_callback(self, msg : Bool):
        # set flag
        self.looking_for_object = msg.data
        self.get_logger().info(
            "Robot {} currently looking for an object to interact with".format(
                "IS" if self.looking_for_object else "IS NOT"
            )
        )

        return

    def human_detection_callback(self, msg : ObjectHypothesis):
        # check if person detected
        if msg.class_id == 'person':
            self.detected_humans = msg.score
            # check confidence of detection
            if msg.score > 0.0:
                self.get_logger().info("Human detector found a person with confidence {}".format(self.detected_humans))
            else:
                self.get_logger().debug("Human detector did not find a person")

        return

    def detect_hazards_callback(self):
        # create empty hazard message
        haz_msg = DetectedHazards()

        # call perception helpers
        self.call_manipulation_color_blob_service()
        self.call_navigation_color_blob_service()

        # check each detected hazard
        human_present = self.human_detection()
        expected_object_seen = self.expected_object_interaction_detection()
        object_fell = (not expected_object_seen) if self.looking_for_object else False
        unexpected_object_seen = self.unexpected_object_interaction_detection()
        nav_collision = self.navigation_collision_detection()

        # reset perceived objects
        self.reset_perceived_objects()

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

        # navigation collision
        haz_msg.hazard_names.append("robot_navigation_object_collision")
        haz_msg.hazard_detections.append(nav_collision)

        # map collision
        haz_msg.hazard_names.append("environment_map_collision")
        haz_msg.hazard_detections.append(nav_collision)

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

    def color_blob_detection_done_callback(self, response, color, blob_name):
        # got response!
        self.get_logger().debug("Service call successful!")

        if blob_name == "NAVIGATION":
            self.detected_blob_pose[color] = None
            return

        # check response
        if self.check_color_blobs_detected(response):
            self.get_logger().info("Color blob {} detector found something".format(blob_name))
            self.detected_blob_pose[color] = response.result().centroid_pose
        else:
            self.get_logger().debug("Color blob {} detector did not find anything".format(blob_name))
            self.detected_blob_pose[color] = None

        return

    def manip_color_blob_detection_done_callback(self, response):
        self.color_blob_detection_done_callback(response,
                                                self.manipulation_blob_color,
                                                "MANIPULATION")

    def nav_color_blob_detection_done_callback(self, response):
        self.color_blob_detection_done_callback(response,
                                                self.navigation_blob_color,
                                                "NAVIGATION")

    #############################################
    ### COLOR BLOB DETECTION HELPER FUNCTIONS ###
    #############################################

    def check_color_blobs_detected(self, response):
        # check if color blobs detected
        return (response.result().centroid_pose.header.frame_id != "")

    def call_color_blob_service(self, color, callback_function):
        # create request
        request = BlobCentroid.Request()

        # set the color, use default for everything else
        request.color = color
        request.min_blob_size = self.min_blob_size

        # call service
        future = self.color_blob_client.call_async(request)
        future.add_done_callback(callback_function)

    def call_manipulation_color_blob_service(self):
        self.call_color_blob_service(self.manipulation_blob_color,
                                     self.manip_color_blob_detection_done_callback)

    def call_navigation_color_blob_service(self):
        self.call_color_blob_service(self.navigation_blob_color,
                                     self.nav_color_blob_detection_done_callback)

    #########################################
    ### HAZARD DETECTION HELPER FUNCTIONS ###
    #########################################

    def reset_perceived_objects(self):
        # reset detected color blobs; this gets called regularly
        self.detected_blob_pose[self.manipulation_blob_color] = None
        self.detected_blob_pose[self.navigation_blob_color] = None
        # do not reset human detections; persist detection until new detection is received
        # self.detected_humans = None
        return

    def human_detection(self) -> bool:
        if self.detected_humans != None:
            # threshold human detection confidence
            return (self.detected_humans > 0.10) # TODO threshold?
        else: # self.detected_humans is None
            return False

    def expected_object_interaction_detection(self) -> bool:
        if self.looking_for_object and self.detected_blob_pose[self.manipulation_blob_color]:
            return True
        else:
            # not looking for object or no detected object
            return False

    def unexpected_object_interaction_detection(self) -> bool:
        if (not self.looking_for_object) and self.detected_blob_pose[self.manipulation_blob_color]:
            return True
        else:
            # looking for object or no detected object
            return False

    def navigation_collision_detection(self) -> bool:
        if self.detected_blob_pose[self.navigation_blob_color]:
            return True
        else:
            # no detected object
            return False
