#!/usr/bin/env python3

# EXPERIMENTAL ROBOTICS LABORATORY
# ASSIGNMENT 1
# By: Ami Sofia Quijano Shimizu

# DESCRIPTION:
# A camera onboard a robot rotates in order to identify all Aruco markers around it. 
# After detection is complete, it rotates to search the markers again in asceding order.
# When found, one by one, the marker is highlighted and published in a custom topic.

# Importing required libraries
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Define the total number of distinct markers in the environment
TOTAL_MARKERS = 5

class MarkerDetector:
    def __init__(self):
        '''Initialize the ROS node, publishers, subscribers, and other parameters'''
        # Initialize ROS node
        rospy.init_node('marker_detector', anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publisher for camera rotation
        self.vel_pub = rospy.Publisher("/robot/camera_vel_controller/command", Float64, queue_size=10)

        # Publisher for images with Aruco markers highlighted
        self.aruco_pub = rospy.Publisher("/aruco_images", Image, queue_size=10)

        # Subscriber to camera feed (in compressed image format)
        self.camera_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.camera_callback, queue_size=10)

        # Subscriber to Aruco publisher for visualizing image (just for debugging!)
        # UNCOMMENT FOR VISUALIZING PUBLISHED IMAGES OUTSIDE RVIZ
        #self.aruco_sub = rospy.Subscriber("/aruco_images", Image, self.aruco_callback, queue_size=10)

        # ArUco dictionary and detector parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Variables for marker tracking
        self.detected_markers = set()  # Set of detected marker IDs
        self.count_published_markers = 0  # Counter for ordered marker search
        self.phase = 1  # 1 = Detection phase, 2 = Ordered search phase

    def camera_callback(self, ros_data):
        '''Callback function for processing images from the camera'''
        # Convert ROS compressed image to OpenCV format
        np_arr = np.frombuffer(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert to grayscale for ArUco detection
        image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the frame
        corners, ids, _ = aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.aruco_params)

        # Process based on the current phase
        if self.phase == 1: 
            self.detect_markers(ids, corners, image_np)
        elif self.phase == 2: 
            self.search_markers(ids, corners, image_np)

    def aruco_callback(self, ros_data):
        '''Display the image being published for verification'''
        # Convert ROS compressed image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        # Display the image in a window for 5 seconds
        cv2.imshow("Aruco Markers", cv_image)  
        cv2.waitKey(3000)

    def detect_markers(self, ids, corners, image_np):
        '''Phase 1: Detect and store all marker IDs in the environment'''
        # If Aruco markers are found,
        if ids is not None:
            # Filter for only new IDs and add them to the set of detected markers
            new_ids = [id_[0] for id_ in ids if id_[0] not in self.detected_markers]
            if new_ids:
                rospy.loginfo(f"New marker detected: {new_ids}")
                self.detected_markers.update(new_ids)

        # Rotate until all markers are detected
        if len(self.detected_markers) < TOTAL_MARKERS:
            self.rotate()
        else:
            # Once all markers are detected, switch to Phase 2
            self.stop()
            rospy.loginfo("All markers detected. Starting ordered search.")
            self.phase = 2
            # Sort marker IDs for ordered search
            self.sorted_ids = sorted(self.detected_markers)  

    def search_markers(self, ids, corners, image_np):
        '''Phase 2: Search for markers in ascending order, highlight them and publish them'''
        # If there are still markers to publish,
        if self.count_published_markers < len(self.sorted_ids):
            # Get current target ID
            target_id = self.sorted_ids[self.count_published_markers]  
            
            # If the target ID is in the image
            if ids is not None and target_id in ids:
                # Get the index of the target ID
                idx = np.where(ids == target_id)[0][0]  

                # Get the corners of the detected marker
                corner = corners[idx]  
                
                # Draw a red circle around the detected marker
                self.highlight_marker(corner, target_id, image_np)

                # Publish the image with the highlighted marker
                self.publish_image(image_np)

                # Move to the next marker in the order
                rospy.loginfo(f"Marker {target_id} published. Moving to the next one...")
                self.count_published_markers += 1
            
            else:
                # Rotate to continue searching for the target marker
                self.rotate()
        
        else:
            # Stop after all markers are processed
            self.stop()
            rospy.loginfo("All markers processed. Task done :D")

    def highlight_marker(self, corner, target_id, image_np):
        '''Draw a red circle around the detected marker'''
        # Calculate the center of the marker
        c_x = int(np.mean(corner[0][:, 0]))
        c_y = int(np.mean(corner[0][:, 1]))

        # Calculate the radius of the circle based on marker size
        radius = int(np.linalg.norm(corner[0][0] - corner[0][1]) / 2)

        # Draw the circle on the image
        cv2.circle(image_np, (c_x, c_y), radius, (0, 0, 255), 2)

        # Add ID number as text on image
        cv2.putText(image_np, str(target_id), 
                    (c_x + 10, c_y + 10),            # Position
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,   # Font ans size
                    (0, 0, 255), 2)                  # Color and thickness

    def publish_image(self, image_np):
        '''Publish the image with the highlighted marker to the ROS topic'''
        try:
            # Convert Cv Image into a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(image_np, "bgr8")  

            # Publish to topic
            self.aruco_pub.publish(ros_image)  

        except CvBridgeError as error:
            rospy.loginfo(f"Error publishing image: {error}")

    def rotate(self):
        '''Rotate the camera by publishing angular velocity'''
        vel = Float64()
        vel.data = -1.0  # Rotate to the right
        self.vel_pub.publish(vel)

    def stop(self):
        '''Stop the camera by publishing zero velocity'''
        vel = Float64()
        vel.data = 0.0  # Stop
        self.vel_pub.publish(vel)

def main(args):
    '''Main function to initialize the node and start processing'''
    md = MarkerDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Marker Detector node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
