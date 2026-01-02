#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
import os
from cv_bridge import CvBridge
from aruco_interface.msg import BoundingBox
from aruco_interface.srv import SendImage
# We are importing cv_bridge to convert the data received from the client into an image.

# Get the image from the client.
# Convert the image to grayscale.
# Get the dictionary.
# Create detector parameters.
# Create the detector object.
# Store bboxs, ids, rejected.
# Append bboxs and ids to the ids and bboxs arrays.
# Send bboxs and ids to the client.

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        # Create the service
        self.srv = self.create_service(SendImage, 'send_image', self.aruco_callback)
        self.bridge = CvBridge()
        # self.get_logger().info("ArUco Service Server is ready.")

    def aruco_callback(self, request, response):
        # Get the image from the client and convert to OpenCV format.
        cv_img = self.bridge.imgmsg_to_cv2(request.image_data, desired_encoding='bgr8')

        # Convert the image to grayscale.
        imggray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # Get the dictionary (Using OpenCV 4.7+ syntax).
        # Assuming 4x4 markers with 50 total markers.
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Create detector parameters.
        parameters = aruco.DetectorParameters()

        # Create the detector object.
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Store bboxs, ids, rejected.
        bboxs, ids, rejected = detector.detectMarkers(imggray)

        # Prepare lists for the response.
        ros_ids = []
        ros_bboxes = []

        # Append bboxs and ids to the arrays if any are detected.
        if ids is not None:
            # ids is a 2D numpy array [[id1], [id2]], flatten it to [id1, id2]. For this Python libraries related stuff, AI was used by me.
            ros_ids = ids.flatten().astype(int).tolist()

            for marker_corner in bboxs:
                # Each 'marker_corner' is shape (1, 4, 2).
                new_bbox = BoundingBox()
                # Flatten the (1, 4, 2) into a 1D list of 8 floats.
                new_bbox.corners = marker_corner.flatten().astype(float).tolist()
                ros_bboxes.append(new_bbox)

        # Send bboxs and ids to the client.
        response.ids = ros_ids
        response.bboxs = ros_bboxes
        
        self.get_logger().info(f"Detected {len(ros_ids)} markers.")
        return response

def main():
    rclpy.init()
    node = ServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
