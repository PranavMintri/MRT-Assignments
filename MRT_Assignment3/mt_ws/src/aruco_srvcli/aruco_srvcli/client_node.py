#!/usr/bin/env python3
import gdown # This library is for automatic downloading and processing of Google drive links.
import sys
import os
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_interface.srv import SendImage

class ClientNode(Node):
    def __init__(self, video_source): # to pass an argument into the client, we have added the parameter video_source in the function
        super().__init__('client_node')
        
        # Setup Service Client
        self.client = self.create_client(SendImage, 'send_image')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ArUco Server...')
        

        if video_source.isdigit():
            # If it's a digit (e.g., "0"), convert to int.
            source = int(video_source)
        elif "drive.google.com" in video_source:
            # If it is a drive link, download it.
            source = self.download_from_drive(video_source)
        else:
            # This is if it's a local video.
            source = video_source

        self.cap = cv2.VideoCapture(source)
        # Instead of directly opening the webcam, above, we have tried to pass an argument into the client.
        # self.cap = cv2.VideoCapture(0) # Open webcam.
        
        self.bridge = CvBridge()

        # Timer to drive the capture loop (approx 30 FPS)
        self.timer = self.create_timer(0.03, self.process_frame)
        self.get_logger().info("Client started.")

    def download_from_drive(self, url):
        # Create a temporary filename.
        output = 'temp_drive_video.mp4'
        # gdown handles the extraction of the file ID automatically.
        gdown.download(url, output, quiet=False, fuzzy=True)
        # fuzzy=True allows the script to extract the FILE_ID even if you provide the full "view" link rather than the direct download link.
        return output

    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            return

        # Convert OpenCV image to ROS 2 message. For this, AI was used by me.
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Prepare and send request.
        request = SendImage.Request()
        request.image_data = ros_image

        # We use call_async so the UI (imshow) doesn't freeze.
        future = self.client.call_async(request)
        future.add_done_callback(lambda f: self.response_callback(f, frame))

    def response_callback(self, future, frame):
        try:
            response = future.result()
            
            # Process the nested response.
            if response.ids:
                self.get_logger().info(f"IDs found: {response.ids}")
                # Loop through each detection.
                for i in range(len(response.ids)):
                    marker_id = response.ids[i]
                    
                    # Unpack the nested BoundingBox.msg.
                    # Reconstruct from flat list [8] to NumPy array [4, 2]. Once again, for this Python-libraries related stuff, I used AI.
                    corners = np.array(response.bboxs[i].corners).reshape((4, 2)).astype(np.int32)
                    # Printing the coordinates of the bounding boxes:
                    print(f"\nMarker ID: {marker_id}")
                    print(f"  Top-Left:     ({corners[0][0]:.2f}, {corners[0][1]:.2f})")
                    print(f"  Top-Right:    ({corners[1][0]:.2f}, {corners[1][1]:.2f})")
                    print(f"  Bottom-Right: ({corners[2][0]:.2f}, {corners[2][1]:.2f})")
                    print(f"  Bottom-Left:  ({corners[3][0]:.2f}, {corners[3][1]:.2f})")
                    
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    video_source = sys.argv[1] if len(sys.argv) > 1 else '0'
    node = ClientNode(video_source)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        # Clean up the download file if it exists.
        if os.path.exists('temp_drive_video.mp4'):
            os.remove('temp_drive_video.mp4')
            # This ensures that the large video file is deleted from the device once the ROS2 node is closed.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
