import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf2_ros


class ArUcoTF(Node):
    '''
    Class to detect ArUco markers and publish their transforms.
    '''

    def __init__(self):
        super().__init__('aruco_tf_publisher')  # Registering node

        ############ Topic SUBSCRIPTIONS ############
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############
        image_processing_rate = 0.2  # Process image every 0.2 seconds
        self.bridge = CvBridge()  # Initialize CvBridge object
        self.tf_buffer = tf2_ros.Buffer()  # Buffer for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)  # Transform broadcaster
        self.timer = self.create_timer(image_processing_rate, self.process_image)  # Timer for image processing

        self.cv_image = None  # Color raw image variable
        self.depth_image = None  # Depth image variable

    def depthimagecb(self, data):
        '''Callback function for depth camera topic.'''
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = np.uint8(depth_display)

            cv2.imshow("Depth Image", depth_display)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {str(e)}")

    def colorimagecb(self, data):
        '''Callback function for color camera raw topic.'''
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image = cv2.flip(self.cv_image, 0)  # Flip image vertically

            cv2.imshow("Color Image", self.cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {str(e)}")

    def process_image(self):
        '''Process the images received from color and depth callbacks.'''
        if self.cv_image is None or self.depth_image is None:
            return

        # Call the ArUco detection function
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = self.detect_aruco(self.cv_image)

        # Publish TFs for detected ArUco markers
        if ids is not None:
            for i in range(len(ids)):
                self.publish_tf(center_aruco_list[i], distance_from_rgb_list[i], angle_aruco_list[i], ids[i])

    def detect_aruco(self, image):
        '''Function to perform ArUco detection.'''
        aruco_area_threshold = 1500
        cam_mat = np.array([[931.1829833984375, 0.0, 640.0],
                            [0.0, 931.1829833984375, 360.0],
                            [0.0, 0.0, 1.0]])
        dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        size_of_aruco_m = 0.15

        center_aruco_list = []
        distance_from_rgb_list = []
        angle_aruco_list = []
        width_aruco_list = []
        ids = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is None:
            return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        for i in range(len(ids)):
            marker_corners = corners[i][0]
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            center_aruco_list.append((center_x, center_y))

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)
            distance = np.linalg.norm(tvec[0][0])
            distance_from_rgb_list.append(distance)

            angle = np.degrees(np.arctan2(tvec[0][0][1], tvec[0][0][0]))
            angle_aruco_list.append(angle)

            width = np.linalg.norm(marker_corners[0] - marker_corners[1])
            width_aruco_list.append(width)

            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)

        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

    def publish_tf(self, center, distance, angle, marker_id):
        '''Publish the TF for detected ArUco markers.'''
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_frame"  # Adjust this based on your TF frame setup
        transform.child_frame_id = f"aruco_marker_{marker_id}"
        transform.transform.translation.x = center[0]
        transform.transform.translation.y = center[1]
        transform.transform.translation.z = distance
        transform.transform.rotation = tf_transformations.quaternion_from_euler(0, 0, np.radians(angle))

        self.br.sendTransform(transform)


def main():
    rclpy.init()
    node = ArUcoTF()  # Initialize your node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
