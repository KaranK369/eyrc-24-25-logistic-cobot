import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from tf_transformations import quaternion_from_matrix

#Calculate Area of all Aruco markers in the image
def calculate_rectangle_area(coordinates):
    """*Making assumption that aruco is square and
        calculating width by taking any consecutive points
       
       *Format of coordinates is [[x1,y1], [x2, y2],., l]"""
    area = None
    width = None
    
    point_1 = coordinates[0]
    point_2 = coordinates[1]
    width = math.sqrt((point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2)
    area = width**2

    return area, width

# Calculate the quanterion components
def euler_from_quaternion(quatern):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = quatern[0]
        y = quatern[1]
        z = quatern[2]
        w = quatern[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in radians
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk


    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    # if (-30 < ak <30):
    #     q[1] += 0.8
        
    # else:
    #     q[0] += 1
    #     q[1] += 0.0

    mag = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    for i in range(4):
        q[i] /= mag



    return q

def quaternion_multiplication(q1,q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2

    q3 = [0,0,0,0]
    q3[3] = w1*w2 - x1*x2-y1*y2-z1*z2
    q3[0] = w1*x2 +x1*w2 +y1*z2-z1*y2
    q3[1] = w1*y2 - x1*z2 + y1*w2 + z1*x2
    q3[2] = w1*z2 + x1*y2 -y1*x2+z1*w2

    return q3

    

def detect_aruco(image):
    aruco_area_threshold = 1500  # threshold value to detect aruco markers of certain size
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0],
                       [0.0, 931.1829833984375, 360.0],
                       [0.0, 0.0, 1.0]])  # camera matrix
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # distortion matrix
    size_of_aruco_m = 0.15  # size of aruco marker    

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Need to Review
    if ids is None:
    
            return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids
    
    cv2.aruco.drawDetectedMarkers(image, corners, ids)

    #Calculating the Area of eachs Aruco Marker
    areas = []

    
    for i,coordinate in enumerate(corners):
        Area, Width = calculate_rectangle_area(coordinate[0])
        areas.append(Area)
        width_aruco_list.append(Width)

        #Calculate the centre of each aruco marker
        center_x = int(np.mean(coordinate[0][:, 0]))
        center_y = int(np.mean(coordinate[0][:, 1]))
        center_aruco_list.append((center_x, center_y))

        #Calculate the distance of each aruco marker form RGB camera
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(coordinate, size_of_aruco_m, cam_mat, dist_mat)
        distance = np.sqrt(tvec[0][0][0]**2 + tvec[0][0][1]**2 + tvec[0][0][2]**2)
        distance_from_rgb_list.append(distance)

        # Calculate the angle
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        angles = r.as_euler('xyz', degrees=True)
        angle_aruco_list.append(angles[2])
        cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)

    cv2.imshow('detect aruco',image)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


class aruco_tf(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None 

    def depthimagecb(self, data):
        try:
            # Convert ROS Image message to OpenCV image type (32-bit floating point)
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

            # Optional: Apply some processing to the depth image if needed
            # For example, normalization for visualization
            depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = np.uint8(depth_display)

            # Display the depth image
            cv2.imshow("Depth Image", depth_display)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {str(e)}")

    def colorimagecb(self, data):
        try:
            # Convert ROS Image message to OpenCV image type (BGR format)
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Optional: Image flipping or rotation if necessary based on camera setup
            # For this example, flip the image vertically
            #self.cv_image = cv2.flip(self.cv_image, 0)

            # Uncomment the following line if the camera requires rotation
            # self.cv_image = cv2.rotate(self.cv_image, cv2.ROTATE_90_CLOCKWISE)

            # Display the color image for visualization
            cv2.imshow("Color Image", self.cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {str(e)}")
    
    def broadcast_timer_callback(self, abs_angle, angle, x, y, z, id):

        # quanterion_comp = R.from_euler('xyz', [0, 0, angle], degrees=True)#euler_to_quaternion(0, 0, -angle)
        # quanterion_comp = quanterion_comp.as_quat()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()                        # select transform time stamp as current clock time
        # frame IDs
        t.header.frame_id = 'camera_link'                                         # parent frame link with whom to send transform
        t.child_frame_id = f"cam_{id[0]}"                                              # child frame link from where to send transfrom
        # translation
        t.transform.translation.x = x
        t.transform.translation.y = y                                    # distance offset in Y axis of 2 units
        t.transform.translation.z = z


        #Applying offsets
        quat = quaternion_from_euler(float(), float(), float(math.radians(angle)))
        quat_abs = quaternion_from_euler(float(),float(), float(math.radians(-abs_angle)))
        



        quat = quaternion_multiplication(quat_abs,quat)
        
        quat15 = quaternion_from_euler(float(),float(math.radians(-15)), float(0))

        
        #rotation about x axis to bring it to the correct orientation    
        quat = quaternion_multiplication(quat15,quat)
        
        if abs(abs_angle) > 60:
            quat15 = quaternion_from_euler(float(), float(0),float(math.radians(((abs_angle)/abs(abs_angle))*15))) 
        #rotation about x axis to bring it to the correct orientation    
            quat = quaternion_multiplication(quat,quat15)

            quat15 = quaternion_from_euler(float(), float(math.radians(0)),float(math.radians(-abs_angle))) 
        #rotation about x axis to bring it to the correct orientation    
            quat = quaternion_multiplication(quat,quat15)

        quat_90 = quaternion_from_euler(float(math.radians(90)), float(), float())
        quat = quaternion_multiplication(quat, quat_90)

        quat_90 = quaternion_from_euler(float(), float(math.radians(90)), float())
        quat = quaternion_multiplication(quat, quat_90)

        t.transform.rotation.x = quat[0]   
        
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        self.get_logger().warn(f"{id}")
        self.get_logger().warn(f"{t.transform.rotation}")
        self.br.sendTransform(t) 

        to_frame_rel = "base_link"
        from_frame_rel = f"cam_{id[0]}"
        #self.listener.waitForTransform('camera_link', f"cam_{id}", rclpy.time.Time(0), rclpy.time.Duration(4.0))
        try:
            t = self.tf_buffer.lookup_transform( to_frame_rel, from_frame_rel, rclpy.time.Time())       # look up for the transformation between 'obj_1' and 'base_link' frames
            self.get_logger().info(f'Successfully received data!')
        except tf2_ros.TransformException as e:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
            return
        # Logging transform data...
        t.header.stamp = self.get_clock().now().to_msg()                        # select transform time stamp as current clock time
        # frame IDs
        t.header.frame_id = 'base_link'                                         # parent frame link with whom to send transform
        t.child_frame_id = "obj_"+str(id[0])  

        self.br.sendTransform(t) 


    
    def process_image(self):

        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        if self.cv_image is None or self.depth_image is None:
            return
        
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)
        
  
        if ids is None:
            self.get_logger().warn("Aruco Markers Not Detected")
            return
        for ind,i in enumerate(ids):
            angle_aruco = (0.788*angle_aruco_list[ind]) - ((angle_aruco_list[ind]**2)/3160)
            # x = (distance_from_rgb_list[ind]) * (sizeCamX - center_aruco_list[ind][0] - centerCamX) / focalX
            # y = (distance_from_rgb_list[ind]) * (sizeCamY - center_aruco_list[ind][1] - centerCamY) / focalY
            # z = (distance_from_rgb_list[ind])
            # 
            print(f"After correction{angle_aruco}")
            depth = self.depth_image[center_aruco_list[ind][1], center_aruco_list[ind][0]] / 1000

            x = (depth) * (sizeCamX - center_aruco_list[ind][0] - centerCamX) / focalX
            y = (depth) * (sizeCamY - center_aruco_list[ind][1] - centerCamY) / focalY
            z = (depth)     

            self.broadcast_timer_callback(angle_aruco_list[ind], angle_aruco,z, x, y,i)


def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()