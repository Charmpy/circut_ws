import cv2
import numpy as np
import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int64

from .marker_f import Finder
from rtk_camera_interfaces.srv import CameraCom
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge 
# path = "/dev/v4l/by-path/pci-0000:00:0c.0-usb-0:2:1.0-video-index0" # action

class CaptureCameraNode(Node):
    def __init__(self, finder):         #    def __init__(self, cap, finder):
        super().__init__('capture_camera')

        self.finder = finder
        
        # self.cap = cap

        self.target_id = -1

        self.catch_flag = False

        self.frame = 0
        self.last_frame = 0

        self.text_publisher = self.create_publisher(Int16, 'camera/marker_error', 10)
        self.scale_publisher = self.create_publisher(Int64, 'camera/marker_size', 10)
        # print("Camera created", self.cap)
        # self.streamer = self.create_timer(0.005, self.streamer)
        self.timer = self.create_timer(0.01, self.marker_position)

        self.srv = self.create_service(CameraCom, 'camera_command', self.command_callback)

        self.subscription = self.create_subscription(Image, 'image_raw/uncompressed', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        self.frame = current_frame
        # Display image
        if self.catch_flag:
            cv2.imshow("camera", self.last_frame)
        else:
            cv2.imshow("camera", self.frame )
        cv2.waitKey(1)
        # def streamer(self):
        #     # ret, frame = self.cap.read()



        # if ret:
        #     cv2.imshow("frame", frame)
        #     key = cv2.waitKey(1)
        #     if key == ord("q"):
        #         cv2.destroyAllWindows()
        #         raise SystemExit


    def command_callback(self, request, response):
        # ret, frame = self.cap.read()
        if request.command == "Start":
            self.target_id = self.finder.get_target_marker(self.frame)
            response.result = str(self.target_id)
        elif request.command == "Catch":
            self.catch_flag = True
            response.result = "Start tracking"
        return response
    
    def marker_position(self):
        # ret, frame = self.cap.read()
        if self.catch_flag:
            h, w = self.frame.shape[:2]
            # cv2.imshow("frame", frame)
            # key = cv2.waitKey(1)
            # if key == ord("q"):
            #     cv2.destroyAllWindows()
            #     raise SystemExit
            gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            try:
                msg = Int16()
                msg2 = Int64()
                marker, marker_size = self.finder.get_marker_pos(gray_frame, self.frame, self.target_id) 
                # self.get_logger().info('Marker: "%s"' % marker)
                # cv2.imshow("camera", self.frame)
                self.last_frame = self.frame
                # cv2.waitKey(1)

                if marker == None:
                    self.get_logger().info('Marker not found')
                else:
                    pose = marker[0] - w//2
                    msg.data = pose
                    # self.get_logger().info('Publishing: "%s"' % msg.data)
                    self.text_publisher.publish(msg)
                    msg2.data = marker_size 
                    self.get_logger().info('Publishing size: "%s"' % msg2.data) 
                    self.scale_publisher.publish(msg2)

            except Exception as e:
                self.get_logger().info('ERROR: "%s"' % e)




def main(args=None):
    
    # cap = cv2.VideoCapture(path, cv2.CAP_V4L)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)

    finder = Finder()    
     # Инициализируем ROS
    rclpy.init(args=args)

    capture_camera = CaptureCameraNode(finder)

    try:
        rclpy.spin(capture_camera)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    
    capture_camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()