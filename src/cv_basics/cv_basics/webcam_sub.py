# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from req_res_str_service.srv import ReqRes
from time import sleep


import torch
import numpy as np
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'image_raw/uncomp', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.srv = self.create_service(ReqRes, 'cam_service', self.cam_serv_callback)
    print(12345)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.index = 0

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    parameters = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)


    self.model = torch.hub.load('src/cv_basics/cv_basics/yolov5', 'custom', path='/home/pentagon/circut_ws/src/cv_basics/cv_basics/best_sign.pt', source='local', force_reload=True)
    self.model.conf = 0.9
    self.current_frame = None

  def cam_serv_callback(self, request, response):
    if self.current_frame is not None:
      self.current_frame = cv2.flip(self.current_frame, 0)
      self.current_frame = cv2.flip(self.current_frame, 1)
      marker_corners, marker_IDs, reject = self.detector.detectMarkers(self.current_frame)
      if marker_corners:
          for ids in marker_IDs:
            if ids[0] == 20:
              response.res = 'a'
              return response


      img = np.array(self.current_frame)
      pred = self.model(img)

      crops = pred.crop(save=False)

      if len(crops) == 1:
        sign = crops[0]
        # print()
        if "right" in sign['label']:
          response.res = 'r'
        elif "left" in sign['label']:
          response.res = 'l'
        else:
          response.res = 'f'
      else:
        response.res = "N"
              #### РЕЗУЛЬТАТ КОМПЬЮТЕРНОГО ЗРЕНИЯ ПОМЕЩАТь СЮДА
      self.get_logger().info(f"{request.req}")
    else:
        response.res = "Image not found"
    return response

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    self.current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    cv2.imshow("camera", self.current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()