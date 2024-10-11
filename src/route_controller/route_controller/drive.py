import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .navi import Navi
import time
import math


from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from .robot_navigator import BasicNavigator # Helper module
from .util import Servo, Gripper, ServoControl
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from rtk_camera_interfaces.srv import CameraCom
from geometry_msgs.msg import Twist 


class DeltaNode(Node):
    def __init__(self):
        super().__init__('delta')
        self.subscription1 = self.create_subscription(
            Int16, 'camera/marker_error',   
            self.listener_callback,
            10)
      
        self.subscription1
        self.error = 0

    def listener_callback(self, msg):
        self.error = msg.data
        self.get_logger().info('ERROR ' + str(self.error))
    
    def get_error(self):
        return self.error



class RobotUtil(Node):
    def __init__(self):
        super().__init__('driver')
        self.subscription1 = self.create_subscription(
            Int16, 'camera/marker_error',   
            self.listener_callback,
            10)
        self.subscription1 = self.create_subscription(
            Int64, 'camera/marker_size',   
            self.listener_size_callback,
            10)
        self.error = 0
        self.subscription1

        self.FLAG = False
        self.FLAG2 = False
        
        self.publisher_twist = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        self.cli = self.create_client(CameraCom, 'camera_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CameraCom.Request()

    

    def send_request(self, command):
        self.req.command = command
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def start_request(self):
        ans = self.send_request("Start")
        self.get_logger().info('target ArUco: ' + ans.result)
    
    def begin_targeting(self):
        ans = self.send_request("Catch")
        self.get_logger().info('target ArUco: ' + ans.result)


    def listener_callback(self, msg):
        if self.FLAG:
            self.error = msg.data
            if abs(self.error) > 5:
                self.send_msg_rot()
                self.get_logger().info('ERROR ' + str(-self.error))
            else: 
                self.FLAG = False
                self.get_logger().info('ERROR Complete')
        
        else:
            self.get_logger().info('not start ERROR ' + str(-self.error))
        
    def listener_size_callback(self, msg):
        if self.FLAG2:
            self.error = msg.data
            if abs(self.error) < 11000:
                self.send_msg_for()
                self.get_logger().info('Size ' + str(-self.error))
            else: 
                self.FLAG2 = False
                self.get_logger().info('ERROR Complete')
                self.send_msg_stop()
        
        else:
            self.get_logger().info('not start ERROR ' + str(-self.error))
            

    def send_msg_rot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -self.error * 0.005
        # self.get_logger().info('DELTA ' + str(self.error * 0.01))
        self.publisher_twist.publish(msg)
    
    def send_msg_for(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_twist.publish(msg)
    
    def send_msg_back(self):
        msg = Twist()
        msg.linear.x = -0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_twist.publish(msg)
    
    
    def send_msg_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_twist.publish(msg)

    
    def publish(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
    

    def move_to_aruco(self):
        # self.begin_targeting()
        while abs(self.error) >= 5:
            rclpy.Rate(100).sleep()

            
        self.send_msg_stop()
        pass


def main(args=None):
    rclpy.init(args=args)


    RU = RobotUtil()
    RU.start_request()

    # time.sleep(2)
    # RU.begin_targeting()

    
    # time.sleep(2)
    # RU.FLAG = True

    # while RU.FLAG == True:
    #     rclpy.spin_once(RU)
 

    # time.sleep(2 )

    back = [0.0, 0.0,  0.71, 0.71]
    right = [0.0, 0.0, -0.71, 0.71]
    forw = [0.0, 0.0, 3.14, 0.0]
    left = [0.0, 0.0, 1.0, 1.0]


    navigator = BasicNavigator()

    # # navi = Navi()
    # # navi.publish(0.0, 0.0, 0.0)
    # navigator.waitUntilNav2Active()

    time_ = navigator.get_clock().now().to_msg()








        # ### first arrow
    # goal_pose = Navi.set_goal_pose(0.0, 0.0, 0.0, time_)
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass
    # print("0")
    # time.sleep(2)
    # goal_pose = Navi.set_goal_pose(0.0, 0.0, 3.14, time_)
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass
    # print("0")
    # time.sleep(2)

    #     # ### first arrow
    # goal_pose = Navi.set_goal_pose(0.0, 0.0, -3.14/2, time_) ##right
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass
    # print("0")
    # time.sleep(2)
    # goal_pose = Navi.set_goal_pose(0.0, 0.0, 3.14/2, time_)
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass
    # print("0")
    # time.sleep(2)





    ### first arrow
    goal_pose = Navi.set_goal_pose(-0.04, 0.58, -3.14/2, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)


    ###  arrow2

    goal_pose = Navi.set_goal_pose(0.56, 0.79, 0, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

    ###  arrow3

    goal_pose = Navi.set_goal_pose(0.77, 0.34, -3.14/2, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

    ##  arrow4

    goal_pose = Navi.set_goal_pose(1.27, 0.15, 0.0, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)


     ##  arrow5

    goal_pose = Navi.set_goal_pose(1.6, 1.28, math.pi/2, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

     ##  finish
    goal_pose = Navi.set_goal_pose(0.4, 2.28, 0.0, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

    goal_pose = Navi.set_goal_pose(1.5, 2.28, 0.0, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

    RU.begin_targeting()
    time.sleep(2)
    RU.FLAG = True
    while RU.FLAG == True:
        rclpy.spin_once(RU)
    time.sleep(2)

    time.sleep(2)
    RU.FLAG2 = True
    while RU.FLAG2 == True:
        rclpy.spin_once(RU)
    time.sleep(2)

    RU.send_msg_back()
    time.sleep(2)
    RU.send_msg_stop()



    goal_pose = Navi.set_goal_pose(1.27, 0.15, 0.0, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)
    
    goal_pose = Navi.set_goal_pose(0.77, 0.34, -math.pi/2, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

       ### first arrow
    goal_pose = Navi.set_goal_pose(0.0, 0.0, math.pi, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("0")
    time.sleep(0.5)

    rclpy.shutdown()

    

if __name__ == '__main__':
    main()