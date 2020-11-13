import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Pose2D
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import numpy as np
import os
import cv2
import sys

from env import launch_env


class ROSAgent(Node):
    def __init__(self):
        super().__init__("duckietown_agent")

#        self.node = rclpy.create_node("duckietown_agent")

        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv('HOSTNAME')

        # Use our env launcher
        self.env = launch_env()

        # Subscribes to the output of the lane_controller_node and IK node
        self.action_sub = self.create_subscription(Twist2DStamped,
                                                   '/{}/lane_controller_node/car_cmd'.format(self.vehicle),
                                                   self._action_cb, 10)
        self.ik_action_sub = self.create_subscription(WheelsCmdStamped,
                                                      '/{}/wheels_driver_node/wheels_cmd'.format(self.vehicle),
                                                      self._ik_action_cb, 10)

        # Place holder for the action
        self.action = np.array([0, 0])

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        self.cam_pub = self.create_publisher(CompressedImage,
                                             '/{}/camera/image/compressed'.format(self.vehicle), 10)

        # Publisher for camera info - needed for the ground_projection
        self.cam_info_pub = self.create_publisher(CameraInfo,
                                                  '/{}/camera/camera_info'.format(self.vehicle), 10)

        self.pose_pub = self.create_publisher(Pose2D,
                                             '/{}/pose'.format(self.vehicle), 10)                                                

        # Initialize timer for continuous publication of camera images and info
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    #        # Initializes the node
    #        rospy.init_node('ROSAgent')
    #
    #        # 10Hz ROS Cycle - TODO: What is this number?
    #        self.r = rospy.Rate(10)

    def timer_callback(self):
        """
        Timer callback for publication of camera data
        """
        img, r, d, misc = self.env.step(self.action)
        self._publish_img(img)
        pose = Pose2D()
        pose.x = misc['Simulator']['cur_pos'][0]
        pose.y = misc['Simulator']['cur_pos'][2]
        pose.theta = misc['Simulator']['cur_angle']
        self.pose_pub.publish(pose)

    def _action_cb(self, msg):
        """
        Callback to listen to last outputted action from lane_controller_node
        Stores it and sustains same action until new message published on topic
        """
        v = msg.v
        omega = msg.omega

    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from lane_controller_node
        Stores it and sustains same action until new message published on topic
        """
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])

    def _publish_info(self, img):
        """
        Publishes a default CameraInfo - TODO: Fix after distortion applied in simulator
        """
        
        ci = CameraInfo()
        ci.header.stamp = img.header.stamp
        ci.header.frame_id = img.header.frame_id

        ci.width = self.env.camera_width
        ci.height = self.env.camera_height
        
        # taken from
        # duckietown/Software/blob/master19/catkin_ws/src/07-image-processing/pi_camera/include/pi_camera/camera_info.py
        ci.k = [
            305.5718893575089, 0.0, 303.0797142544728,
            0.0, 308.8338858195428, 231.8845403702499,
            0.0, 0.0, 1.0
        ]

        # taken from
        # duckietown/Software/blob/master19/catkin_ws/src/07-image-processing/pi_camera/include/pi_camera/camera_info.py
        ci.d = [-0.2944667743901807, 0.0701431287084318, 0.0005859930422629722, -0.0006697840226199427, 0.0]

        # taken from
        # duckietown/Software/blob/master19/catkin_ws/src/07-image-processing/pi_camera/include/pi_camera/camera_info.py
        ci.p = [
            220.2460277141687, 0.0, 301.8668918355899, 0.0,
            0.0, 238.6758484095299, 227.0880056118307, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.cam_info_pub.publish(ci)

    def _publish_img(self, obs):
        """
        Publishes the image to the compressed_image topic, which triggers the lane following loop
        """
        img_msg = CompressedImage()

        time = self.get_clock().now()

        img_msg.header.stamp.sec, img_msg.header.stamp.nanosec = time.seconds_nanoseconds()
#        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        contig = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)
        img_msg.data = np.array(cv2.imencode('.jpg', contig)[1]).tostring()

        self.cam_pub.publish(img_msg)
        
        self._publish_info(img_msg)
    #    def spin(self):


#        """
#        Main loop
#        Steps the sim with the last action at rate of 10Hz
#        """
#        while not rospy.is_shutdown():
#            img, r , d, _ = self.env.step(self.action)
#            self._publish_img(img)
#            self._publish_info()
#            self.r.sleep()


def main(args=None):
    rclpy.init(args=args)

    ros_agent = ROSAgent()

    rclpy.spin(ros_agent)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# r = ROSAgent()
# r.spin()
#
