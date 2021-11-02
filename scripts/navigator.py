#!/bin/python3
import rospy
import tf
from geometry_msgs.msg import Twist
import threading
import sophus as sp
from apriltag_ros.msg import AprilTagDetectionArray
from helper import pose2poselist, transformPose, matrix_from_xyzquat  

class ApriltagNavigator():
    # subcribe the pose message from /tag_detections and publish the velocity to /cmd_vel 
    def __init__(self, constant_vel = False):
        self.lr = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.w = []
        

        self.apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)


        self.thread = threading.Thread(target = self.constant_vel_loop)
        self.thread.start()
        
        rospy.sleep(1)

    # apriltag msg handling function
    def apriltag_callback(self,data):
        # use apriltag pose detection to find where is the robot
        for detection in data.detections:
            poselist_tag_cam = pose2poselist(detection.pose.pose.pose)
            poselist_tag_base = transformPose(self.lr, poselist_tag_cam, sourceFrame = '/camera_rgb_optical_frame', targetFrame = '/base_footprint')
            self.init_pos = poselist_tag_base
            self.tag_detections_topic_in_LIST = poselist_tag_cam
            print(" pose in the camera’sbody-centric coordinate frame")
            print(poselist_tag_cam)


    # sending constant velocity
    def constant_vel_loop(self):

        pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rate = rospy.Rate(10)
        wcv = Twist()
        rospy.sleep(2)

        U = self.w

        wcv.linear.x = U[0]
        wcv.linear.y = U[1]
        wcv.linear.z = U[2]
        wcv.angular.x = U[3]
        wcv.angular.y = U[4]
        wcv.angular.z = U[5]
        rospy.sleep(1)
        
        self.velcmd_pub.publish(wcv)
        
        print("cmd_vel start")
        rospy.sleep(10)
        wcv.linear.x=wcv.linear.y=wcv.linear.z=0.0
        wcv.angular.x=wcv.angular.y=wcv.angular.z=0.0
        print("cmd_vel finished")
        self.velcmd_pub.publish(wcv)
        



def main():
    # initial the node
    rospy.init_node('apriltag_navi',anonymous=True)
    apriltag_navi = ApriltagNavigator()

    # the initial pose (matrix format)
    init_pos_matrix = matrix_from_xyzquat(apriltag_navi.init_pos)

    # target pose (matrix format)
    target_pos = [0.12, 0.0 , apriltag_navi.init_pos[2], 0, 1, 1, 0 ]
    target_pos_matrix = matrix_from_xyzquat(target_pos)

    # using pophus to get pose in SE3 format
    X = sp.SE3(init_pos_matrix)  
    Y = sp.SE3(target_pos_matrix)
    # using the formula in 1(f)
    # Y=X*exp(Tw)
    X_1 = X.inverse()
    YX_1 = X_1*Y

    # log(Y*X^1) = Tw
    Tw = YX_1.log()

    # T = 10s
    w = Tw/10
    print("T=10s,Ω:")
    print(w)
    apriltag_navi.w = w 


    rospy.sleep(1)
    rospy.spin()

    
if __name__=='__main__':
    main()