#!/usr/bin/env python
import rospy
#import scipy
from std_msgs.msg import Float32MultiArray


def callback(data):
	rospy.loginfo("blabla")


    def LP_solve_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('LP_solve_node', anonymous=True)

    rospy.Subscriber("/array", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    LP_solve_node()
