#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

    






def run():
    joint1pub = rospy.Publisher('/j2n6s300/joint_1_position_controller/command', Float64, queue_size=10)
    joint2pub = rospy.Publisher('/j2n6s300/joint_2_position_controller/command', Float64, queue_size=10)
    joint3pub = rospy.Publisher('/j2n6s300/joint_3_position_controller/command', Float64, queue_size=10)
    joint4pub = rospy.Publisher('/j2n6s300/joint_4_position_controller/command', Float64, queue_size=10)
    joint5pub = rospy.Publisher('/j2n6s300/joint_5_position_controller/command', Float64, queue_size=10)
    joint6pub = rospy.Publisher('/j2n6s300/joint_6_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    R = rospy.Rate(1) # tci sends 30hz
    while not rospy.is_shutdown():
        messagejoint1 = Float64()
        messagejoint2 = Float64()
        messagejoint3 = Float64()
        messagejoint4 = Float64()
        messagejoint5 = Float64()
        messagejoint6 = Float64()

        for x in range(10):
            messagejoint1.data = x
            messagejoint2.data = x
            messagejoint3.data = x
            messagejoint4.data = x
            messagejoint5.data = x
            messagejoint6.data = x

            joint1pub.publish(messagejoint1)
            joint2pub.publish(messagejoint2)
            joint3pub.publish(messagejoint3)
            joint4pub.publish(messagejoint4)
            joint5pub.publish(messagejoint5)
            joint6pub.publish(messagejoint6)
            R.sleep()
        for x in range(10, 0, -1):
            messagejoint1.data = x
            messagejoint2.data = x
            messagejoint3.data = x
            messagejoint4.data = x
            messagejoint5.data = x
            messagejoint6.data = x

            joint1pub.publish(messagejoint1)
            joint2pub.publish(messagejoint2)
            joint3pub.publish(messagejoint3)
            joint4pub.publish(messagejoint4)
            joint5pub.publish(messagejoint5)
            joint6pub.publish(messagejoint6)
            R.sleep()
if __name__ == '__main__':
   
    run()
    