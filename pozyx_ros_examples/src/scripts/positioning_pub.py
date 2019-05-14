#!/usr/bin/env python
"""ROS node that performs 3D positioning on Pozyx"""

import pypozyx
import rospy
from geometry_msgs.msg import PointStamped


def pozyx_positioning_pub():
    rospy.init_node('positioning_pub')

    # remote_id = rospy.get_param("~tag_id")
    tags_list = rospy.get_param("~tags")
    # remote_id = None if remote_id == "None" else int(remote_id, 16)
    frequency = rospy.get_param("~frequency")

    for i in range(len(tags_list)):
        if tags_list[i] == 'None':
            tags_list[i] = None
    # for tag in tags_list:
    #     if tag != None:
    #         print("0x%0.4x" % tag)

    pub_list = list()
    for tag in tags_list:
        if tag == None:
            pub_list.append(rospy.Publisher('pozyx_positioning_local', PointStamped, queue_size=1))
        else:
            pub_list.append(rospy.Publisher('pozyx_positioning_' + str("0x%0.4x" % tag), PointStamped, queue_size=1))
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_first_pozyx_serial_port())
    except:
        rospy.loginfo("Pozyx not connected")
        return

    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        msg = PointStamped()
        msg.header.frame_id = "pozyx"
        for tag, pub in zip(tags_list, pub_list):
            pozyx.doPositioning(coords, remote_id=tag)
            msg.header.stamp = rospy.Time.now()
            msg.point.x = coords.x
            msg.point.y = coords.y
            msg.point.z = coords.z
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        pozyx_positioning_pub()
    except rospy.ROSInterruptException:
        pass
