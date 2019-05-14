#!/usr/bin/env python
"""ROS node that performs 3D positioning on Pozyx"""

import pypozyx
import rospy
from geometry_msgs.msg import PointStamped

remote_id = None
# remote_id = 0x6152

def pozyx_positioning_pub():
    pub = rospy.Publisher('pozyx_positioning_' + str(remote_id), PointStamped, queue_size=100)
    rospy.init_node('positioning_pub')
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_first_pozyx_serial_port())
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        pozyx.doPositioning(coords, remote_id=remote_id)
        rospy.loginfo(coords)
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "pozyx"
        msg.point.x = coords.x
        msg.point.y = coords.y
        msg.point.z = coords.z
        pub.publish(msg)

if __name__ == '__main__':
    try:
        pozyx_positioning_pub()
    except rospy.ROSInterruptException:
        pass
