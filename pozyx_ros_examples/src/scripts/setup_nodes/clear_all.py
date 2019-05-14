#!/usr/bin/env python
"""
Performs discovery for all tags in range and then configures the positioning
anchor list on all the discovered devices.

This requires all devices to be on the same UWB settings first, so I highly
recommend to run the uwb_configurator node first.
"""

import pypozyx
import rospy
import time

def clear_config():
    tag_ids = [None]
    rospy.init_node('clear_all')

    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_first_pozyx_serial_port())
    except:
        rospy.loginfo("Pozyx not connected")
        return

    pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_ALL_DEVICES, slots=9, slot_duration=0.2)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)

    if device_list_size[0] > 0:
        dev_list = pypozyx.DeviceList(list_size=device_list_size[0])
        pozyx.getDeviceIds(dev_list)
    else:
        rospy.loginfo("No other device")
        return

    for dev in dev_list.data:
        print("dev: 0x%0.4x" % dev)
        pozyx.printDeviceInfo(remote_id=dev)
        pozyx.clearDevices(remote_id=dev)
        pozyx.clearConfiguration(remote_id=dev)
        pozyx.resetSystem(remote_id=dev)

    pozyx.printDeviceInfo()
    pozyx.clearDevices()
    pozyx.clearConfiguration()
    pozyx.resetSystem()

    rospy.loginfo("Configuration completed! Shutting down node now...")


if __name__ == '__main__':
    clear_config()
