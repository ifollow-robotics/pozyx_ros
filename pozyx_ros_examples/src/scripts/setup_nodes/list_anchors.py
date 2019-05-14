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

def set_anchor_configuration():
    tag_ids = [None]
    rospy.init_node('list_anchors')

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_first_pozyx_serial_port())
    except:
        rospy.loginfo("Pozyx not connected")
        return

    pozyx.clearDevices()

    pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_ANCHORS_ONLY, slots=9, slot_duration=0.2)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)

    if device_list_size[0] > 0:
        anchors_list = pypozyx.DeviceList(list_size=device_list_size[0])
        pozyx.getDeviceIds(anchors_list)
    else:
        rospy.loginfo("Not enough anchors")
        return

    print("Number of found anchors: {}".format(len(anchors_list.data)))

    for anchor_id in anchors_list.data:
        print("anchor: 0x%0.4x" % anchor_id)
        mode = pypozyx.SingleRegister()
        pozyx.getOperationMode(mode, remote_id=anchor_id)
        if mode == 0:
            print("    op mode = tag")
        else:
            print("    op mode = anchor")

if __name__ == '__main__':
    set_anchor_configuration()
