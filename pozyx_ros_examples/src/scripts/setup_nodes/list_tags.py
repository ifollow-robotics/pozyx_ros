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
    rospy.init_node('list_tags')

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_first_pozyx_serial_port())
    except:
        rospy.loginfo("Pozyx not connected")
        return

    pozyx.clearDevices()
    # pozyx.clearConfiguration()

    pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_TAGS_ONLY, slots=9, slot_duration=0.2)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)

    if device_list_size[0] > 0:
        anchors_list = pypozyx.DeviceList(list_size=device_list_size[0])
        pozyx.getDeviceIds(anchors_list)
    else:
        rospy.loginfo("No remote tags")
        return
    # pozyx.clearDevices()

    # pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_ALL_DEVICES, slots=9, slot_duration=0.2)
    # device_list_size = pypozyx.SingleRegister()
    # pozyx.getDeviceListSize(device_list_size)
    # tags_list = pypozyx.DeviceList(list_size=device_list_size[0])

    print("Number of found tags: {}".format(len(anchors_list.data)))

    for anchor_id in anchors_list.data:
        print("tag: 0x%0.4x" % anchor_id)
        mode = pypozyx.SingleRegister()
        pozyx.getOperationMode(mode, remote_id=anchor_id)
        if mode == 0:
            print("tag mode")
        else:
            print("anchor mode")


if __name__ == '__main__':
    set_anchor_configuration()
