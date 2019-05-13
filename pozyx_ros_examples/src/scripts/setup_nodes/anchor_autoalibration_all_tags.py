#!/usr/bin/env python
"""
Performs discovery for all tags in range and then configures the positioning
anchor list on all the discovered devices.

This requires all devices to be on the same UWB settings first, so I highly
recommend to run the uwb_configurator node first.
"""

import pypozyx
import rospy

# anchors = [pypozyx.DeviceCoordinates(0x0001, 1, pypozyx.Coordinates(0, 0, 5000)),
#            pypozyx.DeviceCoordinates(0x0002, 1, pypozyx.Coordinates(5000, 0, 1000)),
#            pypozyx.DeviceCoordinates(0x0003, 1, pypozyx.Coordinates(0, 5000, 1000)),
#            pypozyx.DeviceCoordinates(0x0004, 1, pypozyx.Coordinates(5000, 5000, 1000))]

# anchors_ids = [0x614b, 0x6025, 0x611b, 0x6045, 0x612f, 0x6160]

# dev: 0x6045
# dev: 0x6025
# dev: 0x611b
# dev: 0x614b
# dev: 0x612f


def set_anchor_configuration():
    tag_ids = [None, 0x6152]
    rospy.init_node('uwb_configurator')
    rospy.loginfo("Configuring device list.")


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
    # pozyx.clearDevices()

    # pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_ALL_DEVICES, slots=9, slot_duration=0.2)
    # device_list_size = pypozyx.SingleRegister()
    # pozyx.getDeviceListSize(device_list_size)
    # tags_list = pypozyx.DeviceList(list_size=device_list_size[0])


    print("Number of found anchors: {}".format(len(anchors_list.data)))

    for anchor_id in anchors_list.data:
        print("anchor: 0x%0.4x" % anchor_id)

    pozyx.doAnchorCalibration(pypozyx.PozyxConstants.DIMENSION_2_5D, 255, anchors_list.data, heights=[1735]*len(anchors_list.data), remote_id=None)


    pozyx.clearDevices()

    pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_TAGS_ONLY)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)
    tags_list = pypozyx.DeviceList(list_size=device_list_size[0])
    pozyx.getDeviceIds(tags_list)
    print("Number of found tags: {}".format(len(tags_list.data)))
    for tag_id in tags_list.data:
        print("tag: 0x%0.4x" % tag_id)

    # for tag_id in tags_list:
    #     print("tag: 0x%0.4x" % tag_id)

    # pozyx.getDeviceListSize(device_list_size)
    # if len(anchors_list.data) >= 4:
    #     anchors_list = pypozyx.DeviceList(list_size=device_list_size[0])
    #     pozyx.getDeviceIds(anchors_list)
    #     for dev_id in anchors_list.data:
    #         print("dev: 0x%0.4x" % dev_id)
    #         pozyx.clearDevices(remote_id=dev_id)
    #         if dev_id not in anchors_ids:
    #             tag_ids.append(dev_id)
    # for tag in tag_ids:
    #     if tag != None:
    #         print("tag: 0x%0.4x" % tag)
    for tag in tag_ids:
        if tag != None:
            for anchor in anchors_list.data:
                print("Positioning 0x%0.4x" % anchor)
                coord = pypozyx.Coordinates()
                pozyx.doPositioning(coord, dimension=pypozyx.PozyxConstants.DIMENSION_2_5D, height=pypozyx.Data([1735], 'i'), timeout=0.2, remote_id=anchor)
                print(coord)
                pozyx.addDevice(pypozyx.DeviceCoordinates(anchor, 1, pypozyx.Coordinates(coord[0], coord[1], 1735)), tag)
        if len(anchors_list.data) > 4:
            pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
                                        len(anchors_list.data), remote_id=tag)
            pozyx.saveRegisters(settings_registers, remote_id=tag)
        pozyx.saveNetwork(remote_id=tag)
        if tag is None:
            rospy.loginfo("Local device configured")
        else:
            rospy.loginfo("Device with ID 0x%0.4x configured." % tag)
    rospy.loginfo("Configuration completed! Shutting down node now...")


if __name__ == '__main__':
    set_anchor_configuration()
