#!/usr/bin/env python
"""
Performs discovery for all tags in range and then configures the positioning
anchor list on all the discovered devices.

This requires all devices to be on the same UWB settings first, so I highly
recommend to run the uwb_configurator node first.
"""

ANCHOR_HEIGHT = 1775
TAG_HEIGHT = 10

import pypozyx
import rospy
import time

def set_anchor_configuration():
    tag_ids = [None]
    # tag_ids = [None, 0x600D]
    rospy.init_node('autocalibration')
    rospy.loginfo("Configuring device list.")

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS and POS_Z
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_first_pozyx_serial_port())
    except:
        rospy.loginfo("Pozyx not connected")
        return

    for tag in tag_ids:
        pozyx.clearDevices(remote_id=tag)
        ret = pozyx.setHeight(TAG_HEIGHT, remote_id=tag)
        if pypozyx.POZYX_SUCCESS != ret:
            if tag != None:
                print("Failed to set height to 0x%0.4x" % tag)
            else:
                print("Failed to set height to local device")
        pozyx.saveRegisters([0x38], remote_id=tag)

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
        print("  - 0x%0.4x" % anchor_id)

    for tag in tag_ids:
        while True:
            ret = pozyx.doAnchorCalibration(pypozyx.PozyxConstants.DIMENSION_2_5D, 200, anchors_list.data, heights=[ANCHOR_HEIGHT]*len(anchors_list.data), remote_id=None)
            if pypozyx.POZYX_SUCCESS != ret:
                if tag != None:
                    print("Failed to calibrate 0x%0.4x" % tag)
                else:
                    print("Failed to calibrate for local" % tag)
                continue
            for anchor in anchors_list.data:
                status = pypozyx.SingleRegister()
                pozyx.getCalibrationStatus(status, remote_id=anchor)
                print('Calib status for 0x%0.4x' % anchor)
                print('    {}'.format(status[0]))
                if status[0] == 0:
                    break
            else: # nobreak
                break

    # for anchor in anchors_list.data:
    #     print("Positioning 0x%0.4x" % anchor)
    #     coord = pypozyx.Coordinates()
    #     pozyx.doPositioning(coord, dimension=pypozyx.PozyxConstants.DIMENSION_2_5D, height=pypozyx.Data([1775], 'i'), timeout=0.2, remote_id=anchor)
    #     print(coord)
    #     pozyx.saveRegisters([0x38], remote_id=tag)

        # coord = pypozyx.Coordinates()
        # pozyx.getCoordinates(coord, remote_id=anchor)
        # print(coord)
        # pozyx.addDevice(pypozyx.DeviceCoordinates(anchor, 1, coord), tag)

    # for anchor in anchors_list.data:
    #     pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
    #                                 len(anchors_list.data)-1, remote_id=anchor)
    #     pozyx.saveRegisters(settings_registers, remote_id=anchor)
    #
    # if len(anchors_list.data) > 4:
    #     pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
    #                                 len(anchors_list.data), remote_id=None)
    #     pozyx.saveRegisters(settings_registers, remote_id=None)
    # pozyx.saveNetwork(remote_id=None)
    # rospy.loginfo("Local device configured")

    # pozyx.clearDevices()
    #
    # pozyx.doDiscovery(discovery_type=pypozyx.POZYX_DISCOVERY_TAGS_ONLY)
    # device_list_size = pypozyx.SingleRegister()
    # pozyx.getDeviceListSize(device_list_size)
    # tags_list = pypozyx.DeviceList(list_size=device_list_size[0])
    # pozyx.getDeviceIds(tags_list)
    # print("Number of found tags: {}".format(len(tags_list.data)))
    # for tag_id in tags_list.data:
    #     print("tag: 0x%0.4x" % tag_id)

    # for tag_id in tags_list:
    #     print("tag: 0x%0.4x" % tag_id)
    #
    #

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
            pozyx.clearDevices(remote_id=tag)
            for anchor in anchors_list.data:
                print("Positioning 0x%0.4x" % anchor)
                coord = pypozyx.Coordinates()
                pozyx.doPositioning(coord, dimension=pypozyx.PozyxConstants.DIMENSION_2_5D, height=pypozyx.Data([1735], 'i'), timeout=0.2, remote_id=anchor)
                print(coord)
                pozyx.addDevice(pypozyx.DeviceCoordinates(anchor, 1, coord), tag)
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
