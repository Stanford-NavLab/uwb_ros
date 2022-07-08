#!/usr/bin/env python3
"""Republishes VRPN client messages into UWB ranging pairs.

Subscribes to /vrpn_client_node/assest/pose messages and then repacks
them into individual pair ranges. Converts asset names to UWB address
IDs using conversions placed in .yaml parameter file.

Publishes messages as Float32 messages to topics of the form:
"/uwb/vrpn_unpacked/UWBIdentifierA_UWBIdentifierB"

"""

__authors__ = "D. Knowles"
__date__ = "17 Dec 2021"

import rospy
import itertools
import numpy as np
from std_msgs.msg import Float32
import scipy.spatial.distance as dist
from geometry_msgs.msg import PoseStamped

class VRPNUnpacker():
    """Unpacker class which unpacks flighroom VRPN messages.

    """
    def __init__(self):

        # Number of mounts to detect (assumes asset names starting at
        # 1 and increasing to num_mounts)
        self.num_mounts = 5

        # dictionary of name conversions
        self.name_conversion = {}

        for ii in range(1,self.num_mounts+1):
            asset_name = "uwb_mount_" + str(ii)
            self.name_conversion[asset_name] = rospy.get_param(asset_name)

        # keep track of all created publishers
        self.publishers = {}

        self.combos = list(itertools.combinations(range(1,self.num_mounts+1), 2))
        for combo in self.combos:
            asset_name_1 = "uwb_mount_" + str(combo[0])
            asset_name_2 = "uwb_mount_" + str(combo[1])
            ordered = sorted([self.name_conversion[asset_name_1],
                              self.name_conversion[asset_name_2]])
            topic_name =  topic_name = ordered[0] + "_" + ordered[1]

            # create new publisher if it hasn't been created already
            self.publishers[topic_name] = rospy.Publisher("uwb/vrpn_unpacked/" \
                                        + topic_name, Float32,
                                        queue_size = 10)

        for ii in range(1,self.num_mounts+1):
            # Subscribe to ranges and set callback function
            asset_name = "uwb_mount_" + str(ii)
            rospy.Subscriber("vrpn_client_node/" + asset_name + "/pose",
                             PoseStamped, self.vrpn_callback, asset_name)


        # boolean check to see if all mount positions have been updated
        self.updated = [False]*self.num_mounts

        # updated position for each UWB mount
        self.positions = np.zeros((self.num_mounts,3))

        # Initialize ROS node.
        rospy.init_node('vrpn_unpacker', anonymous=False)

        self.cc = 0

        print("Unpacking vrpn range messages...")

        # keeps python from exiting until the node is stopped
        rospy.spin()


    def vrpn_callback(self, data, asset_name):
        """Callback function when new /uwb/range messages are published.

        Parameters
        ----------
        data : ROS uwb/range message
            Data from the topic that was published.
        asset_name : string
            returns asset name with which data is associated
        """

        # TODO: callbacks may be parallelized which is causing a race
        # condition for the global self.positions, and self.updated
        # variables

        mount_idx = int(asset_name[-1]) - 1

        self.positions[mount_idx,0] = data.pose.position.x
        self.positions[mount_idx,1] = data.pose.position.y
        self.positions[mount_idx,2] = data.pose.position.z

        self.updated[mount_idx] = True

        if np.all(self.updated):
            self.updated = self.updated = [False]*self.num_mounts

            current_positions = self.positions.copy()

            for combo in self.combos:
                asset_name_1 = "uwb_mount_" + str(combo[0])
                asset_name_2 = "uwb_mount_" + str(combo[1])
                ordered = sorted([self.name_conversion[asset_name_1],
                                  self.name_conversion[asset_name_2]])
                topic_name =  topic_name = ordered[0] + "_" + ordered[1]

                msg = Float32()
                msg = np.linalg.norm(current_positions[combo[1]-1,:] \
                                    -current_positions[combo[0]-1,:])
                msg *= 1000 # conversion from [m] to [mm]
                self.publishers[topic_name].publish(msg)


    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """
        print("closing safely")


if __name__ == '__main__':
    try:
        u = VRPNUnpacker()
        rospy.on_shutdown(u.cleanup)
    except rospy.ROSInterruptException:
        pass
