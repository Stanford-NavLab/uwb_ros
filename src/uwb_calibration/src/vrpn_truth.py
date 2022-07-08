#!/usr/bin/env python3
"""Republishes VRPN client messages into UWB ranging pairs.

Subscribes to /vrpn_client_node/assest/pose messages and then repacks
them into individual pair ranges. Converts asset names to UWB address
IDs using conversions placed in .yaml parameter file.

Expects that PoseStamped messages are being published on topics called:
"<vrpn_prefix>/<vrpn_asset_name>/pose" where <vrpn_prefix> and
<vrpn_asset_name>  are defined in uwb_calibration/param/vrpn.yaml

Publishes messages as Float32 messages to topics of the form:
"/uwb/range_truth/UWBIdentifierA_UWBIdentifierB"

"""

__authors__ = "D. Knowles"
__date__ = "08 Jul 2022"

from threading import Lock

import rospy
import itertools
import numpy as np
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

class VRPNTruth():
    """Unpacker class which unpacks flighroom VRPN messages.

    """
    def __init__(self):

        # Initialize ROS node.
        rospy.init_node('vrpn_truth', anonymous=False)
        vrpn_prefix = self.get_vrpn_prefix()
        sync_slop = 0.125/rospy.get_param("vrpn_rate_hz")
        print("ss:",sync_slop)
        self.cc = 0


        self.lock = Lock()

        # keep track of all created publishers
        self.publishers = {}


        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():

            all_topics = rospy.get_published_topics()
            vrpn_topics = [x for x in all_topics if vrpn_prefix in x[0]]

            subscribers = []
            topic_names_temp = []
            for topic_name, topic_type in sorted(vrpn_topics):
                # print(topic_name, topic_type)
                sub = message_filters.Subscriber(topic_name, PoseStamped)
                subscribers.append(sub)
                topic_names_temp.append(topic_name)

            with self.lock:
                self.topic_names = topic_names_temp.copy()
                synchronizer = message_filters.ApproximateTimeSynchronizer(subscribers,
                               queue_size = 1, slop = sync_slop)
                # synchronizer = message_filters.TimeSynchronizer(subscribers,
                #                queue_size = 10)#, slop = synch_slop)
                synchronizer.registerCallback(self.vrpn_callback)

            rate.sleep()


    def vrpn_callback(self, *msgs):
        """Callback function when new /uwb/range messages are published.

        Parameters
        ----------
        data : ROS uwb/range message
            Data from the topic that was published.
        asset_name : string
            returns asset name with which data is associated
        """

        with self.lock:
            if len(msgs) != len(self.topic_names):
                return

            # updated position for each UWB mount
            uwb_positions = np.zeros((len(msgs),3))

            # compute all possible combinations
            self.combos = list(itertools.combinations(range(0,len(msgs)), 2))
            print(self.cc)
            for m_idx, msg_data in enumerate(msgs):
                uwb_positions[m_idx,0] = msg_data.pose.position.x
                uwb_positions[m_idx,1] = msg_data.pose.position.y
                uwb_positions[m_idx,2] = msg_data.pose.position.z
                print(self.topic_names[m_idx],msg_data.header.seq)

            self.cc += 1
            if self.cc >= 400:
                hi

            for combo in self.combos:
                asset_names = [self.topic_names[x].split("/")[-2]
                               for x in combo]
                uwb_ids = sorted([rospy.get_param(x)
                                  for x in asset_names])

                topic_name = "/uwb/range_truth/" + uwb_ids[0] + "_" \
                           + uwb_ids[1]

                if topic_name not in self.publishers.keys():
                    # create new publisher if it hasn't been created already
                    self.publishers[topic_name] = rospy.Publisher(topic_name,
                                                Float32, queue_size = 10)

                new_msg = Float32()
                new_msg_data = np.linalg.norm(uwb_positions[combo[1],:] \
                                            - uwb_positions[combo[0],:])
                new_msg_data *= 1000 # conversion from [m] to [mm]
                new_msg.data = new_msg_data

                self.publishers[topic_name].publish(new_msg)


    def get_vrpn_prefix(self):
        """Get vrpn prefix and check for starting/ending forward slash.

        """
        prefix = rospy.get_param("vrpn_prefix")
        if prefix[0] != "/":
            prefix = "/" + prefix
        if prefix[-1] != "/":
            prefix = prefix + "/"

        return prefix

    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """
        print("closing safely")


if __name__ == '__main__':
    try:
        u = VRPNTruth()
        rospy.on_shutdown(u.cleanup)
    except rospy.ROSInterruptException:
        pass
