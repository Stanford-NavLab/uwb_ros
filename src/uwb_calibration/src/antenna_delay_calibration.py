#!/usr/bin/env python3
"""Antenna delay calibration for UWBs.

Performs antenna delay calibration between an arbitrary number of
Ultrawide band radios (UWBs).

Antenna delay calibration methodology taken from [1]_

Requires measured ranges between each pair of UWBs to be published on a
topic called: "/uwb/range_measured/UWBIdentifierA_UWBIdentifierB"

Requires ground truth ranges between each pair of UWBs to be published
on a topic called: "/uwb/range_truth/UWBIdentifierA_UWBIdentifierB"

For both the measured and truth topics, the following requirements must
also hold:
- each UWB is identified by a code comprising of numbers and/or letters
- the topic name must include the UWB identifiers for both UWBs in the
  pair and be separated by a single underscore
- both topics must be of type Int32

References
----------
.. [1] Decawave. APS014 Application Note. Antenna Delay Calibration of
   DW1000-based products and systems. Version 1.2. (2018).
"""

__authors__ = "D. Knowles"
__date__ = "08 Jul 2022"

import rospy
from std_msgs.msg import Float32
from message_filters import Cache

class AntennaDelayCalibration():
    """Calibration class which unpacks UWB ranging messages.

    """
    def __init__(self):

        # keep track of all range_truth subscribers
        self.truth_subscribers = []

        # keep track of all range_measured subscribers
        self.measured_subscribers = []

        # Initialize ROS node.
        rospy.init_node('antenna_delay_calibration', anonymous=False)
        rate = rospy.Rate(1) # 1Hz

        while not rospy.is_shutdown():

            all_topics = rospy.get_published_topics()
            rospy.loginfo("log %s",all_topics)

            measured_topics = [x for x in all_topics if "/range_measured/" in x[0]]
            truth_topics = [x for x in all_topics if "/range_truth/" in x[0]]

            rate.sleep()

    def subscribe_to_all(self, current_topics, subscribed_topics):
        """Subscribes to all new topics and adds to dictionary.

        Parameters
        ----------
        current_topics : list
            List of [topic_name, topic_type]
        subscribed_topics : list
            List of topics to which the node is already subscribing.

        """
        for topic_name, topic_type in current_topics:
            if topic_name not in subscribed_topics:
                # Subscribe to topic and set callback function
                rospy.Subscriber("range_measured/range/", Float32, self.measured_callback)


    def measured_callback(self, data):
        """Callback function when new /range_measured/ messages are published.

        Parameters
        ----------
        data : ROS /range_measured/ Int32 message
            Data from the topic that was published.

        """
        rospy.loginfo("data:",data)

        # if self.anchor_agnostic:
        #     ordered = sorted([data.anchor_address, data.tag_address])
        #     topic_name = ordered[0][-4:] + "_" + ordered[1][-4:]
        # else:
        #     topic_name = data.anchor_address[-4:] + "_" + data.tag_address[-4:]
        #
        #
        # if topic_name not in self.publishers.keys():
        #     # create new publisher if it hasn't been created already
        #     self.publishers[topic_name] = rospy.Publisher("uwb/range_unpacked/" \
        #                                 + topic_name, RangeEvent,
        #                                 queue_size = 10)
        #
        # self.publishers[topic_name].publish(data)


    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """
        print("closing safely")


if __name__ == '__main__':
    try:
        u = AntennaDelayCalibration()
        rospy.on_shutdown(u.cleanup)
    except rospy.ROSInterruptException:
        pass
