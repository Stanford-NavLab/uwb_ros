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
import numpy as np
import message_filters
from uwb_interface.msg import RangeStamped

class AntennaDelayCalibration():
    """Calibration class which unpacks UWB ranging messages.

    """
    def __init__(self):

        # keep track of all range_truth subscribers
        self.truth_subscribers = set()
        self.truth_cache = {}

        # keep track of all range_measured subscribers
        self.measured_subscribers = set()

        # error for ranges
        self.range_error = {}

        # Initialize ROS node.
        rospy.init_node('antenna_delay_calibration', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(1) # 1Hz

        while not rospy.is_shutdown():


            all_topics = rospy.get_published_topics()

            # update measured topics
            measured_topics = {x[0] for x in all_topics if "/uwb/range_measured/" in x[0]}
            new_measured_topics = measured_topics - self.measured_subscribers
            if len(new_measured_topics) > 0:
                self.subscribe_to_all(new_measured_topics)

            truth_topics = {x[0] for x in all_topics if "/uwb/range_truth/" in x[0]}
            new_truth_topics = truth_topics - self.truth_subscribers
            if len(new_truth_topics) > 0:
                self.cache_all(new_truth_topics)

            rate.sleep()

    def subscribe_to_all(self, current_topics):
        """Subscribes to all new topics and adds to dictionary.

        Parameters
        ----------
        current_topics : list
            List of [topic_name, topic_type]

        """

        for topic_name in current_topics:
            # Subscribe to topic and set callback function

            sorted_topic_name = self.get_sorted_name(topic_name)

            rospy.Subscriber(topic_name, RangeStamped,
                             callback = self.measured_callback,
                             callback_args = sorted_topic_name)
            self.measured_subscribers.add(topic_name)
            self.range_error[sorted_topic_name] = []

    def cache_all(self, current_topics):
        """Subscribes to new topics and creates message_filters cache.

        Parameters
        ----------
        current_topics : list
            List of [topic_name, topic_type]

        """
        for topic_name in current_topics:
            # Subscribe to topic and set cache
            sub = message_filters.Subscriber(topic_name, RangeStamped)

            sorted_topic_name = self.get_sorted_name(topic_name)

            self.truth_cache[sorted_topic_name] = message_filters.Cache(sub, 100)
            self.truth_subscribers.add(topic_name)


    def measured_callback(self, data_measured, sorted_topic_name):
        """Callback function when new /range_measured/ messages are published.

        Parameters
        ----------
        data_measured : ROS /range_measured/ RangeStamped message
            Data from the topic that was published.
        sorted_topic_name : string
            Topic name for which this callback function was called.

        """

        if sorted_topic_name in self.truth_cache:

            timestamp_measured = data_measured.header.stamp
            data_truth = self.truth_cache[sorted_topic_name].getElemBeforeTime(timestamp_measured)

            if data_truth is None:
                rospy.logwarn("No truth measurement before timestamp for %s",sorted_topic_name)
                return

            timestamp_truth = data_truth.header.stamp
            if (timestamp_measured.to_sec() - timestamp_truth.to_sec()) > 1.0:
                rospy.logwarn("Truth data older than one second for %s", sorted_topic_name)
                return


            self.range_error[sorted_topic_name].append(data_truth.range - data_measured.range)

        else:
            rospy.logwarn_throttle(1,"No truth yet for %s",sorted_topic_name)
            # rospy.logwarn("Current truth topics are %s:",self.truth_cache.keys())


    def get_sorted_name(self, topic_name):
        """Get new topic name sorted alphnumerically

        Parameters
        ----------
        topic_name : string
            Current full topic name

        Returns
        -------
        sorted_name : string
            New topic name, but sorted alphabetically

        """
        name_list = topic_name.split("/")[-1]
        uwb_identifiers = sorted(name_list.split("_"))
        sorted_name = "_".join(uwb_identifiers)
        return sorted_name

    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """

        for key, value in self.range_error.items():
            print(key)
            value = np.array(value)
            print(np.min(value),np.mean(value),np.max(value))

        print("closing safely")


if __name__ == '__main__':
    try:
        u = AntennaDelayCalibration()
        rospy.on_shutdown(u.cleanup)
    except rospy.ROSInterruptException:
        pass
