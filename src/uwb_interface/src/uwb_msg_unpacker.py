#!/usr/bin/env python3
"""Unpacks UWB ranging messages into individual pairs.

The original /uwb/range message includes ranges between all pairs of
UWBs with possible additional information messages. This node will
subscribe to the uwb/range topic, unpack the messages into the relevant
pairs and then republish the unpacked information to unique ROS topics
for each pair of UWBs.

"""

__authors__ = "D. Knowles"
__date__ = "16 Dec 2021"

import rospy
from uwb_interface.msg import RangeEvent

class UWBUnpacker():
    """Unpacker class which unpacks UWB ranging messages.

    """
    def __init__(self):

        # If True, it will combine the A -> B messages with the B -> A
        # messages as a single topic.
        self.anchor_agnostic = True

        # keep track of all created publishers
        self.publishers = {}

        # Initialize ROS node.
        rospy.init_node('uwb_unpacker', anonymous=False)

        # Subscribe to ranges and set callback function
        rospy.Subscriber("uwb/range/", RangeEvent, self.range_callback)

        print("Unpacking UWB range messages...")

        # keeps python from exiting until the node is stopped
        rospy.spin()


    def range_callback(self, data):
        """Callback function when new /uwb/range messages are published.

        Parameters
        ----------
        data : ROS uwb/range message
            Data from the topic that was published.

        """
        if self.anchor_agnostic:
            ordered = sorted([data.anchor_address, data.tag_address])
            topic_name = ordered[0] + "_" + ordered[1]
        else:
            topic_name = data.anchor_address + "_" + data.tag_address

        if topic_name not in self.publishers.keys():
            # create new publisher if it hasn't been created already
            self.publishers[topic_name] = rospy.Publisher("uwb/range_unpacked/" \
                                        + topic_name, RangeEvent,
                                        queue_size = 10)

        self.publishers[topic_name].publish(data)


    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """
        print("closing safely")


if __name__ == '__main__':
    try:
        u = UWBUnpacker()
        rospy.on_shutdown(u.cleanup)
    except rospy.ROSInterruptException:
        pass
