#!/usr/bin/env python3
"""Republishes UWB range data as Float32 message.

Subscribes to /uwb/data/UWBIdentifierA_UWBIdentifierB messages, reads
the raw ranges, and repacks them as Float32 messages.

Publishes messages as RangeStamped messages to topics of the form:
"/uwb/range_measured/UWBIdentifierA_UWBIdentifierB"

"""

__authors__ = "D. Knowles"
__date__ = "09 Jul 2022"

import rospy
from uwb_interface.msg import UWBRange, RangeStamped

class UWBMeasured():
    """Unpacker class which unpacks flighroom VRPN messages.

    """
    def __init__(self):

        # Initialize ROS node.
        rospy.init_node('uwb_measured', anonymous=False)

        # keep track of all created publishers
        self.publishers = {}

        # keep track of current subscribers
        self.subscribers = set()

        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():

            all_topics = rospy.get_published_topics()
            uwb_data_topics = {x[0] for x in all_topics if "/uwb/data/" in x[0]}

            # get new topics that haven't yet been subscribed to
            new_topics = uwb_data_topics - self.subscribers

            for new_topic in new_topics:
                rospy.Subscriber(new_topic, UWBRange,
                                 callback = self.uwb_callback,
                                 callback_args = new_topic)
                self.subscribers.add(new_topic)


            rate.sleep()

    def uwb_callback(self, data, topic_name):
        """Callback function when new /uwb/data/ messages are published.

        Parameters
        ----------
        data : RangeEvent
            Data from / topic that was published.
        topic_name : string
            Topic name for which this callback function was called.

        """

        new_msg = RangeStamped()
        new_msg.range = data.range_raw

        # use RangeEvent timestamp as the header timestamp
        new_msg.header.stamp = rospy.Time.from_sec(data.timestamp/1E9)

        # use current ROS time as the header timestamp
        # new_msg.header.stamp = data.header.stamp

        if topic_name not in self.publishers:
            # create new publisher if it hasn't been created already
            new_publisher_name = "uwb/range_measured/" \
                               + topic_name.split("/")[-1]
            self.publishers[topic_name] = rospy.Publisher(new_publisher_name,
                                        RangeStamped, queue_size = 10)

        if new_msg.header.stamp is not None:
            self.publishers[topic_name].publish(new_msg)


    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """
        print("closing safely")


if __name__ == '__main__':
    try:
        u = UWBMeasured()
        rospy.on_shutdown(u.cleanup)
    except rospy.ROSInterruptException:
        pass
