from math import sqrt
from time import sleep
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class RepublishTF2Topic:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 10)
        if (self.rate < 5):
            self.rate = 5
        self.tfparent_frame = rospy.get_param('~tfparent_frame')
        self.tfchild_frame = rospy.get_param('~tfchild_frame')

        self.topic_msg = TransformStamped()
        self.topic_msg.transform.translation.x = 0
        self.topic_msg.transform.translation.y = 0
        self.topic_msg.transform.translation.z = 0
        self.topic_msg.transform.rotation.x = 0
        self.topic_msg.transform.rotation.y = 0
        self.topic_msg.transform.rotation.z = 0
        self.topic_msg.transform.rotation.w = 1
        self.topic_msg.header.frame_id = self.tfparent_frame
        self.topic_msg.child_frame_id = self.tfchild_frame

        topic = rospy.get_param('~topic', 'topic')
        self.topic_pub = rospy.Publisher(topic, TransformStamped, queue_size=10)

        self.tfbuffer = tf2_ros.Buffer(cache_time = rospy.Duration(1))
        self.tflistener = tf2_ros.TransformListener(self.tfbuffer)

    def republishTF2Topic(self):
        try:
            transform_msg = self.tfbuffer.lookup_transform(self.tfparent_frame, self.tfchild_frame, rospy.Time(0))
            self.topic_msg.transform.translation.x = transform_msg.transform.translation.x
            self.topic_msg.transform.translation.y = transform_msg.transform.translation.y
            self.topic_msg.transform.translation.z = transform_msg.transform.translation.z
            self.topic_msg.transform.rotation.x = transform_msg.transform.rotation.x
            self.topic_msg.transform.rotation.y = transform_msg.transform.rotation.y
            self.topic_msg.transform.rotation.z = transform_msg.transform.rotation.z
            self.topic_msg.transform.rotation.w = transform_msg.transform.rotation.w
            self.topic_msg.header.frame_id = self.tfparent_frame
            self.topic_msg.child_frame_id = self.tfchild_frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        self.topic_msg.header.stamp = rospy.Time.now()
        self.topic_pub.publish(self.topic_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.republishTF2Topic()
            try:
                rospy.Rate(self.rate).sleep()
            except (rospy.exceptions.ROSInterruptException, KeyboardInterrupt) as err:
                rospy.logout(rospy.get_name() + err)
                pass

if __name__ == '__main__':

    rospy.init_node('republish_tf2topic')
    republisher = RepublishTF2Topic()
    republisher.run()