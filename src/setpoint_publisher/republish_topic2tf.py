from math import sqrt
from time import sleep
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class RepublishTopic2TF:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 10)
        if (self.rate < 5):
            self.rate = 5
        topic = rospy.get_param('~topic', 'topic')
        topictype = rospy.get_param('~topictype', 'Odometry')

        if topictype.lower() == 'transformstamped':
            rospy.Subscriber(topic, TransformStamped, self.callback)
        elif topictype.lower() == 'odometry':
            rospy.Subscriber(topic, Odometry, self.callback)
        else:
            print(rospy.get_name() + ':' + 'Topic type ' + topictype + ' not recognized!\n')

        self.tfparent_frame = rospy.get_param('~tfparent_frame')
        self.tfchild_frame = rospy.get_param('~tfchild_frame')
        self.tfbroadcaster = tf2_ros.TransformBroadcaster()

    def callback(self, msg):
        if not hasattr(self, 'tf_msg'):
            self.tf_msg = TransformStamped()
            self.tf_msg.header.frame_id = self.tfparent_frame
            self.tf_msg.child_frame_id = self.tfchild_frame

        if isinstance(msg, TransformStamped):
            self.tf_msg.transform.translation.x = msg.transform.translation.x
            self.tf_msg.transform.translation.y = msg.transform.translation.y
            self.tf_msg.transform.translation.z = msg.transform.translation.z
            self.tf_msg.transform.rotation.x = msg.transform.rotation.x
            self.tf_msg.transform.rotation.y = msg.transform.rotation.y
            self.tf_msg.transform.rotation.z = msg.transform.rotation.z
            self.tf_msg.transform.rotation.w = msg.transform.rotation.w

        elif isinstance(msg, Odometry):
            self.tf_msg.transform.translation.x = msg.pose.pose.position.x
            self.tf_msg.transform.translation.y = msg.pose.pose.position.y
            self.tf_msg.transform.translation.z = msg.pose.pose.position.z
            self.tf_msg.transform.rotation.x = msg.pose.pose.orientation.x
            self.tf_msg.transform.rotation.y = msg.pose.pose.orientation.y
            self.tf_msg.transform.rotation.z = msg.pose.pose.orientation.z
            self.tf_msg.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_msg.header.stamp = rospy.Time.now()                
        self.republish()

    def republish(self):
        if hasattr(self, 'tf_msg'):
            self.tfbroadcaster.sendTransform(self.tf_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.republish()
            try:
                rospy.Rate(self.rate).sleep()
            except (rospy.exceptions.ROSInterruptException, KeyboardInterrupt) as err:
                rospy.logout(rospy.get_name() + ' ' + err.message)
                pass

if __name__ == '__main__':

    rospy.init_node('republish_topic2tf')
    republisher = RepublishTopic2TF()
    republisher.run()