from math import sqrt
from time import sleep
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class RepublishTopic2TF:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 10)
        if (self.rate < 5):
            self.rate = 5
        topic = rospy.get_param('~topic', 'topic')
        self.topic_sub = rospy.Subscriber(topic, TransformStamped, self.callback)
        self.tfparent_frame = rospy.get_param('~tfparent_frame')
        self.tfchild_frame = rospy.get_param('~tfchild_frame')
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.tfparent_frame
        self.tf_msg.child_frame_id = self.tfchild_frame
        self.tf_msg.transform.translation.x = 0
        self.tf_msg.transform.translation.y = 0
        self.tf_msg.transform.translation.z = 0
        self.tf_msg.transform.rotation.x = 0
        self.tf_msg.transform.rotation.y = 0
        self.tf_msg.transform.rotation.z = 0
        self.tf_msg.transform.rotation.w = 1
        self.tfbroadcaster = tf2_ros.TransformBroadcaster()

    def callback(self, transform_msg):
        self.tf_msg.transform.translation.x = transform_msg.transform.translation.x
        self.tf_msg.transform.translation.y = transform_msg.transform.translation.y
        self.tf_msg.transform.translation.z = transform_msg.transform.translation.z
        self.tf_msg.transform.rotation.x = transform_msg.transform.rotation.x
        self.tf_msg.transform.rotation.y = transform_msg.transform.rotation.y
        self.tf_msg.transform.rotation.z = transform_msg.transform.rotation.z
        self.tf_msg.transform.rotation.w = transform_msg.transform.rotation.w
        self.topic_msg.header.stamp = rospy.Time.now()                
        #self.republish()

    def republish(self):
        self.tfbroadcaster.sendTransform(self.tf_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.republish()
            try:
                rospy.Rate(self.rate).sleep()
            except (rospy.exceptions.ROSInterruptException, KeyboardInterrupt) as err:
                rospy.logout(rospy.get_name() + err)
                pass

if __name__ == '__main__':

    rospy.init_node('republish_topic2tf')
    republisher = RepublishTopic2TF()
    republisher.run()