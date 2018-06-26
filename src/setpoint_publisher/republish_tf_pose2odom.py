from math import sqrt
from time import sleep
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class RepublishTFPose2Odom:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 10)
        if (self.rate < 5):
            self.rate = 5
        self.tfmap_frame = rospy.get_param('~tf_map_frame')
        self.tfpose_frame = rospy.get_param('~tf_pose_frame')
        self.tfinitialpose_frame = rospy.get_param('~tf_initial_pose_frame')
        self.tfodom_frame = rospy.get_param('~tf_odom_frame')

        self.odom_msg = TransformStamped()
        self.odom_msg.transform.translation.x = 0
        self.odom_msg.transform.translation.y = 0
        self.odom_msg.transform.translation.z = 0
        self.odom_msg.transform.rotation.x = 0
        self.odom_msg.transform.rotation.y = 0
        self.odom_msg.transform.rotation.z = 0
        self.odom_msg.transform.rotation.w = 1
        self.odom_msg.header.frame_id = self.tfinitialpose_frame
        self.odom_msg.child_frame_id = self.tfodom_frame

        self.poseInitialized = False

        self.tfbroadcaster = tf2_ros.TransformBroadcaster()
        self.tfbuffer = tf2_ros.Buffer(cache_time = rospy.Duration(1))
        self.tflistener = tf2_ros.TransformListener(self.tfbuffer)

    def getInitialPose(self):
        try:
            transform_msg = self.tfbuffer.lookup_transform(self.tfmap_frame, self.tfpose_frame, rospy.Time(0))
            print(rospy.get_name() + ': Got initial pose message.')
            self.initial_pose_msg = TransformStamped()
            self.initial_pose_msg.transform.translation.x = transform_msg.transform.translation.x
            self.initial_pose_msg.transform.translation.y = transform_msg.transform.translation.y
            self.initial_pose_msg.transform.translation.z = transform_msg.transform.translation.z
            self.initial_pose_msg.transform.rotation.x = transform_msg.transform.rotation.x
            self.initial_pose_msg.transform.rotation.y = transform_msg.transform.rotation.y
            self.initial_pose_msg.transform.rotation.z = transform_msg.transform.rotation.z
            self.initial_pose_msg.transform.rotation.w = transform_msg.transform.rotation.w
            self.initial_pose_msg.header.frame_id = self.tfmap_frame
            self.initial_pose_msg.child_frame_id = self.tfinitialpose_frame
            self.initial_pose_msg.header.stamp = rospy.Time.now()                
            self.tfbroadcaster.sendTransform(self.initial_pose_msg)
            self.poseInitialized = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print(rospy.get_name() + ': Could not get initial pose message.')
            return

    def republishTF_Pose2Odom(self):
        try:
            if not hasattr(self, 'initial_pose_msg'):
                print('No initial pose message.')
                return
            transform_msg = self.tfbuffer.lookup_transform(self.tfinitialpose_frame, self.tfpose_frame, rospy.Time(0))
            self.odom_msg.transform.translation.x = transform_msg.transform.translation.x
            self.odom_msg.transform.translation.y = transform_msg.transform.translation.y
            self.odom_msg.transform.translation.z = transform_msg.transform.translation.z
            self.odom_msg.transform.rotation.x = transform_msg.transform.rotation.x
            self.odom_msg.transform.rotation.y = transform_msg.transform.rotation.y
            self.odom_msg.transform.rotation.z = transform_msg.transform.rotation.z
            self.odom_msg.transform.rotation.w = transform_msg.transform.rotation.w
            self.odom_msg.header.frame_id = self.tfinitialpose_frame
            self.odom_msg.child_frame_id = self.tfodom_frame
            self.odom_msg.header.stamp = rospy.Time.now()                
            self.initial_pose_msg.header.stamp = rospy.Time.now()                
            self.tfbroadcaster.sendTransform(self.initial_pose_msg)
            self.tfbroadcaster.sendTransform(self.odom_msg)            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print(rospy.get_name() + ': Could not get odom transform message.')
            return

    def run(self):
        while not rospy.is_shutdown():
            if (not self.poseInitialized):
                self.getInitialPose()
            else:
                self.republishTF_Pose2Odom()
            try:
                rospy.Rate(self.rate).sleep()
            except (rospy.exceptions.ROSInterruptException, KeyboardInterrupt) as err:
                rospy.logout(rospy.get_name() + ' ' + err.message)
                pass

if __name__ == '__main__':

    rospy.init_node('republish_tf_pose2odom')
    republisher = RepublishTFPose2Odom()
    republisher.run()
