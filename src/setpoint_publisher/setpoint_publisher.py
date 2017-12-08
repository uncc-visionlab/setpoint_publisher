from math import sqrt
from time import sleep
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SetpointPublisher:

    def __init__(self):
        self.setpoint_index = 0
        self.setpoint_radius = rospy.get_param('~setpoint_radius', 0.1) # meters
        self.use_tf = rospy.get_param('~use_tf', True)
        self.setpoint_frame = rospy.get_param('~setpoint_frame')

        setpoints_file_path = rospy.get_param('~setpoints_file_path')
        self.setpoints = self.getSetpointsFromFile(setpoints_file_path)
        setpoint_topic = rospy.get_param('~setpoint_topic', 'setpoints')
        self.setpoint_pub = rospy.Publisher(setpoint_topic, TransformStamped, queue_size=10)

        self.setpoint_msg = TransformStamped()
        self.setpoint_msg.header.frame_id = self.setpoint_frame
        self.setpoint_msg.child_frame_id = "setpoint"
        self.updateSetpointMsg()
        
        init_time = rospy.get_param('~init_time', 0)
        sleep(init_time)
        
        if self.use_tf:
            self.tfparent_frame = rospy.get_param('~tfparent_frame')
            self.tfchild_frame = rospy.get_param('~tfchild_frame')
            self.tfbuffer = tf2_ros.Buffer(cache_time = rospy.Duration(1))
            self.tflistener = tf2_ros.TransformListener(self.tfbuffer)
            self.tfbroadcaster = tf2_ros.TransformBroadcaster()
        else:
            pose_topic = rospy.get_param('~pose_topic')
            self.pose_sub = rospy.Subscriber(pose_topic, TransformStamped, self.callback)

    def getSetpointsFromFile(self, setpoints_file_path):
        # get setpoints from textfile, [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]
        with open(setpoints_file_path) as txtfile:
            setpoints = [[float(value) for value in line.strip("\n").split()] for line in txtfile.readlines()]
        return setpoints 

    def distance(self, current_position, setpoint_position):
        distance = sqrt(sum([(a - b)**2 for a, b in zip(current_position, setpoint_position)]))
        rospy.logdebug_throttle(.5, rospy.get_name() + ': client is ' + str(distance) + ' m away from target.')
        return distance

    def evaluate(self, current_position):
        active_setpoint = self.setpoints[self.setpoint_index]

        if self.distance(current_position, active_setpoint[0:3]) < self.setpoint_radius:
            if self.setpoint_index < len(self.setpoints)-1:
                self.changeActiveSetpoint(self.setpoint_index + 1)
                self.updateSetpointMsg()
                rospy.logdebug(rospy.get_name() + ' setpoint changed to: ' + str(self.setpoints[self.setpoint_index]))

    def changeActiveSetpoint(self, new_index):
        self.setpoint_index = new_index

    def updateSetpointMsg(self):
        active_setpoint = self.setpoints[self.setpoint_index]
        self.setpoint_msg.transform.translation.x = active_setpoint[0]
        self.setpoint_msg.transform.translation.y = active_setpoint[1]
        self.setpoint_msg.transform.translation.z = active_setpoint[2]
        self.setpoint_msg.transform.rotation.x = active_setpoint[3]
        self.setpoint_msg.transform.rotation.y = active_setpoint[4]
        self.setpoint_msg.transform.rotation.z = active_setpoint[5]
        self.setpoint_msg.transform.rotation.w = active_setpoint[6]

    def callback(self, msg):
        translation = msg.transform.translation
        current_position = [translation.x, translation.y, translation.z]
        self.evaluate(current_position)

    def updateUsingTF(self):
        if self.tflistener:
            try:
                transform_msg = self.tfbuffer.lookup_transform(self.tfparent_frame, self.tfchild_frame, rospy.Time(0))
                position = [
                    transform_msg.transform.translation.x, 
                    transform_msg.transform.translation.y, 
                    transform_msg.transform.translation.z]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return
            self.evaluate(position)
        else:
            raise RuntimeError('No tf listener has been instantiated!')

    def publishSetpoint(self):
        self.setpoint_msg.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.setpoint_msg)

        if self.tfbroadcaster:
            self.tfbroadcaster.sendTransform(self.setpoint_msg)
        else:
            raise RuntimeError('No tf broadcaster has been instantiated!')

    def run(self):
        while not rospy.is_shutdown():
            if self.use_tf:
                self.updateUsingTF()

            self.publishSetpoint()
            rospy.Rate(100).sleep()

if __name__ == '__main__':

    rospy.init_node('setpoint_publisher')
    setpoint_publisher = SetpointPublisher()
    setpoint_publisher.run()