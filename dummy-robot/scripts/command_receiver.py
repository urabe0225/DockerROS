#!/usr/bin/env python
import rospy
import datetime
from uoa_poc2_msgs.msg import r_command, r_result, r_pose_optional, r_angle_optional, r_angle
import pytz
def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)
    pub = rospy.Publisher('/robot_bridge/delivery_robot_01/cmdexe', r_result)
    while True:
        inp=raw_input('robot was reached?[y/n]>>')
        if inp == 'y':break
    cmdexe = r_result()
    cmdexe.id = data.id
    cmdexe.type = data.type
    cmdexe.time = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).isoformat()
    cmdexe.received_time = data.time
    cmdexe.received_cmd = data.cmd
    cmdexe.received_waypoints = data.waypoints
    cmdexe.result = "ack"
    cmdexe.errors = []
    pub.publish(cmdexe)
    rospy.loginfo(rospy.get_caller_id()+"I published %s",cmdexe)

def listener():
    rospy.init_node('command_receiver', anonymous=True)
    rospy.Subscriber("/robot_bridge/delivery_robot_01/cmd", r_command, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
