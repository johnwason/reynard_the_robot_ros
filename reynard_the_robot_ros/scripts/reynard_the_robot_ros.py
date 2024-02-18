#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import reynard_the_robot_ros_msgs.msg as reynard_msg
import reynard_the_robot_ros_msgs.srv as reynard_srv
from reynard_the_robot import Reynard
import numpy as np

reynard = Reynard()
reynard.start()

def say_callback(data):
    reynard.say(data.data)

def teleport_callback(req):
    res = reynard_srv.TeleportResponse()
    try:
        reynard.teleport(req.x*1e3, req.y*1e3)
    except Exception as e:
        res.success = False
        res.status_message = str(e)
    else:
        res.success = True
    return res

def drive_robot_callback(req):
    res = reynard_srv.DriveResponse()
    try:
        assert len(req.velocity) == 2, "Velocity must be a 2-element list"
        vel2 = np.array(req.velocity, dtype=np.float64)*1e3
        reynard.drive_robot(vel2[0], vel2[1], req.timeout, req.wait)
    except Exception as e:
        res.success = False
        res.status_message = str(e)
    else:
        res.success = True
    return res

def set_arm_position_callback(req):
    res = reynard_srv.SetPositionResponse()
    try:
        assert len(req.target_position) == 3, "Position must be a 3-element list"
        pos = np.rad2deg(np.array(req.target_position, dtype=np.float64))
        reynard.set_arm_position(pos[0], pos[1], pos[2])
    except Exception as e:
        res.success = False
        res.status_message = str(e)
    else:
        res.success = True
    return res

def drive_arm_callback(req):
    res = reynard_srv.DriveResponse()
    try:
        assert len(req.velocity) == 3, "Velocity must be a 3-element list"
        vel = np.rad2deg(np.array(req.velocity, dtype=np.float64))
        reynard.drive_arm(vel[0], vel[1], vel[2], req.timeout, req.wait)
    except Exception as e:
        res.success = False
        res.status_message = str(e)
    else:
        res.success = True
    return res

def set_color_callback(req):
    res = reynard_srv.SetColorResponse()
    try:
        reynard.color = (req.r, req.g, req.b)
    except Exception as e:
        res.success = False
        res.status_message = str(e)
    else:
        res.success = True
    return res

def get_color_callback(req):
    res = reynard_srv.GetColorResponse()
    try:
        res.r, res.g, res.b = reynard.color
    except Exception as e:
        res.success = False
        res.status_message = str(e)
    else:
        res.success = True
    return res

def talker():
    rospy.init_node('reynard_the_robot', anonymous=False)
    pub = rospy.Publisher('state', reynard_msg.ReynardState, queue_size=10)
    say_sub = rospy.Subscriber('say', String, say_callback)
    new_mesasge_pub = rospy.Publisher('new_message', String, queue_size=10)
    teleport_srv = rospy.Service('teleport', reynard_srv.Teleport, teleport_callback)
    drive_robot_srv = rospy.Service('drive_robot', reynard_srv.Drive, drive_robot_callback)
    set_arm_position_srv = rospy.Service('set_arm_position', reynard_srv.SetPosition, set_arm_position_callback)
    drive_arm_srv = rospy.Service('drive_arm', reynard_srv.Drive, drive_arm_callback)
    set_color_srv = rospy.Service('set_color', reynard_srv.SetColor, set_color_callback)
    get_color_srv = rospy.Service('get_color', reynard_srv.GetColor, get_color_callback)
    def _new_message_cb(_, message):
        ros_msg = String(message)
        new_mesasge_pub.publish(ros_msg)
    reynard.new_message.connect(_new_message_cb)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ros_state = reynard_msg.ReynardState()
        ros_state.header.stamp = rospy.Time.now()
        ros_state.time = reynard.time
        ros_state.robot_position = np.array(reynard.robot_position,dtype=np.float64)*1e-3
        ros_state.arm_position = np.deg2rad(reynard.arm_position)
        ros_state.robot_velocity = np.array(reynard.robot_velocity,dtype=np.float64)*1e-3
        ros_state.arm_velocity = np.deg2rad(reynard.arm_velocity)
        pub.publish(ros_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
