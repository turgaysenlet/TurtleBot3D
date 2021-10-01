#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import os
import binascii
from std_msgs.msg import String
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import SensorState
from geometry_msgs.msg import Twist

global state
state = 0
global previous_state
previous_state = 0
global counter
counter = 0


def sensorCallback(data):
    global state
    global counter
    global previous_state
    # rospy.loginfo(rospy.get_caller_id() + ' - Current: "%d" - "%d"', int.from_bytes(data.current, "big"))
    # rospy.loginfo(rospy.get_caller_id() + " - Current: '" + data.current[0] + "' '" + data.current[1] + "'")
    # rospy.loginfo(rospy.get_caller_id() + " - Current: '" + data.current[0] + "' '" + data.current[1] + "' %d" , int(binascii.hexlify(data.current[0]), 16))
    current =  int(binascii.hexlify(data.current[0]), 16)
    if current > 42 and state == 1:
        state = 5
        rospy.loginfo('I am stuck: %d', current)

def buttonCallback(data):
    global state
    global counter
    global previous_state
    rospy.loginfo(rospy.get_caller_id() + ' - Button: "%s" - Value: "%s"', data.button, data.state)
    if data.button == 0 and data.state == 1:
        rospy.loginfo('bobi')
        os.system('espeak bobie')
    if data.button == 1 and data.state == 1:
        #os.system('espeak gtryuio')
        state = 0
        rospy.loginfo('stop')
        os.system('espeak stop')
    if data.button == 2 and data.state == 1:
        state = 1
        rospy.loginfo('go')
        os.system('espeak go')
    if data.button == 2 and data.state == 0:
        rospy.loginfo('release go')
        # os.system('espeak stop')
        # state = 0

def bumperCallback(data):
    global state
    global counter
    global previous_state
    rospy.loginfo(rospy.get_caller_id() + ' - Bumper: "%s" - Value: "%s"', data.bumper, data.state)
    if data.bumper == 1 and data.state == 1 and state == 1:
        counter = 30
        previous_state = state
        state = 4
        rospy.loginfo('center bumper')
        os.system('espeak "center bumper"')
    else:
        if data.bumper == 0 and data.state == 1 and state == 1:
            counter = 20
            previous_state = state
            state = 3
            rospy.loginfo('left bumper')        
            os.system('espeak "left bumper"')
        if data.bumper == 2 and data.state == 1 and state == 1:
            counter = 20
            previous_state = state
            state = 2
            rospy.loginfo('right bumper')
            os.system('espeak "right bumper"')

def listener():
    global state
    global previous_state
    global counter
    os.system('espeak "Hello and how are you Bobie?!" -s 120')
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/mobile_base/events/button', ButtonEvent, buttonCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Subscriber('/mobile_base/sensors/core', SensorState, sensorCallback)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rate = rospy.Rate(15) # 15hz

    stop = Twist()
    stop.linear.x = 0
    stop.linear.y = 0
    stop.linear.z = 0
    stop.angular.x = 0
    stop.angular.y = 0
    stop.angular.z = 0
    pub.publish(stop)

    go = Twist()
    go.linear.x = 0.3 #0.08
    go.linear.y = 0
    go.linear.z = 0
    go.angular.x = 0
    go.angular.y = 0
    go.angular.z = 0

    back = Twist()
    back.linear.x = -0.25#-0.1
    back.linear.y = 0
    back.linear.z = 0
    back.angular.x = 0
    back.angular.y = 0
    back.angular.z = 0

    left = Twist()
    left.linear.x = 0
    left.linear.y = 0
    left.linear.z = 0
    left.angular.x = 0
    left.angular.y = 0
    left.angular.z = 3 #1

    right = Twist()
    right.linear.x = 0
    right.linear.y = 0
    right.linear.z = 0
    right.angular.x = 0
    right.angular.y = 0
    right.angular.z = -3 #-1

    while not rospy.is_shutdown():
        if state == 0:
            pub.publish(stop)
        if state == 1:
            pub.publish(go)
        if state == 2:
            pub.publish(left)
            counter = counter - 1
            if counter <= 0:
                state = previous_state
                counter = 0
        if state == 3:
            pub.publish(right)
            counter = counter - 1
            if counter <= 0:
                state = previous_state
                counter = 0
        if state == 4:
            pub.publish(back)
            counter = counter - 1
            if counter <= 0:
                state = previous_state
                counter = 20
                state = 3
        if state == 5:
            # state = 0;
            counter = 10
            previous_state = 1
            state = 4
            os.system('espeak "I am stuck!"')


        rate.sleep()

if __name__ == '__main__':
    listener()
