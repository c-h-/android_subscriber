#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Charlie Hulcher
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
#  * Neither the name of Charlie Hulcher nor the names of its
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

## Android subscriber listens for published topics and logs the data

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Temperature

# IMU (Inertial Measurement Unit)
# http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
def imu_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' IMU: %f', data.linear_acceleration.x)
    # To see data about the connection:
    # print data._connection_header

# Barometric Pressure
# http://docs.ros.org/api/sensor_msgs/html/msg/FluidPressure.html
def bp_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' BP: %f', data.fluid_pressure)

# Illuminance
# http://docs.ros.org/api/sensor_msgs/html/msg/Illuminance.html
def ill_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Ill: %f', data.illuminance)

# Magnetic Field
# http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
def mf_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' MF: %f', data.magnetic_field.x)

# Nav Sat Fix
# http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
def nsf_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' NSF: %f', data.longitude)

# Temperature
# http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
def t_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Temp: %f', data.temperature)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('android_listener', anonymous=True)

    rospy.Subscriber('android/imu', Imu, imu_callback)
    rospy.Subscriber('android/barometric_pressure', FluidPressure, bp_callback)
    rospy.Subscriber('android/illuminance', Illuminance, ill_callback)
    rospy.Subscriber('android/magnetic_field', MagneticField, mf_callback)
    rospy.Subscriber('android/fix', NavSatFix, nsf_callback)
    rospy.Subscriber('android/temperature', Temperature, t_callback)

    rospy.loginfo(rospy.get_caller_id() + ' Listening...')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
