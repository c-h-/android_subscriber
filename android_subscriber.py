#!/usr/bin/env python
# The MIT License (MIT)
#
# Copyright (c) 2016 Charles Hulcher
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
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
