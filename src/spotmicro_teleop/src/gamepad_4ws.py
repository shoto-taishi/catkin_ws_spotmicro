#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty
import time

ypr = [0, 0, 0]
L7 = 0.078

cmd_vel_4ws_alpha = 0.99
cmd_ypr_alpha = 0.5

stick_l_h = 0
stick_l_v = 0 
trig_l = 0 
stick_r_h = 0 
stick_r_v = 0
trig_r = 0 

def srv(srv_name):
    rospy.wait_for_service(srv_name)
    try:
        srvHandle = rospy.ServiceProxy(srv_name, Empty)
        srvHandle()
        time.sleep(2.5)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def callback(data):

    global ypr, ypr_alpha, stabilization_on

    global stick_l_h
    global stick_l_v 
    global trig_l 
    global stick_r_h 
    global stick_r_v
    global trig_r 

    # map axes
    stick_l_h = stick_l_h * (1-cmd_vel_4ws_alpha) + data.axes[0] * cmd_vel_4ws_alpha
    stick_l_v = stick_l_v * (1-cmd_vel_4ws_alpha) + data.axes[1] * cmd_vel_4ws_alpha
    trig_l = trig_l * (1-cmd_vel_4ws_alpha) + data.axes[2] * cmd_vel_4ws_alpha
    stick_r_h = stick_r_h * (1-cmd_ypr_alpha) + data.axes[3] * cmd_ypr_alpha
    stick_r_v = stick_r_v * (1-cmd_ypr_alpha) + data.axes[4] * cmd_ypr_alpha
    trig_r = trig_r * (1-cmd_vel_4ws_alpha) + data.axes[5] * cmd_vel_4ws_alpha

    arrow_h = data.axes[6]
    arrow_v = data.axes[7]

    # rounding
    # stick_l_h = round(stick_l_h, 3)
    # stick_l_v = round(stick_l_v, 3)
    # trig_l = round(trig_l, 3)
    # stick_r_h = round(stick_r_h, 3)
    # stick_r_v = round(stick_r_v, 3)
    # trig_r = round(trig_r, 3)

    # map buttons
    button_a = data.buttons[0]
    button_b = data.buttons[1]
    button_x = data.buttons[2]
    button_y = data.buttons[3]
    button_l = data.buttons[4]
    button_r = data.buttons[5]
    button_back = data.buttons[6]
    button_start = data.buttons[7]
    button_centre = data.buttons[8]
    button_stick_l = data.buttons[9]
    button_stick_r = data.buttons[10]

    # remap left/right trigger (map to 0-1)
    trig_l = (1 - trig_l)/2
    trig_r = (1 - trig_r)/2

    # services
    if (button_start == 1):
        srv("/spot/arm")
    if (button_back == 1):
        srv("/spot/disarm")
    if (button_y == 1):
        srv("/spot/stand")
    if (button_a == 1):
        srv("/spot/home")

    cmd_4ws = Twist()
    
    cmd_4ws.angular.z = trig_l - trig_r
    cmd_4ws.angular.z *= 3.1416

    cmd_4ws.angular.z = round(cmd_4ws.angular.z, 2)

    cmd_4ws.linear.x = stick_l_v
    cmd_4ws.linear.x *= 0.27

    cmd_4ws.linear.x = round(cmd_4ws.linear.x, 2)

    if ((abs(cmd_4ws.angular.z) > 0) or (abs(cmd_4ws.linear.x) > 0)):

        if (cmd_4ws.angular.z == 0):
            rotation_radius = 999999
        else:
            rotation_radius = cmd_4ws.linear.x / cmd_4ws.angular.z

        # need to check |R| > L7/2        
        if ( (0 <= rotation_radius) and (rotation_radius < L7/2 ) ):
            rotation_radius = L7/2
            cmd_4ws.linear.x = cmd_4ws.angular.z * rotation_radius

        if ( (-L7/2 < rotation_radius) and (rotation_radius <= 0 ) ):
            rotation_radius = L7/2
            cmd_4ws.linear.x = cmd_4ws.angular.z * rotation_radius
        
        pub_4ws.publish(cmd_4ws)
    
    # ypr command
    if ( (abs(stick_r_v) > 0) or (abs(stick_r_h) > 0) ):
        stick_r = (stick_r_v ** 2 + stick_r_h ** 2)**0.5
        if(stick_r > 1):
            ypr[0] = (stick_r_h/stick_r * 0.4)
            ypr[1] = (stick_r_v/stick_r * 0.3)
        else:
            ypr[0] = (stick_r_h * 0.4)
            ypr[1] = (stick_r_v * 0.3)

        cmd_ypr = Float32MultiArray(data = ypr)
        pub_ypr.publish(cmd_ypr)

if __name__ == "__main__":
    try:
        rospy.init_node('gamepad_4ws')
        pub_4ws = rospy.Publisher("spot/cmd_vel_4ws", Twist, queue_size=1)
        pub_ypr = rospy.Publisher("spot/cmd_ypr", Float32MultiArray, queue_size=10)
        # pub_rot = rospy.Publisher("spot/cmd_vel_rotation", cmd, queue_size=1)
        rospy.Subscriber("joy", Joy, callback)
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass

                
