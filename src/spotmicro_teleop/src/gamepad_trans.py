#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty
import time

# def srv_setting(relayOn, servoWriteFreq, commandFreq):
#     rospy.wait_for_service('/spotmicro_custom/setting')
#     try:
#         srvHandle_setting = rospy.ServiceProxy('/spotmicro_custom/setting', setting)
#         srvHandle_setting(relayOn, servoWriteFreq, commandFreq)
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

stabilization_on = False;
ypr = [0, 0, 0]
ypr_alpha = 0.5

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

    # map axes
    stick_l_h = data.axes[0]
    stick_l_v = data.axes[1]
    trig_l = data.axes[2]
    stick_r_h = data.axes[3]
    stick_r_v = data.axes[4]
    trig_r = data.axes[5]
    arrow_h = data.axes[6]
    arrow_v = data.axes[7]

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
    if (button_centre == 1):
        if (stabilization_on):
            srv("/spot/deactivateStabilization")
            stabilization_on = False
        else:
            srv("/spot/activateStabilization")
            stabilization_on = True

    cmd_trans = Twist()
    cmd_rot = Twist()
    # if there are translation command
    if ( (abs(stick_l_v) > 0) or (abs(stick_l_h) > 0) ):
        stick_l = (stick_l_v ** 2 + stick_l_h ** 2)**0.5
        if(stick_l > 1):
            cmd_trans.linear.x = stick_l_v / stick_l
            cmd_trans.linear.y = stick_l_h / stick_l
        else:
            cmd_trans.linear.x = stick_l_v
            cmd_trans.linear.y = stick_l_h

        cmd_trans.linear.x *= 0.27
        cmd_trans.linear.y *= 0.27
        pub_trans.publish(cmd_trans)
        
    # if there are rotation command
    elif ( (abs(trig_l) > 0) or (abs(trig_r) > 0) ):
        cmd_rot.angular.z = trig_l - trig_r;
        cmd_rot.angular.z *= 1.5708 
        pub_rot.publish(cmd_rot)
    
    else:
        cmd_trans.linear.x = 0
        cmd_trans.linear.y = 0
        pub_trans.publish(cmd_trans)

        cmd_rot.angular.z = 0
        pub_rot.publish(cmd_rot)

    # ypr command
    if ( (abs(stick_r_v) > 0) or (abs(stick_r_h) > 0) ):
        stick_r = (stick_r_v ** 2 + stick_r_h ** 2)**0.5
        if(stick_r > 1):
            ypr[0] = ypr[0]*(1-ypr_alpha) + (stick_r_h/stick_r * 0.4) * ypr_alpha
            ypr[1] = ypr[1]*(1-ypr_alpha) + (stick_r_v/stick_r * 0.3) * ypr_alpha
        else:
            ypr[0] = ypr[0]*(1-ypr_alpha) + (stick_r_h * 0.4) * ypr_alpha
            ypr[1] = ypr[1]*(1-ypr_alpha) + (stick_r_v * 0.3) * ypr_alpha

        cmd_ypr = Float32MultiArray(data = ypr)
        pub_ypr.publish(cmd_ypr)

if __name__ == "__main__":
    try:
        rospy.init_node('gamepad')
        pub_trans = rospy.Publisher("spot/cmd_vel_translation", Twist, queue_size=1)
        pub_rot = rospy.Publisher("spot/cmd_vel_rotation", Twist, queue_size=1)
        pub_ypr = rospy.Publisher("spot/cmd_ypr", Float32MultiArray, queue_size=10)
        # pub_rot = rospy.Publisher("spot/cmd_vel_rotation", cmd, queue_size=1)
        rospy.Subscriber("joy", Joy, callback)
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass

                
