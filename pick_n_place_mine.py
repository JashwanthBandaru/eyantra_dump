#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from mavros_msgs.msg import ParamValue
from gazebo_ros_link_attacher.srv import Gripper


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('pick_n_place', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
        # Similarly delacre other service proxies 

    def setLand(self,alt):
        rospy.wait_for_service('mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService(altitude = alt)
        except rospy.ServiceException as e:
            print("Service land call failed: %s"%e)   

    def setTakeoff(self,alt):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = alt)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)  

    def offboard_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')  # Waiting untill the service starts 
        try:
            modeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            modeService(custom_mode="OFFBOARD")
        except rospy.ServiceException as e:
            print ("Service set mode failed: %s"%e)

    def setParam(self,param, value):
        rospy.wait_for_service('mavros/param/set')     
        
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)
        try:
            paramService = rospy.ServiceProxy('mavros/param/set', mavros_msgs.srv.ParamSet)
            paramService(param_id=param, value=val)
        except rospy.ServiceException as e:
            print("Service param set failed: %s"%e) 
           
    def gripper(self,value):
        
        rospy.wait_for_service('activate_gripper') 
        try:
            gripperService = rospy.ServiceProxy('activate_gripper', Gripper)
            return gripperService(activate_gripper=value)

        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)


class stateMoniter:
    land_trigger=False

    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers 
    def pose_callback(self,msg1):
        global position_x,position_y,position_z,position_x1,position_y1,position_z1,position_z2
        position_x = round(msg1.pose.position.x)
        position_y = round(msg1.pose.position.y)
        position_z = round(msg1.pose.position.z)

        position_x1 = round(msg1.pose.position.x,1)
        position_y1 = round(msg1.pose.position.y,1)
        position_z1 = round(msg1.pose.position.z,1)

        # position_z2 = round(msg1.pose.position.z,2)

    def gripper_callback(self,msg2):
        global gripper_bool_value
        gripper_bool_value=msg2.data 

def main():

    global position_x, position_y, position_z,position_x1,position_y1,position_z1,position_z2
    global gripper_bool_value
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [[0.0,0.0,3.0],[3.0,0.0,3.0],[3.0,0.0,0.0],[3.0,0.0,3.0],[3.0,3.0,3.0],[3.0,3.0,0.0]] #List to setpoints

    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("mavros/local_position/pose",PoseStamped,stateMt.pose_callback)
    rospy.Subscriber("/gripper_check",String,stateMt.gripper_callback)

    # ofb_ctl.setParam('COM_RCL_EXCEPT',4)
    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
        # print("ARM!!")
    print("Armed!!")

    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":
        local_pos_pub.publish(pos)
        # print("OFFBOARD activating")
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    count=0
    # stateMt.land_trigger=False
    gripper_bool_value=True
    while not rospy.is_shutdown():
        ofb_ctl.gripper(value=True)
        rate.sleep()

    # Publish the setpoints 
    # while not rospy.is_shutdown():

    #     if count==0:
    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
    #         count=1

    #     if count==1 and [position_x,position_y,position_z]==setpoints[0]:
    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
    #         count=2

    #     if count==2 and [position_x,position_y,position_z]==setpoints[1]:
    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[2][0],setpoints[2][1],setpoints[2][2]
    #         count=3
    #         # stateMt.land_trigger=True
        
    #     if count==3 and [position_x1,position_y1,position_z1]==setpoints[2]:
    #         # stateMt.land_trigger=False
    #         while not stateMt.state.guided:
    #             ofb_ctl.setLand(alt=0)
    #             rate.sleep()
    #         print("Landing!!")

    #         print(gripper_bool_value)
    #         print(type(gripper_bool_value))
    #         ofb_ctl.gripper(value=True)
    #         if (gripper_bool_value=="True"):
    #             ofb_ctl.gripper(value=True)
    #             count=4
    #             pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[3][0],setpoints[3][1],setpoints[3][2]
    #         # else:
    #         #     print("Adjusting position!")
    #         #     while not stateMt.state.mode=="OFFBOARD":
    #         #         # local_pos_pub.publish(pos)
    #         #         # print("OFFBOARD activating")
    #         #         ofb_ctl.offboard_set_mode()
    #         #         rate.sleep()
    #         #     print ("OFFBOARD mode activated")
    #         #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
    #         #     count=2

    #     if count==4 and [position_x,position_y,position_z]==setpoints[3]:
    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[4][0],setpoints[4][1],setpoints[4][2]
    #         count=5

    #     if count==5 and [position_x,position_y,position_z]==setpoints[4]:
    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[5][0],setpoints[5][1],setpoints[5][2]
    #         count=6

    #     if count==6 and [position_x1,position_y1,position_z1]==setpoints[5]:
    #         break

    #     local_pos_pub.publish(pos)
    #     rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
