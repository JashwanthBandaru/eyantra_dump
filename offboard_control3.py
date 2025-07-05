#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
#from tf.transformations import quaternion_from_euler
#import math


class offboard_control:


    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    
    def setArm(self, value=True):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(value)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

    def setLand(self):
        rospy.wait_for_service('mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService(altitude = 0)
        except rospy.ServiceException as e:
            print("Service land call failed: %s"%e)               

   
    def offboard_set_mode(self):
        #pass

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode') 
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print ("Service setmode call failed: %s"%e)
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def pose_callback(self,msg1):
        global position_x,position_y,position_z
        #math.trunc(a*10)/10
        position_x = round(msg1.pose.position.x)
        position_y = round(msg1.pose.position.y)
        position_z = round(msg1.pose.position.z)
        


def main():


    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [[0.0,0.0,10.0],[10.0,0.0,10.0],[10.0,10.0,10.0],[0.0,10.0,10.0]] #List to setpoints

    # Similarly initialize other publishers 

    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, stateMt.pose_callback)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":

        for i in range(100):
            local_pos_pub.publish(pos)
            rate.sleep()  

        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    count=0

    # Publish the setpoints 
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        if count==0:
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
            count=1

        if count==1 and [position_x,position_y,position_z]==setpoints[0]:
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
            vel.linear.x=5
        if [position_x,position_y,position_z]==setpoints[1]:
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[2][0],setpoints[2][1],setpoints[2][2]
            vel.linear.x=0 
            vel.linear.y=5 
        if [position_x,position_y,position_z]==setpoints[2]:
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[3][0],setpoints[3][1],setpoints[3][2]
            vel.linear.y=0
            vel.linear.x=-5
        if [position_x,position_y,position_z]==setpoints[3]:
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
            vel.linear.x=0
            vel.linear.y=-5   
            count=2 

        if count==3 and [position_x,position_y,position_z]==setpoints[0]:
            while not stateMt.state.guided:
                ofb_ctl.setLand()
                rate.sleep()
            print("Landing!!")
            break    
        if count==2 and [position_x,position_y,position_z]==setpoints[0]:
            vel.linear.x=0
            vel.linear.y=0 
            count=3
        

        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass